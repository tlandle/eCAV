# -*- coding: utf-8 -*-
"""
Script to run a simulated vehicle
"""

# Authors: Aaron Drysdale <adrysdale3@gatech.edu>
#        : Jordan Rapp <jrapp7@gatech.edu>


import argparse
import sys
import json
import asyncio
import os
import logging
import threading
import time
import queue

import carla
import numpy as np
import coloredlogs
import pickle

from opencda.version import __version__
from opencda.core.common.cav_world import CavWorld
from opencda.core.common.vehicle_manager import VehicleManager
from opencda.scenario_testing.utils.yaml_utils import load_yaml
from opencda.core.application.edge.networking import NetworkEmulator
from opencda.core.common.ecloud_config import EcloudConfig, eDoneBehavior, eLocationType
from opencda.ecloud_server.ecloud_comms import EcloudClient, ecloud_run_push_server

import grpc
from google.protobuf.json_format import MessageToJson
from google.protobuf.timestamp_pb2 import Timestamp

import ecloud_pb2 as ecloud
import ecloud_pb2_grpc as ecloud_rpc

logger = logging.getLogger(__name__)
coloredlogs.install(level='DEBUG', logger=logger)
logger.setLevel(logging.DEBUG)

# TODO: move to eCloudConfig
cloud_config = load_yaml("cloud_config.yaml")
CARLA_IP = cloud_config["carla_server_public_ip"]
ECLOUD_IP = cloud_config["ecloud_server_public_ip"]
VEHICLE_IP = cloud_config["vehicle_client_public_ip"]
ECLOUD_PUSH_BASE_PORT = 50101 # TODO: config

class Ecav2VehicleClient:

    #TODO: move to eCloudConfig
    # default params which can be over-written from the simulation controller
    SPECTATOR_INDEX = 0

    def __init__(self, ecav_vehicle_index):
        self.ecloud_server = None
        self.channel = None
        self.vehicle_index = ecav_vehicle_index
        self.actor_id = None
        self.push_port = None
        self.push_server = None
        self.vid = None
        self.pong = None
        self.vehicle_manager = None
        self.network_emulator = None

        self.target_speed = None
        self.is_edge = False # TODO: added this to the actual protobuf message
        self.network_emulator = None
        self.edge_sets_destination = False
        self.done_behavior = eDoneBehavior.CONTROL
        self.location_type = eLocationType.EXPLICIT
        self.verbose_updates = False

        if cloud_config["log_level"] == "error":
            logger.setLevel(logging.ERROR)
        elif cloud_config["log_level"] == "warning":
            logger.setLevel(logging.WARNING)
        elif cloud_config["log_level"] == "info":
            logger.setLevel(logging.INFO)

        self.application = ["single"]
        self.version = "0.9.15"
        self.tick_id = 0
        self.reported_done = False
        self.push_q = asyncio.Queue()

        self.opt = self.arg_parse()
        if self.opt.verbose:
            logger.setLevel(logging.DEBUG)
        elif self.opt.quiet:
            logger.setLevel(logging.WARNING)
        logger.info("OpenCDA Version: %s", self.version)

        logging.basicConfig()

    async def run(self):
        assert self.vehicle_index is not None
        ecloud_update = await self.connect()
        assert ecloud_update is not None, "ecloud_update not received"
        test_scenario = ecloud_update.test_scenario
        application = ecloud_update.application
        version = ecloud_update.version

        logger.debug("main - application: %s", application)
        logger.debug("main - version: %s", version)

        # create CAV world
        cav_world = CavWorld(self.opt.apply_ml)

        logger.info("eCloud debug: creating VehicleManager vehicle_index: %s", self.vehicle_index)

        scenario_yaml = json.loads(test_scenario) #load_yaml(test_scenario)
        if 'debug_scenario' in scenario_yaml:
            logger.debug("main - test_scenario: %s", test_scenario)

        ecloud_config = EcloudConfig(scenario_yaml, logger)
        
        SPAWN_SLEEP_TIME = ecloud_config.get_client_spawn_ping_time_s()
        TICK_SLEEP_TIME = ecloud_config.get_client_tick_ping_time_s()
        WORLD_TIME_SLEEP_FACTOR = ecloud_config.get_client_world_tick_factor()
        NUM_SERVERS = ecloud_config.get_num_servers()
        NUM_PORTS = ecloud_config.get_num_ports()

        self.location_type = ecloud_config.get_location_type()
        self.done_behavior = ecloud_config.get_done_behavior()

        self.verbose_updates = ecloud_config.do_verbose_update()
        if 'edge_list' in scenario_yaml['scenario']:
            self.is_edge = True
            # TODO: support multiple edges...
            #self.target_speed = scenario_yaml['scenario']['edge_list'][0]['target_speed']
            self.target_speed = scenario_yaml['scenario']['edge_list'][0]['members'][int(f'{self.vehicle_index}')]['behavior']['max_speed']
            print(self.target_speed)
            time.sleep(5)
            self.edge_sets_destination = scenario_yaml['scenario']['edge_list'][0]['edge_sets_destination'] \
                if 'edge_sets_destination' in scenario_yaml['scenario']['edge_list'][0] else False

        if self.opt.apply_ml:
            await asyncio.sleep(self.vehicle_index + 1)

        self.vehicle_manager = VehicleManager(vehicle_index=self.vehicle_index, config_yaml=scenario_yaml, application=application, cav_world=cav_world, \
                                        carla_version=version, location_type=self.location_type, run_distributed=True, is_edge=self.is_edge, perception_active=self.opt.apply_ml)

        if self.is_edge:
            self.network_emulator = NetworkEmulator(edge_sets_destination=self.edge_sets_destination,
                                                vehicle_manager=self.vehicle_manager)

        assert self.vehicle_manager.vehicle is not None, "vehicle_manager failed to spawn the vehicle"
        self.actor_id = self.vehicle_manager.vehicle.id
        self.vid = self.vehicle_manager.vid

        await self.send_carla_data_to_opencda()

        assert self.push_q.empty(), logger.exception("push_q had %s in it when it should have been empty", self.push_q.get_nowait())
        self.pong = await self.push_q.get()
        self.push_q.task_done()

        self.vehicle_manager.update_info()
        self.vehicle_manager.set_destination(
                    self.vehicle_manager.vehicle.get_location(),
                    self.vehicle_manager.destination_location,
                    clean=True)

    async def connect(self) -> ecloud.SimulationInfo:
        # spawn push server
        self.push_port = ECLOUD_PUSH_BASE_PORT + self.opt.vehicle_index
        self.push_server = asyncio.create_task(ecloud_run_push_server(self.push_port, self.push_q))

        await asyncio.sleep(1)

        push_port = await self.push_q.get() # make sure we get the actual port - try logic may have altered it.
        self.push_q.task_done()

        logger.info("push server spun up on port %s", push_port)

        # TODO: move to eCloudClient
        self.channel = grpc.aio.insecure_channel(
            target=f"{ECLOUD_IP}:{self.opt.port}",
            options=[
                ("grpc.lb_policy_name", "pick_first"),
                ("grpc.enable_retries", 1),
                ("grpc.keepalive_timeout_ms", 10000),
                ("grpc.service_config", EcloudClient.retry_opts),],
            )
        self.ecloud_server = ecloud_rpc.EcloudStub(self.channel)

        ecloud_update = await self.send_registration_to_ecloud_server()
        self.vehicle_index = ecloud_update.vehicle_index
        assert self.vehicle_index is not None, "vehicle_index not set by ecloud server"
        
        return ecloud_update

    async def close(self):
        assert self.channel is not None, "channel not initialized"
        await self.channel.close()

    #TODO: move to eCloudClient
    def serialize_debug_info(self, vehicle_update, vehicle_manager) -> None:
        planer_debug_helper = vehicle_manager.agent.debug_helper
        planer_debug_helper_msg = ecloud.PlanerDebugHelper()
        planer_debug_helper.serialize_debug_info(planer_debug_helper_msg)
        vehicle_update.planer_debug_helper.CopyFrom( planer_debug_helper_msg )

        loc_debug_helper = vehicle_manager.localizer.debug_helper
        loc_debug_helper_msg = ecloud.LocDebugHelper()
        loc_debug_helper.serialize_debug_info(loc_debug_helper_msg)
        vehicle_update.loc_debug_helper.CopyFrom( loc_debug_helper_msg )

        client_debug_helper = vehicle_manager.debug_helper
        #logger.debug(vehicle_manager.debug_helper.perception_time_list)
        client_debug_helper_msg = ecloud.ClientDebugHelper()
        client_debug_helper.serialize_debug_info(client_debug_helper_msg)
        vehicle_update.client_debug_helper.CopyFrom(client_debug_helper_msg)

    #TODO: move to eCloudClient
    async def send_registration_to_ecloud_server(self) -> ecloud.SimulationInfo:
        request = ecloud.RegistrationInfo()
        request.vehicle_state = ecloud.VehicleState.REGISTERING
        try:
            request.container_name = os.environ["HOSTNAME"]
        except Exception as e:
            request.container_name = f"vehiclesim.py"

        request.vehicle_ip = VEHICLE_IP
        request.vehicle_port = self.push_port

        assert self.ecloud_server is not None, "stub not initialized"
        sim_info = await self.ecloud_server.Client_RegisterVehicle(request)

        logger.info("vehicle ID %s received...", sim_info.vehicle_index)

        return sim_info

    #TODO: move to eCloudClient
    async def send_carla_data_to_opencda(self,) -> ecloud.SimulationInfo:
        assert self.ecloud_server is not None, "stub not initialized"
        message = {"vehicle_index": self.vehicle_index, "actor_id": self.actor_id, "vid": self.vid}
        logger.info("Vehicle: Sending Carla rpc %s", message)

        # send actor ID and vid to API
        update = ecloud.RegistrationInfo()
        update.vehicle_state = ecloud.VehicleState.CARLA_UPDATE
        update.vehicle_index = self.vehicle_index
        update.vid = self.vid
        update.actor_id = self.actor_id

        sim_info = await self.ecloud_server.Client_RegisterVehicle(update)

        logger.info("send_carla_data_to_opencda: response received")

        return sim_info

    #TODO: move to eCloudClient
    async def send_vehicle_update(self, vehicle_update_):
        assert self.ecloud_server is not None, "stub not initialized"
        logger.debug("send_vehicle_update: sending")
        empty = await self.ecloud_server.Client_SendUpdate(vehicle_update_)
        logger.debug("send_vehicle_update: send complete")
        return empty

    def arg_parse(self):
        parser = argparse.ArgumentParser(description="OpenCDA Vehicle Simulation.")
        parser.add_argument("--apply_ml",
                            action='store_true',
                            help='whether ml/dl framework such as sklearn/pytorch is needed in the testing. '
                                'Set it to true only when you have installed the pytorch/sklearn package.')
        parser.add_argument('-a', "--ipaddress", type=str, default=CARLA_IP,
                            help="Specifies the ip address of the server to connect to. [Default: localhost]")
        parser.add_argument('-p', "--port", type=int, default=50051,
                            help="Specifies the port to connect to. [Default: 50051]")
        parser.add_argument("--verbose", action="store_true",
                            help="Make more noise")
        parser.add_argument('-q', "--quiet", action="store_true",
                            help="Make no noise")
        parser.add_argument('-t', "--test_scenario", required=True, type=str,
                            help='Define the name of the scenario you want to test. The given name must'
                             'match one of the testing scripts(e.g. single_2lanefree_carla) in '
                             'opencda/scenario_testing/ folder'
                             ' as well as the corresponding yaml file in opencda/scenario_testing/config_yaml.')
        parser.add_argument("--record", action='store_true',
                            help='whether to record and save the simulation process to .log file')
        parser.add_argument('-v', "--version", type=str, default='0.9.15',
                            help='Specify the CARLA simulator version, default'
                                'is 0.9.15')
        parser.add_argument("--build", action="store_true",
                            help="Rebuild gRPC proto files")
        parser.add_argument('-i', "--vehicle_index", type=int, default=-1,
                            help='Specify the vehicle index, default is -1')
        parser.add_argument("--output_dir", default=None)

        opt = parser.parse_args()
        return opt

    async def tick(self):
        assert self.pong is not None, "pong not initialized"
        assert self.vehicle_manager is not None, "vehicle_manager not initialized"

        logger.info("vehicle %s beginning scenario tick flow", self.vehicle_index)
        
        vehicle_update = ecloud.VehicleUpdate()
        
        if self.pong.command != ecloud.Command.TICK: # don't print tick message since there are too many
            logger.info("Vehicle: received cmd %s", self.pong.command)

        # HANDLE DEBUG DATA REQUEST
        if self.pong.command == ecloud.Command.REQUEST_DEBUG_INFO:
            vehicle_update.vehicle_state = ecloud.VehicleState.DEBUG_INFO_UPDATE
            self.serialize_debug_info(vehicle_update, self.vehicle_manager)

        # HANDLE TICK
        elif self.pong.command == ecloud.Command.TICK:
            client_start_timestamp = Timestamp()
            client_start_timestamp.GetCurrentTime()
            # update info runs BEFORE waypoint injection
            update_info_start_time = time.time()
            self.vehicle_manager.update_info()
            update_info_end_time = time.time()
            self.vehicle_manager.debug_helper.update_update_info_time((update_info_end_time-update_info_start_time)*1000)
            logger.debug("update_info complete")

            if self.is_edge:
                self.network_emulator.update_waypoints()

            if self.reported_done:
                self.target_speed = 0
            control = self.vehicle_manager.run_step(target_speed=self.target_speed)
            logger.debug("run_step complete")

            vehicle_update.tick_id = self.tick_id

            if control is None or self.vehicle_manager.is_close_to_scenario_destination():
                vehicle_update.vehicle_state = ecloud.VehicleState.TICK_DONE
                if not self.reported_done:
                    self.serialize_debug_info(vehicle_update, self.vehicle_manager)

                if control is not None and self.done_behavior == eDoneBehavior.CONTROL:
                    self.vehicle_manager.apply_control(control)

            else:
                self.vehicle_manager.apply_control(control)
                logger.debug("apply_control complete")

                step_timestamps = ecloud.Timestamps()
                step_timestamps.tick_id = self.tick_id
                step_timestamps.client_end_tstamp.GetCurrentTime()
                step_timestamps.client_start_tstamp.CopyFrom(client_start_timestamp)
                self.vehicle_manager.debug_helper.update_timestamp(step_timestamps)

                vehicle_update.vehicle_state = ecloud.VehicleState.TICK_OK
                vehicle_update.duration_ns = step_timestamps.client_end_tstamp.ToNanoseconds() - step_timestamps.client_start_tstamp.ToNanoseconds()

            if self.is_edge or self.vehicle_index == eCloudClient.SPECTATOR_INDEX or self.verbose_updates:
                velocity = self.vehicle_manager.vehicle.get_velocity()
                pv = ecloud.Velocity()
                pv.x = velocity.x
                pv.y = velocity.y
                pv.z = velocity.z
                vehicle_update.velocity.CopyFrom(pv)

                transform = self.vehicle_manager.vehicle.get_transform()
                pt = ecloud.Transform()
                pt.location.x = transform.location.x
                pt.location.y = transform.location.y
                pt.location.z = transform.location.z
                pt.rotation.roll = transform.rotation.roll
                pt.rotation.yaw = transform.rotation.yaw
                pt.rotation.pitch = transform.rotation.pitch
                vehicle_update.transform.CopyFrom(pt)

            # vehicle_update.vehicle_state = ecloud.VehicleState.ERROR # TODO: handle error status
            # logger.error("ecloud_client error")

            #cur_location = vehicle_manager.vehicle.get_location()
            #logger.debug("send OK and location for vehicle_%s - is - x: %s, y: %s", vehicle_index, cur_location.x, cur_location.y)

        # block waiting for a response
        if not self.reported_done or self.done_behavior == eDoneBehavior.CONTROL:
            if not self.reported_done:
                vehicle_update.tick_id = self.tick_id
                vehicle_update.vehicle_index = self.vehicle_index
                logger.debug('vehicle_update: \n vehicle_index: %s \n tick_id: %s \n %s', self.vehicle_index, self.tick_id, vehicle_update)
                await self.send_vehicle_update( vehicle_update)

            if vehicle_update.vehicle_state == ecloud.VehicleState.TICK_DONE or vehicle_update.vehicle_state == ecloud.VehicleState.DEBUG_INFO_UPDATE:
                if vehicle_update.vehicle_state == ecloud.VehicleState.DEBUG_INFO_UPDATE and self.pong.command == ecloud.Command.REQUEST_DEBUG_INFO:
                    # we were asked for debug data and provided it, so NOW we exit
                    # TODO: this is better handled by done
                    logger.info("pushed DEBUG_INFO_UPDATE")

                self.reported_done = True
                logger.info("reported_done")

            assert self.push_q.empty(), logger.exception("push_q had %s in it when it should have been empty", self.push_q.get_nowait())
            self.pong = await self.push_q.get()
            self.push_q.task_done()
            assert( self.pong.tick_id != self.tick_id )
            self.tick_id = self.pong.tick_id

            if self.pong.command == ecloud.Command.PULL_WAYPOINTS_AND_TICK:
                wp_request = ecloud.WaypointRequest()
                wp_request.vehicle_index = self.vehicle_index
                waypoint_proto = await self.ecloud_server.Client_GetWaypoints(wp_request)
                self.network_emulator.enqueue_wp(waypoint_proto)
                self.pong.command = ecloud.Command.TICK

            elif self.pong.command == ecloud.Command.PULL_OBJECTS_AND_TICK:
                obj_request = ecloud.ObjectRequest()
                obj_request.vehicle_index = self.vehicle_index

                object_proto = await self.ecloud_server.Client_GetObjects(obj_request)
                self.network_emulator.enqueue_obj(object_proto)
                self.pong.command = ecloud.Command.TICK

            # HANDLE END
            elif self.pong.command == ecloud.Command.END:
                logger.critical("END received")
                #break

        else: # done
            logger.info("EXIT destroy-on-done vehicle actor")
            #break

        return self.pong

    async def end(self):
        self.vehicle_manager.destroy()
        self.push_server.cancel()
        logger.info("scenario complete.")

