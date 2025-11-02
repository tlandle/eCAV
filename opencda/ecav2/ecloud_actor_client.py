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

from opencda.core.common import vehicle_manager
from opencda.version import __version__
from opencda.core.common.cav_world import CavWorld
from opencda.core.common.vehicle_manager import VehicleManager
from opencda.core.common.rsu_manager import RSUManager
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

if cloud_config["log_level"] == "error":
    logger.setLevel(logging.ERROR)
elif cloud_config["log_level"] == "warning":
    logger.setLevel(logging.WARNING)
elif cloud_config["log_level"] == "info":
    logger.setLevel(logging.INFO)

class Ecav2ActorClient:

    #TODO: move to eCloudConfig
    # default params which can be over-written from the simulation controller
    SPECTATOR_INDEX = 0

    def __init__(self, vehicle=None, actor_type=ecloud.ActorType.VEHICLE):
        self.actor_type = actor_type
        
        self.ecloud_server = None
        self.channel = None
        self.vehicle = vehicle
        self.vehicle_index = 0 # TODO: fix me
        self.actor_id = None
        self.push_port = None
        self.push_server = None
        self.vid = None
        self.pong = None
        self.vehicle_manager = None
        self.network_emulator = None
        self.rsu_manager = None

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
        self.tick_id = 1
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
            time.sleep(5)
            self.edge_sets_destination = scenario_yaml['scenario']['edge_list'][0]['edge_sets_destination'] \
                if 'edge_sets_destination' in scenario_yaml['scenario']['edge_list'][0] else False

        if self.opt.apply_ml:
            await asyncio.sleep(self.vehicle_index + 1)

        if self.actor_type == ecloud.ActorType.VEHICLE:
            self.vehicle_manager = VehicleManager(vehicle=self.vehicle, config_yaml=scenario_yaml, application=application, cav_world=cav_world, \
                                        carla_version=version, location_type=self.location_type, run_distributed=True, is_edge=self.is_edge, perception_active=self.opt.apply_ml)
            assert self.vehicle_manager.vehicle is not None, "vehicle_manager failed to spawn the vehicle"
            self.actor_id = self.vehicle_manager.vehicle.id
            self.vid = self.vehicle_manager.vid
        elif self.actor_type == ecloud.ActorType.RSU:
            rsu_config = scenario_yaml['scenario']['edge_list'][0]['rsus'][self.vehicle_index]
            client = carla.Client(CARLA_IP, scenario_yaml['world']['client_port'])
            client.set_timeout(10.0)
            world = client.get_world()
            carla_map = world.get_map()
            cav_world = CavWorld(self.opt.apply_ml)
            self.rsu_manager = RSUManager(world, 
                                rsu_config,
                                carla_map,
                                cav_world,
                                scenario_yaml['current_time'],
                                data_dumping=self.verbose_updates)
            self.vid = scenario_yaml['scenario']['edge_list'][0]['rsus'][self.vehicle_index]['name']
            self.actor_id = scenario_yaml['scenario']['edge_list'][0]['rsus'][self.vehicle_index]['id']
            self.vehicle_manager = self.rsu_manager # for common handling below

        if self.is_edge:
            self.network_emulator = NetworkEmulator(edge_sets_destination=self.edge_sets_destination,
                                                vehicle_manager=self.vehicle_manager)

        await self.send_carla_data_to_opencda()

        logger.info("send_carla_data_to_opencda completed")

        assert self.push_q.empty(), logger.exception("push_q had %s in it when it should have been empty", self.push_q.get_nowait())
        self.pong = await self.push_q.get()
        self.push_q.task_done()

        logger.info("pong received")

        self.vehicle_manager.update_info()
        if self.actor_type == ecloud.ActorType.VEHICLE:
            self.vehicle_manager.set_destination(
                    self.vehicle_manager.vehicle.get_location(),
                    self.vehicle_manager.destination_location,
                    clean=True)
        
        logger.info("udpate_info & set_destination complete")

    async def connect(self) -> ecloud.SimulationInfo:
        # spawn push server
        self.push_port = ECLOUD_PUSH_BASE_PORT + self.opt.vehicle_index + ( 100 if self.actor_type == ecloud.ActorType.RSU else 0 )
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
        self.vehicle_index = ecloud_update.vehicle_index - ( 1 if self.actor_type == ecloud.ActorType.RSU else 0 )
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
        parser.add_argument('-d', "--distributed", action="store_true", default=True,
                            help="Enable distributed mode")

        opt = parser.parse_args()
        return opt

    async def tick(self):
        assert self.pong is not None, "pong not initialized"
        assert self.vehicle_manager is not None, "vehicle_manager not initialized"

        logger.info("vehicle %s beginning scenario tick flow", self.vehicle_index)
        
        vehicle_update = ecloud.VehicleUpdate()
        
        #if self.pong.command != ecloud.Command.TICK: # don't print tick message since there are too many
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
            logger.info("update_info complete")
            
            control = self.vehicle_manager.run_step()
            logger.debug("Applying control for vehicle manager: %s", self.vehicle_manager)
            if self.actor_type == ecloud.ActorType.VEHICLE:
                self.vehicle_manager.vehicle.apply_control(control)

                update_info_end_time = time.time()
                self.vehicle_manager.debug_helper.update_update_info_time((update_info_end_time-update_info_start_time)*1000)
                
                if self.is_edge:
                    self.network_emulator.update_waypoints()

            logger.info("run_step complete")

            vehicle_update.actor_type = self.actor_type
            vehicle_update.tick_id = self.tick_id
            try:
                if self.actor_type == ecloud.ActorType.VEHICLE:
                    vehicle_update.pickled_agent_objects = pickle.dumps(self.vehicle_manager.agent.objects)
                else:
                    vehicle_update.pickled_agent_objects = pickle.dumps(self.rsu_manager.objects)
            except Exception as e:
                print(f"Error serializing objects: {e}", flush=True)
                def find_unpicklable(obj, path=""):
                    try:
                        pickle.dumps(obj)
                        return None  # Object is picklable
                    except Exception as e:
                        print(f"Failed to pickle {path}: {e}")
                        if hasattr(obj, '__dict__'):
                            for key, value in obj.__dict__.items():
                                result = find_unpicklable(value, f"{path}.{key}")
                                if result is not None:
                                    return result  # Found the unpicklable item
                        return obj  # This object itself is unpicklable
                if self.actor_type == ecloud.ActorType.VEHICLE:
                    for o in self.vehicle_manager.agent.objects:
                        print(find_unpicklable(o, path=f"preds[{type(o).__name__}]"), flush=True)
                else:
                    for o in self.rsu_manager.objects:
                        print(find_unpicklable(o, path=f"preds[{type(o).__name__}]"), flush=True)

            if self.actor_type == ecloud.ActorType.VEHICLE:
                vehicle_update.vehicle_state = ecloud.VehicleState.TICK_OK if not self.vehicle_manager.is_close_to_scenario_destination() else ecloud.VehicleState.TICK_DONE
            else:
                vehicle_update.vehicle_state = ecloud.VehicleState.TICK_OK # RSUs never "done"

            # vehicle_update.vehicle_state = ecloud.VehicleState.ERROR # TODO: handle error status
            # logger.error("ecloud_client error")

            #cur_location = vehicle_manager.vehicle.get_location()
            #logger.debug("send OK and location for vehicle_%s - is - x: %s, y: %s", vehicle_index, cur_location.x, cur_location.y)

        # block waiting for a response
        if not self.reported_done or self.done_behavior == eDoneBehavior.CONTROL:
            if not self.reported_done:
                vehicle_update.tick_id = self.tick_id
                vehicle_update.vehicle_index = self.vehicle_index
                logger.info('vehicle_update: \n vehicle_index: %s \n tick_id: %s \n %s', self.vehicle_index, self.tick_id, vehicle_update)
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
                def recursive_print_object(obj, indent=0, visited=None):
                    if visited is None:
                        visited = set()

                    # Prevent infinite recursion for circular references
                    if id(obj) in visited:
                        print(f"{'  ' * indent}<Circular Reference to {type(obj).__name__} object at {hex(id(obj))}>")
                        return
                    visited.add(id(obj))

                    print(f"{'  ' * indent}{type(obj).__name__} object at {hex(id(obj))}:")
                    indent += 1

                    if isinstance(obj, dict):
                        for key, value in obj.items():
                            print(f"{'  ' * indent}{key}:")
                            recursive_print_object(value, indent + 1, visited)
                    elif isinstance(obj, list):
                        for i, item in enumerate(obj):
                            print(f"{'  ' * indent}[{i}]:")
                            recursive_print_object(item, indent + 1, visited)
                    elif hasattr(obj, '__dict__'):
                        for attr_name, attr_value in obj.__dict__.items():
                            if hasattr(attr_value, '__dict__'):  # Check if the attribute is another object
                                print(f"{'  ' * indent}{attr_name}:")
                                recursive_print_object(attr_value, indent + 1, visited)
                            else:
                                print(f"{'  ' * indent}{attr_name}: {attr_value}")
                    else:
                        print(f"{'  ' * indent}{obj}")

                preds = pickle.loads(object_proto.pickled_edge_predictions) if object_proto.pickled_edge_predictions else None
                print("Edge Predictions:")
                # recursive_print_object(preds)
                if self.actor_type == ecloud.ActorType.VEHICLE:
                    self.vehicle_manager.agent.edge_predictions = preds
                else:
                    pass # don't have a scenario yet where the RSU gets edge predictions
                self.pong.command = ecloud.Command.TICK

            # HANDLE END
            elif self.pong.command == ecloud.Command.END:
                logger.critical("END received")

        else: # done
            logger.info("EXIT destroy-on-done vehicle actor")

        return self.pong

    async def end(self):
        self.vehicle_manager.destroy()
        self.push_server.cancel()
        logger.info("scenario complete.")

if __name__ == '__main__':
    try:
        loop = asyncio.get_event_loop()
        ecav_client = Ecav2ActorClient(actor_type=ecloud.ActorType.RSU) # RSUs are created as standalone processes; vehicles are created by scenario manager
        asyncio.get_event_loop().run_until_complete(ecav_client.run())
        while True:
            pong = asyncio.get_event_loop().run_until_complete(ecav_client.tick())
            if pong.command == ecloud.Command.END:
                break
            time.sleep(0.001)
        asyncio.get_event_loop().run_until_complete(ecav_client.end())

    except KeyboardInterrupt:
        logger.info("caught keyboard interrupt")

    except Exception as err: # pylint: disable=broad-exception-caught
        logger.exception("exception hit: %s - %s", type(err), err)
        if EcloudConfig.fatal_errors:
            raise