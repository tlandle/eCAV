# -*- coding: utf-8 -*-
# Authors: Aaron Drysdale <adrysdale3@gatech.edu>
#          Jordan Rapp <jrapp7@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib
"""
Script to run a simulated vehicle
"""


import json
import asyncio
import os
import logging
import time
import sys

import carla
import pickle

sys.path.insert(0,'/opt/carla-simulator/PythonAPI/carla')
sys.path.insert(0, os.path.join(os.getcwd(), 'ecav'))
sys.path.insert(0, os.path.join(os.getcwd(), 'ecav', 'protos'))
sys.path.insert(0, os.path.join(os.getcwd(), 'scenario_runner'))
sys.path.insert(0, os.getcwd())

from ecav.version import __version__
from ecav.ecav2.arg_utils import build_arg_parser
from ecav.utils import find_unpicklable
from ecav.core.common.cav_world import CavWorld
from ecav.core.common.vehicle_manager import VehicleManager
from ecav.core.common.rsu_manager import RSUManager
from ecav.scenario_testing.utils.yaml_utils import load_yaml
from ecav.core.application.edge.networking import NetworkEmulator
from ecav.core.common.ecloud_config import EcloudConfig, eDoneBehavior, eLocationType
from ecav.ecloud_server.ecloud_comms import EcloudClient, ecloud_run_push_server

import grpc
from google.protobuf.json_format import MessageToJson
from google.protobuf.timestamp_pb2 import Timestamp

import ecloud_pb2 as ecloud
import ecloud_pb2_grpc as ecloud_rpc

logger = logging.getLogger(__name__)

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

    def __init__(self, vehicle=None, actor_type=ecloud.ActorType.VEHICLE, vehicle_index=0):
        self.actor_type = actor_type

        self.ecloud_server = None
        self.channel = None
        self.vehicle = vehicle
        self.vehicle_index = vehicle_index
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

        # Edge connection tracking (new distributed edge architecture)
        self.connected_to_edge = False
        self.edge_channel = None
        self.edge_stub = None
        self.edge_ip = None
        self.edge_port = None
        self.edge_index = None

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
        logger.info("eCAV Version: %s", self.version)

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
        # litserve=True offloads ML inference to LitServe server on port 18000
        cav_world = CavWorld(self.opt.apply_ml, litserve=self.opt.litserve)

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
            self.vehicle_manager = VehicleManager(vehicle=self.vehicle, config_yaml=scenario_yaml, vehicle_index=self.vehicle_index, application=application, cav_world=cav_world, \
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
            cav_world = CavWorld(self.opt.apply_ml, litserve=self.opt.litserve)
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

        await self.send_carla_data_to_ecav()

        assert self.push_q.empty(), f"push_q should be empty before first tick — had item already"
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
        self.push_port = ECLOUD_PUSH_BASE_PORT + self.opt.vehicle_index + ( 100 if self.actor_type == ecloud.ActorType.RSU else 0 ) # TODO: needs to count number of vehicles so that we can then index RSUs properly
        self.push_server = asyncio.create_task(ecloud_run_push_server(self.push_port, self.push_q))

        await asyncio.sleep(1)

        push_port = await self.push_q.get() # make sure we get the actual port - try logic may have altered it.
        self.push_q.task_done()

        logger.info("push server spun up on port %s", push_port)

        # First connect to orchestrator to get connection info
        self.channel = grpc.aio.insecure_channel(
            target=f"{ECLOUD_IP}:50051",
            options=[
                ("grpc.lb_policy_name", "pick_first"),
                ("grpc.enable_retries", 1),
                ("grpc.keepalive_timeout_ms", 10000),
                ("grpc.max_send_message_length", 200 * 1024 * 1024),
                ("grpc.max_receive_message_length", 200 * 1024 * 1024),
                ("grpc.service_config", EcloudClient.retry_opts),],
            )
        self.ecloud_server = ecloud_rpc.EcloudStub(self.channel)

        # Query orchestrator for connection info (new edge architecture)
        connection_info = await self.get_connection_info()

        if connection_info and connection_info.has_edge:
            # Connect to edge instead of orchestrator
            logger.info("Actor %s connecting to edge %s at %s:%s",
                       self.opt.vehicle_index, connection_info.edge_index,
                       connection_info.edge_ip, connection_info.edge_port)

            self.connected_to_edge = True
            self.edge_ip = connection_info.edge_ip
            self.edge_port = connection_info.edge_port
            self.edge_index = connection_info.edge_index
            self.vehicle_index = connection_info.vehicle_index

            # Create channel to edge
            self.edge_channel = grpc.aio.insecure_channel(
                target=f"{self.edge_ip}:{self.edge_port}",
                options=[
                    ("grpc.lb_policy_name", "pick_first"),
                    ("grpc.enable_retries", 1),
                    ("grpc.keepalive_timeout_ms", 10000),
                    ("grpc.service_config", EcloudClient.retry_opts),],
            )
            self.edge_stub = ecloud_rpc.EcloudStub(self.edge_channel)

            # Register with edge
            ecloud_update = await self.register_with_edge()
        else:
            # No edge, connect directly to orchestrator (existing behavior)
            logger.info("Actor %s connecting directly to orchestrator (no edge)",
                       self.opt.vehicle_index)
            self.connected_to_edge = False
            ecloud_update = await self.send_registration_to_ecloud_server()
            self.vehicle_index = ecloud_update.vehicle_index - ( 2 if self.actor_type == ecloud.ActorType.RSU else 0 )
            logger.info("Registered with orchestrator, vehicle_index=%s", self.vehicle_index)

        assert self.vehicle_index is not None, "vehicle_index not set"

        return ecloud_update

    async def get_connection_info(self) -> ecloud.ActorConnectionInfo:
        """Query orchestrator for connection info (edge vs orchestrator)."""
        request = ecloud.RegistrationInfo()
        request.vehicle_index = self.opt.vehicle_index
        request.actor_type = self.actor_type
        try:
            request.container_name = os.environ["HOSTNAME"]
        except:
            request.container_name = f"actor_{self.opt.vehicle_index}"

        try:
            connection_info = await self.ecloud_server.Client_GetConnectionInfo(request)
            logger.info("Connection info received: has_edge=%s", connection_info.has_edge)
            return connection_info
        except grpc.aio.AioRpcError as e:
            # If RPC not implemented, fall back to orchestrator connection
            logger.warning("Client_GetConnectionInfo not available: %s", e.code())
            return None

    async def register_with_edge(self) -> ecloud.SimulationInfo:
        """Register this actor with its assigned edge."""
        request = ecloud.RegistrationInfo()
        request.vehicle_state = ecloud.VehicleState.REGISTERING
        request.vehicle_index = self.vehicle_index
        request.actor_type = self.actor_type
        try:
            request.container_name = os.environ["HOSTNAME"]
        except:
            request.container_name = f"actor_{self.vehicle_index}"

        request.vehicle_ip = VEHICLE_IP
        request.vehicle_port = self.push_port

        sim_info = await self.edge_stub.Edge_ActorRegister(request)
        logger.info("Registered with edge, vehicle_index=%s", sim_info.vehicle_index)

        return sim_info

    async def close(self):
        if self.edge_channel is not None:
            await self.edge_channel.close()
        if self.channel is not None:
            await self.channel.close()

    #TODO: move to eCloudClient
    def serialize_debug_info(self, vehicle_update, vehicle_manager) -> None:
        planer_debug_helper = vehicle_manager.agent.planning_metrics
        planer_debug_helper_msg = ecloud.PlanerDebugHelper()
        planer_debug_helper.serialize_debug_info(planer_debug_helper_msg)
        vehicle_update.planer_debug_helper.CopyFrom( planer_debug_helper_msg )

        loc_debug_helper = vehicle_manager.localizer.localization_metrics
        loc_debug_helper_msg = ecloud.LocDebugHelper()
        loc_debug_helper.serialize_debug_info(loc_debug_helper_msg)
        vehicle_update.loc_debug_helper.CopyFrom( loc_debug_helper_msg )

        client_debug_helper = vehicle_manager.client_metrics
        #logger.debug(vehicle_manager.client_metrics.perception_time_list)
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
    async def send_carla_data_to_ecav(self,) -> ecloud.SimulationInfo:
        message = {"vehicle_index": self.vehicle_index, "actor_id": self.actor_id, "vid": self.vid}
        logger.info("Vehicle: Sending Carla rpc %s", message)

        update = ecloud.RegistrationInfo()
        update.vehicle_state = ecloud.VehicleState.CARLA_UPDATE
        update.vehicle_index = self.vehicle_index
        update.vid = self.vid
        update.actor_id = self.actor_id
        update.actor_type = self.actor_type

        if self.connected_to_edge:
            # Signal edge we are initialized and ready for ticks.
            # The edge aggregates these across all actors, then notifies the orchestrator,
            # which unblocks run_comms() and allows the tick loop to start.
            assert self.edge_stub is not None, "edge stub not initialized"
            await self.edge_stub.Edge_ActorReady(update)
        else:
            assert self.ecloud_server is not None, "orchestrator stub not initialized"
            await self.ecloud_server.Client_RegisterVehicle(update)

        logger.info("send_carla_data_to_ecav completed")

    #TODO: move to eCloudClient
    async def send_vehicle_update(self, vehicle_update_):
        actor_label = "VEHICLE" if self.actor_type == ecloud.ActorType.VEHICLE else "RSU"
        logger.debug("[DATA_FLOW] tick=%d %s %d outgoing: state=%s features=%db objects=%db carla_id=%d",
                     vehicle_update_.tick_id, actor_label, self.vehicle_index,
                     vehicle_update_.vehicle_state,
                     len(vehicle_update_.pickled_features),
                     len(vehicle_update_.pickled_agent_objects),
                     vehicle_update_.carla_actor_id)

        if self.connected_to_edge:
            # Send to edge, receive fused predictions in response
            assert self.edge_stub is not None, "edge stub not initialized"
            object_buffer = await self.edge_stub.Edge_ActorSendUpdate(vehicle_update_)

            # Process fused predictions if available
            if object_buffer and object_buffer.pickled_edge_predictions:
                preds = pickle.loads(object_buffer.pickled_edge_predictions)
                n_preds = len(preds.get('vehicles', [])) if isinstance(preds, dict) else 0
                logger.debug("[DATA_FLOW] tick=%d %s %d edge_predictions: %d objects",
                             vehicle_update_.tick_id, actor_label, self.vehicle_index, n_preds)
                if self.actor_type == ecloud.ActorType.VEHICLE and self.vehicle_manager:
                    self.vehicle_manager.agent.edge_predictions = preds
            else:
                logger.debug("[DATA_FLOW] tick=%d %s %d edge_predictions: empty",
                             vehicle_update_.tick_id, actor_label, self.vehicle_index)
                if (self.opt.apply_ml and self.connected_to_edge
                        and vehicle_update_.tick_id > 3
                        and self.actor_type == ecloud.ActorType.VEHICLE):
                    logger.warning("[DATA_FLOW] tick=%d %s %d: no edge predictions after warmup",
                                   vehicle_update_.tick_id, actor_label, self.vehicle_index)
            return object_buffer
        else:
            # Send directly to orchestrator (existing behavior)
            assert self.ecloud_server is not None, "stub not initialized"
            empty = await self.ecloud_server.Client_SendUpdate(vehicle_update_)
            logger.debug("send_vehicle_update: send complete")
            return empty

    def arg_parse(self):
        parser = build_arg_parser("eCAV Vehicle Simulation.")
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
            t_tick_start = time.time()
            client_start_timestamp = Timestamp()
            client_start_timestamp.GetCurrentTime()
            # update info runs BEFORE waypoint injection.
            # Wrapped in asyncio.to_thread so the blocking LitServe HTTP call
            # (requests.post inside WorldFusion PM) does not stall the event loop.
            update_info_start_time = time.time()
            await asyncio.to_thread(self.vehicle_manager.update_info)
            t_update_info = time.time()

            control = self.vehicle_manager.run_step()
            t_run_step = time.time()
            if self.actor_type == ecloud.ActorType.VEHICLE:
                self.vehicle_manager.vehicle.apply_control(control)

                update_info_end_time = time.time()
                self.vehicle_manager.client_metrics.update_update_info_time((update_info_end_time-update_info_start_time)*1000)

                if self.is_edge:
                    self.network_emulator.update_waypoints()

            vehicle_update.actor_type = self.actor_type
            vehicle_update.tick_id = self.tick_id
            if self.actor_type == ecloud.ActorType.VEHICLE:
                vehicle_update.carla_actor_id = self.vehicle_manager.vehicle.id
            try:
                if self.actor_type == ecloud.ActorType.VEHICLE:
                    self.vehicle_manager.agent.objects["traffic_lights"] = []
                    vehicle_update.pickled_agent_objects = pickle.dumps(self.vehicle_manager.agent.objects)
                else:
                    self.rsu_manager.objects["traffic_lights"] = []
                    vehicle_update.pickled_agent_objects = pickle.dumps(self.rsu_manager.objects)
            except Exception as e:
                logger.error("Error serializing objects: %s", e)
                objects = (self.vehicle_manager.agent.objects
                           if self.actor_type == ecloud.ActorType.VEHICLE
                           else self.rsu_manager.objects)
                for o in objects:
                    bad = find_unpicklable(o, path=f"preds[{type(o).__name__}]")
                    if bad is not None:
                        logger.error("Unpicklable object: %s", bad)
            t_pickle_objects = time.time()

            # Send intermediate features for WorldFusion/BM2CP
            try:
                if self.actor_type == ecloud.ActorType.VEHICLE:
                    pm = self.vehicle_manager.perception_manager
                else:
                    pm = self.rsu_manager.perception_manager
                if hasattr(pm, 'feature_dict') and pm.feature_dict is not None:
                    import zlib
                    import msgpack
                    import msgpack_numpy as m_np
                    m_np.patch()
                    feat_payload = {k: v.half().cpu().numpy() for k, v in pm.feature_dict.items()}
                    vehicle_update.pickled_features = zlib.compress(
                        msgpack.packb(feat_payload, use_bin_type=True))
            except Exception as e:
                print(f"[FEATURES] Error serializing features: {e}", flush=True)
            t_pickle_features = time.time()

            actor_label = "VEHICLE" if self.actor_type == ecloud.ActorType.VEHICLE else "RSU"
            print(f"[CLIENT {actor_label} TICK {self.tick_id}] "
                  f"update_info={(t_update_info-t_tick_start)*1000:.0f}ms | "
                  f"run_step={(t_run_step-t_update_info)*1000:.0f}ms | "
                  f"pickle_obj={(t_pickle_objects-t_run_step)*1000:.0f}ms | "
                  f"pickle_feat={(t_pickle_features-t_pickle_objects)*1000:.0f}ms | "
                  f"total={(t_pickle_features-t_tick_start)*1000:.0f}ms",
                  flush=True)

            # Send localized position to server
            if self.actor_type == ecloud.ActorType.VEHICLE:
                ego_pos = self.vehicle_manager.localizer.get_ego_pos()
                vel = self.vehicle_manager.vehicle.get_velocity()
            else:
                ego_pos = self.rsu_manager.localizer.get_ego_pos()
                vel = None

            if ego_pos is not None:
                vehicle_update.transform.location.x = ego_pos.location.x
                vehicle_update.transform.location.y = ego_pos.location.y
                vehicle_update.transform.location.z = ego_pos.location.z
                vehicle_update.transform.rotation.yaw = ego_pos.rotation.yaw
                vehicle_update.transform.rotation.pitch = ego_pos.rotation.pitch
                vehicle_update.transform.rotation.roll = ego_pos.rotation.roll

            if vel is not None:
                vehicle_update.velocity.x = vel.x
                vehicle_update.velocity.y = vel.y
                vehicle_update.velocity.z = vel.z

            if self.actor_type == ecloud.ActorType.VEHICLE:
                vehicle_update.vehicle_state = ecloud.VehicleState.TICK_OK if not self.vehicle_manager.is_close_to_scenario_destination() else ecloud.VehicleState.TICK_DONE
            else:
                vehicle_update.vehicle_state = ecloud.VehicleState.TICK_OK

            if (self.actor_type == ecloud.ActorType.VEHICLE
                    and vehicle_update.vehicle_state == ecloud.VehicleState.TICK_DONE):
                metrics = {
                    'planning_metrics': self.vehicle_manager.agent.planning_metrics,
                    'client_metrics': self.vehicle_manager.client_metrics,
                    'vehicles_detected': self.vehicle_manager.vehicles_detected,
                }
                vehicle_update.pickled_actor_metrics = pickle.dumps(metrics)
                n_brake_attrs = len(getattr(self.vehicle_manager.agent.planning_metrics, 'brake_attributions', []))
                n_collisions = len(getattr(self.vehicle_manager.client_metrics, 'collisions_event_list', []))
                logger.info("[DATA_FLOW] tick=%d VEHICLE %d: serialized actor metrics on TICK_DONE "
                            "(brake_attrs=%d, collisions=%d)",
                            self.tick_id, self.vehicle_index, n_brake_attrs, n_collisions)

        # block waiting for a response
        t_pre_send = time.time()
        if not self.reported_done or self.done_behavior == eDoneBehavior.CONTROL:
            if not self.reported_done:
                vehicle_update.tick_id = self.tick_id
                vehicle_update.vehicle_index = self.vehicle_index
                await self.send_vehicle_update( vehicle_update)
                t_post_send = time.time()
                actor_label = "VEHICLE" if self.actor_type == ecloud.ActorType.VEHICLE else "RSU"
                print(f"[CLIENT {actor_label}] send_update={(t_post_send-t_pre_send)*1000:.0f}ms", flush=True)

            if vehicle_update.vehicle_state == ecloud.VehicleState.TICK_DONE or vehicle_update.vehicle_state == ecloud.VehicleState.DEBUG_INFO_UPDATE:
                if vehicle_update.vehicle_state == ecloud.VehicleState.DEBUG_INFO_UPDATE and self.pong.command == ecloud.Command.REQUEST_DEBUG_INFO:
                    logger.info("pushed DEBUG_INFO_UPDATE")

                self.reported_done = True
                logger.info("reported_done")

            assert self.push_q.empty(), logger.exception("push_q had %s in it when it should have been empty", self.push_q.get_nowait())
            t_wait_start = time.time()
            self.pong = await self.push_q.get()
            t_wait_end = time.time()
            self.push_q.task_done()
            assert( self.pong.tick_id != self.tick_id )
            self.tick_id = self.pong.tick_id
            actor_label = "VEHICLE" if self.actor_type == ecloud.ActorType.VEHICLE else "RSU"
            print(f"[CLIENT {actor_label}] wait_for_next_cmd={(t_wait_end-t_wait_start)*1000:.0f}ms", flush=True)

            if self.pong.command == ecloud.Command.PULL_WAYPOINTS_AND_TICK:
                wp_request = ecloud.WaypointRequest()
                wp_request.vehicle_index = self.vehicle_index
                waypoint_proto = await self.ecloud_server.Client_GetWaypoints(wp_request)
                self.network_emulator.enqueue_wp(waypoint_proto)
                self.pong.command = ecloud.Command.TICK

            elif self.pong.command == ecloud.Command.PULL_OBJECTS_AND_TICK:
                if self.connected_to_edge:
                    # Edge mode: predictions already set inline via Edge_ActorSendUpdate response.
                    # Client_GetObjects would hit the C++ orchestrator (which has no fused data)
                    # and overwrite the edge-provided predictions with empty.
                    pass
                else:
                    t_pull_start = time.time()
                    obj_request = ecloud.ObjectRequest()
                    obj_request.vehicle_index = self.vehicle_index
                    object_proto = await self.ecloud_server.Client_GetObjects(obj_request)
                    preds = pickle.loads(object_proto.pickled_edge_predictions) if object_proto.pickled_edge_predictions else None
                    if self.actor_type == ecloud.ActorType.VEHICLE:
                        self.vehicle_manager.agent.edge_predictions = preds
                    t_pull_end = time.time()
                    logger.debug("[CLIENT %s] pull_objects=%.0fms", actor_label, (t_pull_end - t_pull_start) * 1000)
                self.pong.command = ecloud.Command.TICK

            # HANDLE END
            elif self.pong.command == ecloud.Command.END:
                logger.info("END received")

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