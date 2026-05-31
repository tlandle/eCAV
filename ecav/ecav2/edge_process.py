# -*- coding: utf-8 -*-
# Author: Jordan Rapp <jrapp7@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib
"""
Standalone Edge Process for Distributed eCAV Simulation

This process acts as an intermediary between actors (vehicles/RSUs) and the orchestrator.
It receives tick notifications from the orchestrator, coordinates its actors, and reports
tick completion back to the orchestrator.
"""

import argparse
import json
import asyncio
import os
import logging
import time
import sys
import pickle

sys.path.insert(0, '/opt/carla-simulator/PythonAPI/carla')
sys.path.insert(0, os.path.join(os.getcwd(), 'ecav'))
sys.path.insert(0, os.path.join(os.getcwd(), 'scenario_runner'))
sys.path.insert(0, os.getcwd())

import grpc

from ecav.scenario_testing.utils.yaml_utils import load_yaml
from ecav.ecloud_server.ecloud_comms import EcloudClient

import ecloud_pb2 as ecloud
import ecloud_pb2_grpc as ecloud_rpc

logger = logging.getLogger(__name__)

# Load cloud config
cloud_config = load_yaml("cloud_config.yaml")
ECLOUD_IP = cloud_config["ecloud_server_public_ip"]
EDGE_IP = cloud_config.get("edge_server_public_ip", cloud_config["vehicle_client_public_ip"])

if cloud_config["log_level"] == "error":
    logger.setLevel(logging.ERROR)
elif cloud_config["log_level"] == "warning":
    logger.setLevel(logging.WARNING)
elif cloud_config["log_level"] == "info":
    logger.setLevel(logging.INFO)


class _FeatureStub:
    """
    Lightweight duck-type stub for an actor in edge-only distributed mode.

    Satisfies the interface expected by WorldFusionEdge.update_information() and
    run_step() without requiring a CARLA connection or full VehicleManager/RSUManager.
    Populated from IntermediateFeatures fields in an Edge_PerformFusion batch.
    """

    class _PerceptionManager:
        def __init__(self, feature_dict):
            self.feature_dict = feature_dict

    class _Localizer:
        def __init__(self, pose):
            # pose is [x, y, z, roll, yaw, pitch]
            from ecav.ecav_carla import Location as _Loc, Rotation as _Rot, Transform as _Tf
            self._transform = _Tf(
                _Loc(x=float(pose[0]), y=float(pose[1]), z=float(pose[2])),
                _Rot(roll=float(pose[3]), yaw=float(pose[4]), pitch=float(pose[5]))
            )

        def get_ego_pos(self):
            return self._transform

        def localize(self):
            pass  # pose already set from batch; no sensor to re-read

    class _Vehicle:
        def __init__(self, vehicle_id, pose):
            self.id = vehicle_id
            self._pose = pose

        def get_location(self):
            from ecav.ecav_carla import Location as _Loc
            return _Loc(x=float(self._pose[0]), y=float(self._pose[1]), z=float(self._pose[2]))

        def get_transform(self):
            from ecav.ecav_carla import Location as _Loc, Rotation as _Rot, Transform as _Tf
            return _Tf(
                _Loc(x=float(self._pose[0]), y=float(self._pose[1]), z=float(self._pose[2])),
                _Rot(roll=float(self._pose[3]), yaw=float(self._pose[4]), pitch=float(self._pose[5]))
            )

    class _Agent:
        def __init__(self):
            self.edge_predictions = []
            self._anchoring = False

    def __init__(self, agent_id, feature_dict, pose):
        """
        Args:
            agent_id:     Stable integer ID for this actor (used by BSM tracker).
            feature_dict: Dict of tensors from unpack_intermediate_features().
            pose:         6-element list [x, y, z, roll, yaw, pitch].
        """
        self.perception_manager = self._PerceptionManager(feature_dict)
        self.localizer = self._Localizer(pose)
        self.vehicle = self._Vehicle(agent_id, pose)
        self.agent = self._Agent()


class EdgeActorInfo:
    """Information about a registered actor."""
    def __init__(self, vehicle_index, actor_id, vid, actor_type, push_port=None):
        self.vehicle_index = vehicle_index
        self.actor_id = actor_id
        self.vid = vid
        self.actor_type = actor_type
        self.push_port = push_port
        self.push_channel = None
        self.push_stub = None
        self.last_update = None
        self.reported_tick = False
        self.is_done = False
        self.manager = None


class EdgeServer(ecloud_rpc.EcloudServicer):
    """
    gRPC Server that accepts connections from actors and the orchestrator.

    Handles:
    - PushTick/Edge_PushTick from orchestrator
    - Edge_ActorRegister from actors
    - Edge_ActorSendUpdate from actors
    """

    def __init__(self, edge_process):
        self.edge_process = edge_process
        logger.info("EdgeServer initialized")

    def PushTick(self, request: ecloud.Tick, context) -> ecloud.Empty:
        """Receive tick notification from orchestrator (backwards compat)."""
        return self._handle_tick(request.tick_id, request.command)

    def Edge_PushTick(self, request: ecloud.EdgeTick, context) -> ecloud.Empty:
        """Receive tick notification from orchestrator."""
        return self._handle_tick(request.tick_id, request.command)

    def _handle_tick(self, tick_id, command):
        """Common tick handling logic."""
        logger.info("Edge received tick %s command %s", tick_id, command)

        # Reset actor completion tracking
        for actor in self.edge_process.actors.values():
            actor.reported_tick = False

        self.edge_process.current_tick_id = tick_id
        self.edge_process.current_command = command

        # Signal the main loop that a new tick arrived
        if not self.edge_process.tick_queue.full():
            self.edge_process.tick_queue.put_nowait((tick_id, command))

        return ecloud.Empty()

    async def Edge_ActorRegister(self, request: ecloud.RegistrationInfo, context) -> ecloud.SimulationInfo:
        """Actor registers with this edge."""
        await self.edge_process.scenario_ready.wait()
        logger.info("Actor registration: vehicle_index=%s, container=%s, push_port=%s",
                   request.vehicle_index, request.container_name, request.vehicle_port)

        # Assign vehicle index (use the one from request or assign sequentially)
        vehicle_index = request.vehicle_index
        actor_type = request.actor_type
        if vehicle_index < 0:
            vehicle_index = len(self.edge_process.actors)

        # Store actor info
        actor_info = EdgeActorInfo(
            vehicle_index=vehicle_index,
            actor_id=request.actor_id,
            vid=request.vid,
            actor_type=actor_type,
            push_port=request.vehicle_port
        )

        # Create push client for this actor (to push ticks to it)
        if request.vehicle_port > 0:
            push_target = f"localhost:{request.vehicle_port}"
            actor_info.push_channel = grpc.aio.insecure_channel(
                target=push_target,
                options=[
                    ("grpc.lb_policy_name", "pick_first"),
                    ("grpc.enable_retries", 1),
                    ("grpc.keepalive_timeout_ms", 10000),
                ],
            )
            actor_info.push_stub = ecloud_rpc.EcloudStub(actor_info.push_channel)
            logger.info("Created push client for %s %s at %s", "vehicle" if actor_type == ecloud.ActorType.VEHICLE else "rsu", vehicle_index, push_target)

        self.edge_process.actors[f"{actor_type}_{vehicle_index}"] = actor_info

        # Return simulation info
        reply = ecloud.SimulationInfo()
        reply.vehicle_index = vehicle_index
        reply.test_scenario = self.edge_process.scenario_yaml_str
        reply.application = self.edge_process.application
        reply.version = self.edge_process.version
        reply.is_edge = True
        reply.carla_ip = self.edge_process.carla_ip

        logger.info("Registered actor %s (total: %d/%d)",
                   vehicle_index, len(self.edge_process.actors),
                   self.edge_process.expected_num_actors)

        return reply

    async def Edge_ActorReady(self, request: ecloud.RegistrationInfo, context) -> ecloud.Empty:
        """Actor signals it has completed initialization and is ready for ticks."""
        vehicle_index = request.vehicle_index
        actor_type = request.actor_type
        key = f"{actor_type}_{vehicle_index}"

        if key in self.edge_process.actors:
            self.edge_process.actors[key].actor_id = request.actor_id
            self.edge_process.actors[key].vid = request.vid

        self.edge_process.num_actors_ready += 1
        logger.info("Actor %s ready (%d/%d)", vehicle_index,
                    self.edge_process.num_actors_ready,
                    self.edge_process.expected_num_actors)

        if self.edge_process.num_actors_ready == self.edge_process.expected_num_actors:
            await self.edge_process.notify_actors_ready()

        return ecloud.Empty()

    def Edge_PerformFusion(self, request: ecloud.IntermediateFeaturesBatch, context) -> ecloud.FusionResult:
        """
        Edge-only distributed mode: ecav.py sends a batch of pre-extracted intermediate
        features for all actors and receives fused predictions.

        Idempotency: if tick_id < expected_tick_id the cached result is returned.
        If tick_id > expected_tick_id the tick is out of order and we return empty.
        """
        import zlib
        import msgpack
        import msgpack_numpy as m_np
        import torch
        m_np.patch()

        ep = self.edge_process
        tick_id = request.tick_id

        # Idempotency gate
        if tick_id < ep.expected_tick_id:
            logger.info("Edge_PerformFusion: duplicate tick_id=%d (expected %d) — returning cached result",
                        tick_id, ep.expected_tick_id)
            return ep._last_fusion_result if ep._last_fusion_result is not None else ecloud.FusionResult(tick_id=tick_id)

        if tick_id != ep.expected_tick_id:
            logger.error("Edge_PerformFusion: out-of-order tick_id=%d (expected %d) — returning empty",
                         tick_id, ep.expected_tick_id)
            return ecloud.FusionResult(tick_id=tick_id)

        if ep.edge_manager is None:
            logger.warning("Edge_PerformFusion: edge_manager not ready at tick %d", tick_id)
            return ecloud.FusionResult(tick_id=tick_id)

        # Unpack features from batch.
        # WorldFusion requires RSU as agent 0; the batch must arrive RSU-first.
        rsu_stubs = []
        vehicle_stubs = []

        for feat in request.features:
            pose = list(feat.pose.pose) if feat.pose.pose else [0.0] * 6

            if feat.spatial_features.data:
                compressed = feat.spatial_features.data
                raw = zlib.decompress(compressed)
                feat_np = msgpack.unpackb(raw, raw=False)
                feature_dict = {k: torch.from_numpy(v) for k, v in feat_np.items()}
            else:
                feature_dict = None

            stub = _FeatureStub(agent_id=feat.agent_id, feature_dict=feature_dict, pose=pose)

            if feat.agent_type == ecloud.ActorType.RSU:
                rsu_stubs.append(stub)
            else:
                vehicle_stubs.append(stub)

        # Inject stubs: RSUs first, then vehicles (WorldFusion agent-0 = RSU invariant)
        ep.edge_manager.rsu_manager_list = rsu_stubs
        ep.edge_manager.vehicle_manager_list = vehicle_stubs

        # Run the standard fusion step
        serialized_preds = ep.edge_manager.run_step(tick_id)

        # Pack predictions into FusionResult.
        # buf.vehicle_id is the index within vehicle_stubs (same ordering as the batch).
        all_preds = {}
        if serialized_preds is not None:
            for buf in serialized_preds.all_object_buffers:
                if buf.pickled_edge_predictions:
                    all_preds[buf.vehicle_id] = buf.pickled_edge_predictions

        result = ecloud.FusionResult(tick_id=tick_id)
        if all_preds:
            result.pickled_predictions = pickle.dumps(all_preds)

        ep._last_fusion_result = result
        ep.expected_tick_id += 1

        logger.info("Edge_PerformFusion: tick=%d done, %d vehicle predictions packed",
                    tick_id, len(all_preds))
        return result

    def Edge_EndScenario(self, request: ecloud.Empty, context) -> ecloud.EdgeEvaluationResult:
        """
        Edge-only distributed mode: ecav.py signals end of scenario.
        Finalizes the edge profiler and returns metrics.
        """
        ep = self.edge_process
        result = ecloud.EdgeEvaluationResult(edge_index=ep.edge_index)

        if ep.edge_manager is None:
            logger.warning("Edge_EndScenario called but edge_manager is None")
            return result

        try:
            _, _, metrics = ep.edge_manager.evaluate()
            result.pickled_edge_profiler = pickle.dumps(metrics)
            logger.info("Edge_EndScenario: profiler serialized (%d keys)", len(metrics))
        except Exception as exc:
            logger.warning("Edge_EndScenario: evaluate() failed: %s", exc)

        return result

    async def Edge_ActorSendUpdate(self, request: ecloud.VehicleUpdate, context) -> ecloud.ObjectBuffer:
        """Actor sends update to edge, receives previous tick's fused data."""
        vehicle_index = request.vehicle_index
        actor_type = request.actor_type
        logger.debug("Actor %s sent update for tick %s", vehicle_index, request.tick_id)

        # Store the update
        key = f"{actor_type}_{vehicle_index}"
        if key in self.edge_process.actors:
            actor_info = self.edge_process.actors[key]
            actor_info.last_update = request
            actor_info.reported_tick = True
            if request.vehicle_state == ecloud.VehicleState.TICK_DONE:
                actor_info.is_done = True
                logger.info("Actor %s reported TICK_DONE — excluding from future tick barriers", vehicle_index)
                if (request.actor_type == ecloud.ActorType.VEHICLE
                        and request.pickled_actor_metrics
                        and actor_info.manager is not None):
                    try:
                        m = pickle.loads(request.pickled_actor_metrics)
                        actor_info.manager.agent.planning_metrics = m['planning_metrics']
                        actor_info.manager.client_metrics = m['client_metrics']
                        actor_info.manager.vehicles_detected = m['vehicles_detected']
                        logger.info("[DATA_FLOW] Edge stored actor metrics for vehicle %s from TICK_DONE", vehicle_index)
                    except Exception as e:
                        logger.warning("Failed to unpack actor metrics for vehicle %s: %s", vehicle_index, e)
        else:
            logger.warning("Received update from unknown actor %s", vehicle_index)

        # Signal that an actor completed
        if not self.edge_process.actor_complete_queue.full():
            self.edge_process.actor_complete_queue.put_nowait(vehicle_index)

        # Return previous tick's fused predictions (if any)
        reply = ecloud.ObjectBuffer()
        reply.vehicle_id = vehicle_index

        actor_key = f"{actor_type}_{vehicle_index}"
        if actor_key in self.edge_process.fused_predictions:
            reply.pickled_edge_predictions = self.edge_process.fused_predictions[actor_key]

        return reply


class EdgeProcess:
    """
    Main edge process that coordinates actors and communicates with orchestrator.
    """

    def __init__(self):
        self.opt = self.arg_parse()

        # Edge identity
        self.edge_index = self.opt.edge_index
        self.edge_port = self.opt.edge_port

        # Scenario info (received from orchestrator on registration)
        self.scenario_yaml_str = ""
        self.application = ""
        self.version = ""
        self.carla_ip = ""
        self.scenario_ready = asyncio.Event()  # set once register_with_orchestrator() completes

        # Actor tracking
        self.actors = {}  # vehicle_index -> EdgeActorInfo
        self.expected_num_actors = 0
        self.num_actors_ready = 0
        self.fused_predictions = {}  # vehicle_index -> pickled predictions

        # Tick tracking
        self.current_tick_id = 0
        self.current_command = ecloud.Command.TICK
        self.tick_queue = asyncio.Queue(maxsize=1)
        self.actor_complete_queue = asyncio.Queue()

        # Edge manager (instantiated in setup_edge_manager after all actors ready)
        self.edge_manager = None
        self._edge_manager_cls = None
        self._edge_manager_cfg = None
        self._edge_manager_args = {}
        self._cav_world = None
        self._init_task = None          # asyncio.Task for _init_edge_manager_model (phase A)
        self._vm_list_to_actor_key = []  # position in vm_list -> actor key

        # Edge-only distributed mode: tick-ID tracking for idempotent fusion RPCs
        self.expected_tick_id = 0        # next tick_id we expect from Edge_PerformFusion
        self._last_fusion_result = None  # cached FusionResult for retry detection

        # gRPC connections
        self.orchestrator_channel = None
        self.orchestrator_stub = None
        self.server = None

        if self.opt.verbose:
            logger.setLevel(logging.DEBUG)
        elif self.opt.quiet:
            logger.setLevel(logging.WARNING)

        logging.basicConfig(level=logging.INFO)

    def arg_parse(self):
        parser = argparse.ArgumentParser(description="eCAV Edge Process")
        parser.add_argument('-e', "--edge_index", type=int, default=0,
                           help='Edge index (unique identifier; default 0)')
        parser.add_argument('-P', "--edge_port", type=int, default=50054,
                           help='Port for this edge to listen on')
        parser.add_argument('-o', "--orchestrator_port", type=int, default=50051,
                           help='Orchestrator port')
        parser.add_argument('-O', "--orchestrator_ip", type=str, default=ECLOUD_IP,
                           help='Orchestrator IP address')
        parser.add_argument("--standalone", action="store_true",
                           help="Edge-only distributed mode: skip orchestrator registration, "
                                "load config from --config, accept Edge_PerformFusion calls directly")
        parser.add_argument("--config", type=str, default=None,
                           help="Path to scenario YAML (required in --standalone mode)")
        parser.add_argument("--verbose", action="store_true",
                           help="Enable debug logging")
        parser.add_argument('-q', "--quiet", action="store_true",
                           help="Reduce logging")
        return parser.parse_args()

    async def register_with_orchestrator(self):
        """Register this edge with the orchestrator and receive scenario config."""
        logger.info("Registering edge %s with orchestrator at %s:%s",
                   self.edge_index, self.opt.orchestrator_ip, self.opt.orchestrator_port)

        # Create channel to orchestrator
        self.orchestrator_channel = grpc.aio.insecure_channel(
            target=f"{self.opt.orchestrator_ip}:{self.opt.orchestrator_port}",
            options=[
                ("grpc.lb_policy_name", "pick_first"),
                ("grpc.enable_retries", 1),
                ("grpc.keepalive_timeout_ms", 10000),
                ("grpc.service_config", EcloudClient.retry_opts),
            ],
        )
        self.orchestrator_stub = ecloud_rpc.EcloudStub(self.orchestrator_channel)

        # Build registration request
        request = ecloud.EdgeRegistrationInfo()
        request.edge_index = self.edge_index
        request.edge_ip = EDGE_IP
        request.edge_port = self.edge_port
        request.num_vehicles = 0  # Will be filled in by orchestrator
        request.num_rsus = 0
        try:
            request.container_name = os.environ["HOSTNAME"]
        except:
            request.container_name = f"edge_{self.edge_index}"

        # Register and receive config — retry until the orchestrator is reachable.
        # In edge-only mode the edge container may start before ecav.py's registration
        # server is up; in fully-distributed mode the C++ server may still be initializing.
        retry_timeout_s = 120.0
        retry_interval_s = 2.0
        deadline = time.monotonic() + retry_timeout_s
        config = None
        while True:
            try:
                config = await self.orchestrator_stub.Edge_Register(request)
                break
            except grpc.aio.AioRpcError as exc:
                if exc.code() != grpc.StatusCode.UNAVAILABLE:
                    raise
                if time.monotonic() >= deadline:
                    raise RuntimeError(
                        f"Orchestrator at {self.opt.orchestrator_ip}:{self.opt.orchestrator_port} "
                        f"not reachable after {retry_timeout_s:.0f}s"
                    ) from exc
                logger.debug("Orchestrator not yet reachable — retrying in %.0fs", retry_interval_s)
                await asyncio.sleep(retry_interval_s)

        self.scenario_yaml_str = config.edge_config_yaml
        self.application = config.application
        self.version = config.version
        self.carla_ip = config.carla_ip
        self.expected_num_actors = config.num_vehicles + config.num_rsus
        # In edge-only mode the server assigns the index; overwrite CLI default.
        self.edge_index = config.edge_index
        self.scenario_ready.set()

        logger.info("Edge %s registered successfully. Expected actors: %d vehicles, %d RSUs",
                   self.edge_index, config.num_vehicles, config.num_rsus)
        logger.info("Assigned vehicle indices: %s", list(config.vehicle_indices))
        logger.info("Assigned RSU indices: %s", list(config.rsu_indices))

        # Phase A: resolve edge manager class while actors are registering
        self._init_task = asyncio.create_task(self._init_edge_manager_model())

        return config

    async def run_server(self):
        """Start the gRPC server for actors to connect to."""
        logger.info("Starting edge server on port %s", self.edge_port)

        self.server = grpc.aio.server()
        ecloud_rpc.add_EcloudServicer_to_server(EdgeServer(self), self.server)

        listen_addr = f"0.0.0.0:{self.edge_port}"
        self.server.add_insecure_port(listen_addr)
        await self.server.start()

        logger.info("Edge server started on %s", listen_addr)
        await self.server.wait_for_termination()

    async def wait_for_actors(self):
        """Wait for all expected actors to register."""
        logger.info("Waiting for %d actors to register...", self.expected_num_actors)

        while len(self.actors) < self.expected_num_actors:
            await asyncio.sleep(0.1)

        logger.info("All %d actors registered", self.expected_num_actors)

    async def _init_edge_manager_model(self):
        """Phase A: resolve edge manager class and store config. Runs while actors register."""
        try:
            from ecav.core.common.cav_world import CavWorld
            from ecav.core.application.edge.edge_manager import get_edge_class

            scenario = json.loads(self.scenario_yaml_str)
            edge_cfg = scenario['scenario']['edge_list'][self.edge_index]
            world_dt = scenario.get('world', {}).get('fixed_delta_seconds', 0.05)
            edge_dt = scenario.get('edge_base', {}).get('edge_dt', 0.2)

            self._edge_manager_cls = get_edge_class(edge_cfg.get('manager_type', 'late_fusion'))
            self._edge_manager_cfg = edge_cfg
            self._edge_manager_args = dict(world_dt=world_dt, edge_dt=edge_dt, run_distributed=True)
            logger.info("Edge manager class resolved: %s", self._edge_manager_cls.__name__)
            logger.info("Phase A: creating CavWorld (apply_ml=False — edge loads WF model directly)...")
            self._cav_world = CavWorld(apply_ml=False, config={'distributed': True})
            logger.info("Phase A: CavWorld created successfully")
        except Exception as exc:
            logger.error("Phase A (_init_edge_manager_model) failed: %s: %s", type(exc).__name__, exc, exc_info=True)
            raise

    async def setup_edge_manager(self):
        """Phase B: connect to CARLA, instantiate proxy managers, start edge manager."""
        if self._init_task is not None:
            if not self._init_task.done():
                logger.info("Waiting for edge manager model init (phase A) to complete...")
            await self._init_task  # always await — re-raises exception if phase A failed

        import carla as _carla
        from ecav.core.common.vehicle_manager import VehicleManager
        from ecav.core.common.rsu_manager import RSUManager

        scenario = json.loads(self.scenario_yaml_str)
        client = _carla.Client(self.carla_ip, 2000)
        client.set_timeout(10.0)
        world = client.get_world()
        carla_map = world.get_map()
        edge_cfg = self._edge_manager_cfg
        current_time = scenario.get('current_time', '')

        self.edge_manager = self._edge_manager_cls(
            world, edge_cfg, self._cav_world,
            carla_client=client,
            **self._edge_manager_args
        )

        # RSUs first — WorldFusion requires agent 0 to be the RSU
        rsu_cfgs = edge_cfg.get('rsus', [])
        rsu_keys = sorted(
            (k for k, a in self.actors.items() if a.actor_type == ecloud.ActorType.RSU),
            key=lambda k: self.actors[k].vehicle_index
        )
        for idx, key in enumerate(rsu_keys):
            info = self.actors[key]
            rsu_manager = RSUManager(
                world, rsu_cfgs[idx], carla_map, self._cav_world,
                current_time, is_proxy=True, run_distributed=True
            )
            if info.actor_id >= 0:
                rsu_manager.rsu = world.get_actor(info.actor_id)
            info.manager = rsu_manager
            self.edge_manager.add_rsu(rsu_manager)

        vehicle_keys = sorted(
            (k for k, a in self.actors.items() if a.actor_type == ecloud.ActorType.VEHICLE),
            key=lambda k: self.actors[k].vehicle_index
        )
        for idx, key in enumerate(vehicle_keys):
            info = self.actors[key]
            carla_actor = world.get_actor(info.actor_id)
            vm = VehicleManager(
                vehicle=carla_actor,
                vehicle_index=info.vehicle_index,
                config_yaml=scenario,
                application=self.application,
                carla_world=world,
                carla_map=carla_map,
                cav_world=self._cav_world,
                current_time=current_time,
                is_proxy=True,
                is_edge=True,
                run_distributed=True,
                perception_active=False,
            )
            info.manager = vm
            self.edge_manager.add_member(vm)
            self._vm_list_to_actor_key.append(key)

        self.edge_manager.start_edge()
        logger.info("Edge manager ready: %s, %d vehicles, %d RSUs",
                    type(self.edge_manager).__name__,
                    len(self.edge_manager.vehicle_manager_list),
                    len(self.edge_manager.rsu_manager_list))

    async def notify_actors_ready(self):
        """Notify orchestrator that all actors have completed initialization."""
        await self.setup_edge_manager()
        request = ecloud.EdgeReadyNotification()
        request.edge_index = self.edge_index
        request.num_actors = self.num_actors_ready
        for info in self.actors.values():
            ai = ecloud.ActorCarlaInfo()
            ai.vehicle_index = info.vehicle_index
            ai.actor_type = info.actor_type
            ai.carla_actor_id = info.actor_id
            request.actor_infos.append(ai)
        await self.orchestrator_stub.Edge_ActorsReady(request)
        logger.info("Edge %s: notified orchestrator — %d actors ready, %d carla IDs sent",
                    self.edge_index, self.num_actors_ready, len(request.actor_infos))

    async def report_tick_complete(self, tick_id, newly_done=None):
        """Report to orchestrator that this edge completed processing for the tick."""
        logger.info("Reporting tick %s complete to orchestrator", tick_id)

        request = ecloud.EdgeTickComplete()
        request.edge_index = self.edge_index
        request.tick_id = tick_id
        request.num_actors_processed = len(self.actors)

        for actor in (newly_done or []):
            if actor.actor_type == ecloud.ActorType.VEHICLE and actor.last_update is not None:
                request.vehicle_updates.append(actor.last_update)
                logger.info("Forwarding TICK_DONE for vehicle %s to orchestrator", actor.vehicle_index)

        await self.orchestrator_stub.Edge_TickComplete(request)

    async def push_tick_to_actors(self, tick_id, command):
        """Push tick notification to all active (non-done) registered actors."""
        active = {k: a for k, a in self.actors.items() if not a.is_done}
        logger.info("Pushing tick %s to %d/%d actors", tick_id, len(active), len(self.actors))

        tick = ecloud.Tick()
        tick.tick_id = tick_id
        tick.command = command

        push_tasks = []
        for vehicle_index, actor in active.items():
            if actor.push_stub is not None:
                push_tasks.append(self._push_tick_to_actor(actor, tick, vehicle_index))

        if push_tasks:
            results = await asyncio.gather(*push_tasks, return_exceptions=True)
            for i, result in enumerate(results):
                if isinstance(result, Exception):
                    logger.warning("Failed to push tick to actor: %s", result)

    async def _push_tick_to_actor(self, actor, tick, vehicle_index):
        """Push tick to a single actor."""
        try:
            await actor.push_stub.PushTick(tick)
            logger.debug("Pushed tick %s to actor %s", tick.tick_id, vehicle_index)
        except Exception as e:
            logger.warning("Failed to push tick to actor %s: %s", vehicle_index, e)
            raise

    async def process_tick(self, tick_id, command):
        """Process a single tick: push to actors, wait for responses, fuse data, report complete."""
        logger.info("Processing tick %s (command: %s)", tick_id, command)

        if command == ecloud.Command.END:
            logger.info("END command received")
            await self.push_tick_to_actors(tick_id, command)
            await self._send_evaluation_data()
            return False  # Signal to stop

        # Snapshot done set before pushing so newly_done diff is clean
        prior_done = {key for key, a in self.actors.items() if a.is_done}

        # Push tick to all active (non-done) actors
        await self.push_tick_to_actors(tick_id, command)

        # Wait for all active (non-done) actors to report
        expected = sum(1 for a in self.actors.values() if not a.is_done)
        actors_reported = 0
        while actors_reported < expected:
            try:
                vehicle_index = await asyncio.wait_for(
                    self.actor_complete_queue.get(),
                    timeout=30.0
                )
                actors_reported += 1
                logger.debug("Actor %s completed (%d/%d)",
                           vehicle_index, actors_reported, expected)
            except asyncio.TimeoutError:
                logger.warning("Timeout waiting for actors. Reported: %d/%d",
                             actors_reported, expected)
                break

        # Determine which actors became done this tick
        newly_done = [a for key, a in self.actors.items()
                      if a.is_done and key not in prior_done]

        self.run_edge_step(tick_id)

        # Report completion to orchestrator, forwarding any newly-done vehicle state
        await self.report_tick_complete(tick_id, newly_done)

        return True  # Continue processing

    FUSION_WARMUP_TICKS = 3

    def run_edge_step(self, tick_id: int):
        """Push actor data into proxy managers then run the edge manager step."""
        if self.edge_manager is None:
            logger.warning("run_edge_step called before edge_manager ready")
            return

        import zlib
        import msgpack
        import msgpack_numpy as m_np
        import torch
        m_np.patch()

        actors_with_features = 0
        actors_with_objects = 0
        total_actors = sum(1 for info in self.actors.values() if not info.is_done)

        for key, info in self.actors.items():
            if info.last_update is None or info.manager is None:
                continue
            upd = info.last_update
            mgr = info.manager

            # Refresh localizer so get_ego_pos() returns current CARLA position
            mgr.localizer.localize()

            has_features = bool(upd.pickled_features)
            has_objects = bool(upd.pickled_agent_objects)
            logger.debug("[DATA_FLOW] tick=%d actor=%s features=%s(%db) objects=%s(%db)",
                         tick_id, key,
                         "yes" if has_features else "no", len(upd.pickled_features),
                         "yes" if has_objects else "no", len(upd.pickled_agent_objects))

            # Intermediate features (WorldFusion)
            if has_features:
                actors_with_features += 1
                try:
                    feat_np = msgpack.unpackb(
                        zlib.decompress(upd.pickled_features), raw=False)
                    mgr.perception_manager.feature_dict = {
                        k: torch.from_numpy(v) for k, v in feat_np.items()
                    }
                except Exception as e:
                    logger.warning("Feature unpack failed for %s: %s", key, e)

            # Detection objects (late fusion)
            if has_objects:
                actors_with_objects += 1
                try:
                    objects = pickle.loads(upd.pickled_agent_objects)
                    if info.actor_type == ecloud.ActorType.RSU:
                        mgr.objects = objects
                    else:
                        mgr.agent.objects = objects
                except Exception as e:
                    logger.warning("Objects unpack failed for %s: %s", key, e)

        try:
            serialized_preds = self.edge_manager.run_step(tick_id)
            n_preds = 0
            vehicles_updated = 0
            if serialized_preds is not None:
                for obj_buf in serialized_preds.all_object_buffers:
                    vm_list_idx = obj_buf.vehicle_id
                    if vm_list_idx < len(self._vm_list_to_actor_key):
                        actor_key = self._vm_list_to_actor_key[vm_list_idx]
                        self.fused_predictions[actor_key] = obj_buf.pickled_edge_predictions
                        if obj_buf.pickled_edge_predictions:
                            _p = pickle.loads(obj_buf.pickled_edge_predictions)
                            n_preds += len(_p) if isinstance(_p, list) else len(_p.get('vehicles', []))
                        vehicles_updated += 1

            logger.info("[DATA_FLOW] tick=%d features=%d/%d objects=%d/%d predictions=%d vehicles_updated=%d",
                        tick_id, actors_with_features, total_actors,
                        actors_with_objects, total_actors, n_preds, vehicles_updated)

            if tick_id > self.FUSION_WARMUP_TICKS and not self.fused_predictions:
                logger.warning("[DATA_FLOW] tick=%d: edge_manager produced no fused predictions after warmup", tick_id)
            if actors_with_features == 0 and actors_with_objects == 0 and total_actors > 0 and tick_id > self.FUSION_WARMUP_TICKS:
                logger.warning("[DATA_FLOW] tick=%d: zero actors sent any data (total_actors=%d)", tick_id, total_actors)

            if self.opt.verbose and tick_id > self.FUSION_WARMUP_TICKS:
                assert actors_with_features > 0 or actors_with_objects > 0, \
                    f"[DATA_FLOW] tick={tick_id}: no actor data received after warmup (verbose assertion)"

        except Exception as e:
            logger.exception("edge_manager.run_step failed at tick %s: %s", tick_id, e)

    async def _setup_edge_manager_standalone(self):
        """
        Initialize edge manager without CARLA (edge-only distributed mode).

        Phase A (_init_edge_manager_model) must have completed before this is called.
        vehicle_manager_list and rsu_manager_list start empty; stubs are injected
        per-tick by the Edge_PerformFusion handler.
        """
        if self._init_task is not None:
            await self._init_task  # re-raises if phase A failed

        self.edge_manager = self._edge_manager_cls(
            world=None,
            cfg=self._edge_manager_cfg,
            cav_world=self._cav_world,
            carla_client=None,
            **self._edge_manager_args
        )
        self.edge_manager.start_edge()
        logger.info("Edge manager ready (standalone, no CARLA): %s",
                    type(self.edge_manager).__name__)

    async def _run_standalone(self):
        """
        Edge-only distributed mode run loop.

        Skips orchestrator registration. Loads scenario YAML from --config,
        initializes the edge manager without CARLA, then sits in the gRPC server
        loop. The tick loop is driven entirely by Edge_PerformFusion calls from
        ecav.py; there is no tick queue here.
        """
        if not self.opt.config:
            raise ValueError("--standalone requires --config <scenario_yaml>")

        logger.info("Edge %s starting in standalone mode (port %s)", self.edge_index, self.edge_port)

        # Load scenario YAML; store as JSON string so _init_edge_manager_model can parse it
        import json
        scenario = load_yaml(self.opt.config)
        edge_list = scenario.get('scenario', {}).get('edge_list', [])
        if self.edge_index >= len(edge_list):
            raise ValueError(
                f"--edge_index {self.edge_index} out of range "
                f"(scenario has {len(edge_list)} edge(s))")
        self.scenario_yaml_str = json.dumps(scenario)

        # Phase A: resolve edge manager class and create CavWorld (no CARLA needed)
        self._init_task = asyncio.create_task(self._init_edge_manager_model())

        # Phase B: instantiate edge manager (no CARLA)
        await self._setup_edge_manager_standalone()

        # Signal scenario ready so Edge_ActorRegister (if it ever fires) won't block
        self.scenario_ready.set()

        logger.info("Edge %s standalone ready — waiting for Edge_PerformFusion calls", self.edge_index)

        # gRPC server runs until process is killed or Edge_EndScenario is received
        await self.run_server()

    async def run(self):
        """Main entry point for the edge process."""
        if self.opt.standalone:
            try:
                await self._run_standalone()
            except Exception as e:
                logger.exception("Edge standalone error: %s", e)
                raise
            finally:
                await self.cleanup()
            return

        logger.info("Starting edge process %s", self.edge_index)

        try:
            # Start our server first so orchestrator can push to us
            self.push_server_task = asyncio.create_task(self.run_server())

            # Register with orchestrator (C++ ecloud_server in fully-distributed mode;
            # ecav.py EdgeRegistrationServer in edge-only distributed mode).
            await self.register_with_orchestrator()

            # Edge-only distributed mode: ecav.py sends carla_ip="" to signal no CARLA.
            # Skip actor wait and tick queue; initialize without CARLA and serve fusion RPCs.
            if not self.carla_ip:
                logger.info("Edge %s: carla_ip empty — edge-only distributed mode (no CARLA)", self.edge_index)
                await self._setup_edge_manager_standalone()
                self.scenario_ready.set()
                logger.info("Edge %s edge-only ready — serving Edge_PerformFusion calls", self.edge_index)
                await self.push_server_task   # blocks until server terminates (Edge_EndScenario or kill)
                return

            # Wait for actors to connect
            await self.wait_for_actors()

            logger.info("Edge %s ready for simulation", self.edge_index)

            # Main tick loop
            running = True
            while running:
                # Wait for tick from orchestrator
                tick_id, command = await self.tick_queue.get()
                self.tick_queue.task_done()

                # Process the tick
                running = await self.process_tick(tick_id, command)

            logger.info("Edge %s shutting down", self.edge_index)

        except Exception as e:
            logger.exception("Edge process error: %s", e)
            raise
        finally:
            await self.cleanup()

    async def _send_evaluation_data(self):
        """Serialize edge profiler and push to orchestrator at END time (while C++ server is still alive)."""
        if self.edge_manager is None or self.orchestrator_stub is None:
            return
        try:
            _, _, metrics = self.edge_manager.evaluate()
            request = ecloud.EdgeEvaluationResult()
            request.edge_index = self.edge_index
            request.pickled_edge_profiler = pickle.dumps(metrics)
            await self.orchestrator_stub.Edge_SendEvaluationData(request)
            logger.info("[DATA_FLOW] Sent edge profiler metrics to orchestrator: %d keys", len(metrics))
        except Exception as exc:  # pylint: disable=broad-except
            logger.warning("Failed to send edge evaluation data: %s", exc)

    async def cleanup(self):
        """Clean up resources."""
        logger.info("Cleaning up edge %s", self.edge_index)

        if self.server:
            await self.server.stop(grace=5)

        if self.orchestrator_channel:
            await self.orchestrator_channel.close()


async def main():
    edge = EdgeProcess()
    await edge.run()


if __name__ == "__main__":
    asyncio.run(main())
