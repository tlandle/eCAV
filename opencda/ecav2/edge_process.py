# -*- coding: utf-8 -*-
"""
Standalone Edge Process for Distributed eCAV Simulation

This process acts as an intermediary between actors (vehicles/RSUs) and the orchestrator.
It receives tick notifications from the orchestrator, coordinates its actors, and reports
tick completion back to the orchestrator.

Authors: Jordan Rapp <jrapp7@gatech.edu>
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
sys.path.insert(0, os.path.join(os.getcwd(), 'opencda'))
sys.path.insert(0, os.path.join(os.getcwd(), 'scenario_runner'))
sys.path.insert(0, os.getcwd())

import coloredlogs
import grpc

from opencda.scenario_testing.utils.yaml_utils import load_yaml
from opencda.ecloud_server.ecloud_comms import EcloudClient

import ecloud_pb2 as ecloud
import ecloud_pb2_grpc as ecloud_rpc

logger = logging.getLogger(__name__)
coloredlogs.install(level='DEBUG', logger=logger)
logger.setLevel(logging.DEBUG)

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

    async def Edge_ActorSendUpdate(self, request: ecloud.VehicleUpdate, context) -> ecloud.ObjectBuffer:
        """Actor sends update to edge, receives previous tick's fused data."""
        vehicle_index = request.vehicle_index
        actor_type = request.actor_type
        logger.debug("Actor %s sent update for tick %s", vehicle_index, request.tick_id)

        # Store the update
        if f"{actor_type}_{vehicle_index}" in self.edge_process.actors:
            self.edge_process.actors[f"{actor_type}_{vehicle_index}"].last_update = request
            self.edge_process.actors[f"{actor_type}_{vehicle_index}"].reported_tick = True
        else:
            logger.warning("Received update from unknown actor %s", vehicle_index)

        # Signal that an actor completed
        if not self.edge_process.actor_complete_queue.full():
            self.edge_process.actor_complete_queue.put_nowait(vehicle_index)

        # Return previous tick's fused predictions (if any)
        reply = ecloud.ObjectBuffer()
        reply.vehicle_id = vehicle_index

        if vehicle_index in self.edge_process.fused_predictions:
            reply.pickled_edge_predictions = self.edge_process.fused_predictions[vehicle_index]

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

        # Actor tracking
        self.actors = {}  # vehicle_index -> EdgeActorInfo
        self.expected_num_actors = 0
        self.fused_predictions = {}  # vehicle_index -> pickled predictions

        # Tick tracking
        self.current_tick_id = 0
        self.current_command = ecloud.Command.TICK
        self.tick_queue = asyncio.Queue(maxsize=1)
        self.actor_complete_queue = asyncio.Queue()

        # gRPC connections
        self.orchestrator_channel = None
        self.orchestrator_stub = None
        self.server = None

        if self.opt.verbose:
            logger.setLevel(logging.DEBUG)
        elif self.opt.quiet:
            logger.setLevel(logging.WARNING)

    def arg_parse(self):
        parser = argparse.ArgumentParser(description="eCAV Edge Process")
        parser.add_argument('-e', "--edge_index", type=int, required=True,
                           help='Edge index (unique identifier)')
        parser.add_argument('-P', "--edge_port", type=int, default=50054,
                           help='Port for this edge to listen on')
        parser.add_argument('-o', "--orchestrator_port", type=int, default=50051,
                           help='Orchestrator port')
        parser.add_argument('-O', "--orchestrator_ip", type=str, default=ECLOUD_IP,
                           help='Orchestrator IP address')
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

        # Register and receive config
        config = await self.orchestrator_stub.Edge_Register(request)

        self.scenario_yaml_str = config.edge_config_yaml
        self.application = config.application
        self.version = config.version
        self.carla_ip = config.carla_ip
        self.expected_num_actors = config.num_vehicles + config.num_rsus

        logger.info("Edge %s registered successfully. Expected actors: %d vehicles, %d RSUs",
                   self.edge_index, config.num_vehicles, config.num_rsus)
        logger.info("Assigned vehicle indices: %s", list(config.vehicle_indices))
        logger.info("Assigned RSU indices: %s", list(config.rsu_indices))

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

    async def report_tick_complete(self, tick_id):
        """Report to orchestrator that this edge completed processing for the tick."""
        logger.info("Reporting tick %s complete to orchestrator", tick_id)

        request = ecloud.EdgeTickComplete()
        request.edge_index = self.edge_index
        request.tick_id = tick_id
        request.num_actors_processed = len(self.actors)

        await self.orchestrator_stub.Edge_TickComplete(request)

    async def push_tick_to_actors(self, tick_id, command):
        """Push tick notification to all registered actors."""
        logger.info("Pushing tick %s to %d actors", tick_id, len(self.actors))

        tick = ecloud.Tick()
        tick.tick_id = tick_id
        tick.command = command

        push_tasks = []
        for vehicle_index, actor in self.actors.items():
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
            # Push END to actors before stopping
            await self.push_tick_to_actors(tick_id, command)
            return False  # Signal to stop

        # Push tick to all actors
        await self.push_tick_to_actors(tick_id, command)

        # Wait for all actors to report
        actors_reported = 0
        while actors_reported < len(self.actors):
            try:
                vehicle_index = await asyncio.wait_for(
                    self.actor_complete_queue.get(),
                    timeout=30.0
                )
                actors_reported += 1
                logger.debug("Actor %s completed (%d/%d)",
                           vehicle_index, actors_reported, len(self.actors))
            except asyncio.TimeoutError:
                logger.warning("Timeout waiting for actors. Reported: %d/%d",
                             actors_reported, len(self.actors))
                break

        # Fuse predictions (placeholder - just collect them for now)
        self.fuse_predictions()

        # Report completion to orchestrator
        await self.report_tick_complete(tick_id)

        return True  # Continue processing

    def fuse_predictions(self):
        """
        Fuse predictions from all actors.

        This is a placeholder implementation. In a full implementation, this would:
        - Collect perception data from all actors
        - Fuse overlapping detections
        - Generate unified predictions
        - Distribute back to actors
        """
        for vehicle_index, actor in self.actors.items():
            if actor.last_update and actor.last_update.pickled_agent_objects:
                # For now, just pass through the predictions
                # In a full implementation, this would fuse data from multiple sources
                self.fused_predictions[vehicle_index] = actor.last_update.pickled_agent_objects

    async def run(self):
        """Main entry point for the edge process."""
        logger.info("Starting edge process %s", self.edge_index)

        try:
            # Start our server first so orchestrator can push to us
            self.push_server_task = asyncio.create_task(self.run_server())

            # Register with orchestrator
            await self.register_with_orchestrator()

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
