"""
EdgeRegistrationServer: asyncio gRPC server that handles Edge_Register from edge containers.

In edge-only distributed mode (-eo), ecav.py runs this server instead of the C++ ecloud_server.
Edge containers start with --orchestrator_ip <ecav_host> --orchestrator_port 50055 and call
Edge_Register on startup. This server assigns sequential edge IDs, sends each edge its scenario
config, and signals completion when all expected edges have registered.

Typical usage (from run_scenario):

    server = EdgeRegistrationServer(scenario_params, port=opt.edge_reg_port)
    fusion_clients = asyncio.get_event_loop().run_until_complete(
        server.start_and_wait(timeout_s=120.0)
    )
"""

import asyncio
import json
import logging
import time

import grpc

logger = logging.getLogger(__name__)

# Lazy imports — proto stubs are on sys.path after ecav.py startup
def _proto():
    import ecav.protos.ecloud_pb2 as ecloud
    import ecav.protos.ecloud_pb2_grpc as ecloud_rpc
    return ecloud, ecloud_rpc


class _EdgeRegistrationServicer:
    """
    gRPC servicer implementing only Edge_Register.
    All other RPCs return UNIMPLEMENTED — they are never called in -eo mode.
    """

    def __init__(self, scenario_params, expected_num_edges: int, registration_event: asyncio.Event):
        self._scenario = scenario_params
        self._expected = expected_num_edges
        self._event = registration_event
        self._lock = asyncio.Lock()
        self._next_edge_id = 0
        # registered edges: edge_id → (edge_ip, edge_port)
        self.registrations: dict = {}

        # Pre-build actor index maps once (same logic as sim_api._build_edge_mapping_setup)
        self._vehicle_indices_per_edge, self._rsu_indices_per_edge = self._build_actor_maps()

    def _build_actor_maps(self):
        """Assign contiguous global indices to actors per edge (mirrors sim_api logic)."""
        edge_list = self._scenario.get('scenario', {}).get('edge_list', [])
        vehicle_map = {}
        rsu_map = {}
        g_veh = 0
        g_rsu = 0
        for edge_idx, edge_cfg in enumerate(edge_list):
            veh_idxs = []
            rsu_idxs = []
            for _ in edge_cfg.get('vehicles', []):
                veh_idxs.append(g_veh)
                g_veh += 1
            for _ in edge_cfg.get('rsus', []):
                rsu_idxs.append(g_rsu)
                g_rsu += 1
            vehicle_map[edge_idx] = veh_idxs
            rsu_map[edge_idx] = rsu_idxs
        return vehicle_map, rsu_map

    async def Edge_Register(self, request, context):
        ecloud, _ = _proto()

        async with self._lock:
            edge_id = self._next_edge_id
            self._next_edge_id += 1
            self.registrations[edge_id] = (request.edge_ip, request.edge_port)

        edge_list = self._scenario.get('scenario', {}).get('edge_list', [])
        if edge_id >= len(edge_list):
            context.set_code(grpc.StatusCode.OUT_OF_RANGE)
            context.set_details(
                f"Edge {edge_id} registered but scenario only has {len(edge_list)} edge(s)"
            )
            return ecloud.EdgeScenarioConfig()

        veh_idxs = self._vehicle_indices_per_edge.get(edge_id, [])
        rsu_idxs = self._rsu_indices_per_edge.get(edge_id, [])

        logger.info(
            "Edge_Register: assigned edge_id=%d (ip=%s port=%d) "
            "vehicles=%s rsus=%s  [%d/%d registered]",
            edge_id, request.edge_ip, request.edge_port,
            veh_idxs, rsu_idxs,
            len(self.registrations), self._expected,
        )

        config = ecloud.EdgeScenarioConfig(
            edge_index=edge_id,
            edge_config_yaml=json.dumps(dict(self._scenario)),
            num_vehicles=len(veh_idxs),
            num_rsus=len(rsu_idxs),
            carla_ip="",       # empty → edge uses standalone setup (no CARLA)
            carla_port=0,
        )
        config.vehicle_indices.extend(veh_idxs)
        config.rsu_indices.extend(rsu_idxs)

        if len(self.registrations) >= self._expected:
            self._event.set()

        return config

    # All other Ecloud RPCs — return UNIMPLEMENTED
    async def __getattr__(self, name):
        async def _unimplemented(*args, **kwargs):
            ctx = args[1] if len(args) > 1 else None
            if ctx is not None:
                ctx.set_code(grpc.StatusCode.UNIMPLEMENTED)
                ctx.set_details(f"{name} not implemented in edge-only registration server")
            ecloud, _ = _proto()
            return ecloud.Empty()
        return _unimplemented


class EdgeRegistrationServer:
    """
    Asyncio gRPC server for edge container registration in edge-only distributed mode.
    """

    def __init__(self, scenario_params, port: int = 50055):
        self._scenario = scenario_params
        self._port = port
        edge_list = scenario_params.get('scenario', {}).get('edge_list', [])
        self._expected_num_edges = len(edge_list)

    async def start_and_wait(self, timeout_s: float = 120.0):
        """
        Start gRPC server, wait until all expected edge containers have registered,
        then return a list of EdgeFusionClient instances (one per edge, not yet connected).

        Raises RuntimeError on timeout.
        """
        from ecav.scenario_testing.utils.edge_fusion_client import EdgeFusionClient
        ecloud, ecloud_rpc = _proto()

        registration_event = asyncio.Event()
        servicer = _EdgeRegistrationServicer(
            self._scenario, self._expected_num_edges, registration_event
        )

        server = grpc.aio.server()
        ecloud_rpc.add_EcloudServicer_to_server(servicer, server)
        server.add_insecure_port(f"0.0.0.0:{self._port}")
        await server.start()
        logger.info(
            "EdgeRegistrationServer: listening on 0.0.0.0:%d, "
            "waiting for %d edge(s) ...",
            self._port, self._expected_num_edges,
        )

        try:
            await asyncio.wait_for(registration_event.wait(), timeout=timeout_s)
        except asyncio.TimeoutError:
            await server.stop(grace=0)
            raise RuntimeError(
                f"EdgeRegistrationServer: only {len(servicer.registrations)}/"
                f"{self._expected_num_edges} edges registered within {timeout_s:.0f}s"
            )

        logger.info("EdgeRegistrationServer: all %d edge(s) registered — stopping server",
                    self._expected_num_edges)
        await server.stop(grace=2)

        # Build fusion clients (connect() called by caller after returning)
        clients = []
        for edge_id in sorted(servicer.registrations):
            edge_ip, edge_port = servicer.registrations[edge_id]
            client = EdgeFusionClient(edge_ip=edge_ip, edge_port=edge_port, edge_index=edge_id)
            clients.append(client)

        return clients
