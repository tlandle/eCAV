# -*- coding: utf-8 -*-
"""
Script to run different scenarios.

Supports both sequential and distributed execution modes:
- Sequential mode: All components run in the same process (Python 3.7 with BM2CP)
- Distributed mode: Vehicles/RSUs run as separate processes/containers (Python 3.10)

Usage:
    # Sequential mode
    python ecav.py -t openscenario_3_edge_worldfusion --apply_ml

    # Distributed mode (server)
    python ecav.py -t openscenario_3_edge -d

    # Distributed mode (vehicle client)
    python ecav.py -d -i 0

Authors: Runsheng Xu <rxx3386@ucla.edu>
         Tyler Landle <tlandle3@gatech.edu>
License: MIT
"""

import importlib
import os
import sys
import subprocess
from omegaconf import OmegaConf
import torch
import warnings
import json

import grpc

from ecav.version import __version__
from ecav.ecav2.arg_utils import build_arg_parser
import coloredlogs, logging

logger = logging.getLogger(__name__)
coloredlogs.install(level='INFO', logger=logger)

# Default ecloud server address
ECLOUD_SERVER_ADDRESS = "localhost:50051"


def fetch_scenario_from_server(vehicle_index: int = -1) -> str:
    """
    Fetch the test scenario name from the ecloud server.
    Used when running in distributed mode without specifying a scenario.

    Parameters
    ----------
    vehicle_index : int
        Optional vehicle index for logging/tracking purposes.

    Returns
    -------
    str
        The test scenario name from the server.
    """
    import ecloud_pb2 as ecloud
    import ecloud_pb2_grpc as ecloud_rpc

    logger.info("Fetching scenario from ecloud server at %s", ECLOUD_SERVER_ADDRESS)

    # Create a synchronous channel and stub
    channel = grpc.insecure_channel(ECLOUD_SERVER_ADDRESS)
    stub = ecloud_rpc.EcloudStub(channel)

    # Create the request
    request = ecloud.ScenarioRequest(vehicle_index=vehicle_index)

    # Call the RPC (with retry logic)
    max_retries = 10
    retry_delay = 2  # seconds

    for attempt in range(max_retries):
        try:
            sim_info = stub.Client_GetScenario(request)
            test_scenario = sim_info.test_scenario

            if test_scenario:
                logger.info("Received scenario from server: %s", test_scenario)
                channel.close()
                return test_scenario
            else:
                logger.warning("Server returned empty scenario (attempt %d/%d)",
                             attempt + 1, max_retries)
        except grpc.RpcError as e:
            logger.warning("Failed to fetch scenario (attempt %d/%d): %s",
                         attempt + 1, max_retries, e.details() if hasattr(e, 'details') else str(e))

        if attempt < max_retries - 1:
            import time
            time.sleep(retry_delay)

    channel.close()
    raise RuntimeError("Failed to fetch scenario from ecloud server after %d attempts" % max_retries)


def arg_parse():
    parser = build_arg_parser("eCAV scenario runner.")
    opt = parser.parse_args()
    return opt


def main():
    # parse the arguments
    opt = arg_parse()

    # LitServe mode implies apply_ml (ML inference via remote server)
    if opt.litserve:
        opt.apply_ml = True

    if opt.verbose:
        logger.setLevel(logging.DEBUG)
    elif opt.quiet:
        logger.setLevel(logging.WARNING)

    # print the version of eCAV
    logger.info("eCAV Version: %s", __version__)

    scene_dict = None
    config_yaml = None

    # If distributed mode and no scenario specified, fetch from server
    if opt.distributed and opt.test_scenario is None:
        logger.info("Distributed mode enabled without scenario - fetching from ecloud server")
        test_scenario_json = fetch_scenario_from_server(opt.vehicle_index)
        scene_json = json.loads(test_scenario_json)
        scene_dict = OmegaConf.create(scene_json)
        scene_dict.distributed = True
        opt.test_scenario = scene_dict.get('scenario_name')
    else:
        # Require test_scenario in non-distributed mode
        if opt.test_scenario is None:
            logger.error("--test_scenario is required unless running in distributed mode (-d) without a scenario")
            sys.exit(1)

        # set the default yaml file
        default_yaml = os.path.join(
            os.path.dirname(os.path.realpath(__file__)),
            'ecav/scenario_testing/config_yaml/default.yaml')
        # set the yaml file for the specific testing scenario
        config_yaml = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                   'ecav/scenario_testing/config_yaml/%s.yaml' % opt.test_scenario)
        # load the default yaml file and the scenario yaml file as dictionaries
        default_dict = OmegaConf.load(default_yaml)
        scene_dict = OmegaConf.load(config_yaml)
        # merge the dictionaries
        scene_dict = OmegaConf.merge(default_dict, scene_dict)

    assert scene_dict is not None, "scene_dict is not initialized!"
    scene_dict.scenario_runner.distributed = opt.distributed

    if opt.apply_ml:
        torch.cuda.init()

    # import the testing script
    testing_scenario = importlib.import_module(
        "ecav.scenario_testing.%s" % opt.test_scenario)

    # check if the yaml file for the specific testing scenario exists
    if not opt.distributed and config_yaml and not os.path.isfile(config_yaml):
        sys.exit(
            "ecav/scenario_testing/config_yaml/%s.yaml not found!" % opt.test_scenario)

    # Rebuild gRPC proto files if requested
    if opt.build:
        subprocess.run([
            'python', '-m', 'grpc_tools.protoc',
            '-I./ecav/protos',
            '--python_out=.',
            '--grpc_python_out=.',
            './ecav/protos/ecloud.proto'
        ])

    if opt.vehicle_index == -2:
        # Run the main scenario (server mode in distributed, or full scenario in sequential)
        scenario_runner = getattr(testing_scenario, 'run_scenario')
        scenario_runner(opt, scene_dict)
    else:
        # Run a specific vehicle (distributed mode only)
        assert opt.distributed, "Must run in distributed mode (-d) when specifying vehicle index"
        vehicle_runner = getattr(testing_scenario, 'run_vehicle')
        scene_dict.scenario_runner.vehicle_index = opt.vehicle_index
        vehicle_runner(opt, scene_dict)


if __name__ == '__main__':
    try:
        warnings.simplefilter(action='ignore', category=FutureWarning)

        sys.path.insert(0, '/opt/carla-simulator/PythonAPI/carla')
        sys.path.insert(0, os.path.join(os.getcwd(), 'ecav'))
        sys.path.insert(0, os.path.join(os.getcwd(), 'ecav', 'protos'))
        sys.path.insert(0, os.path.join(os.getcwd(), 'scenario_runner'))
        sys.path.insert(0, os.getcwd())

        main()
    except KeyboardInterrupt:
        logger.info('exited by user.')
    except SystemExit as exit:
        logger.info('system exit - %s', exit)
    except:
        logger.exception('unhandled exception')
