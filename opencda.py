# -*- coding: utf-8 -*-
"""
Script to run different scenarios.
"""

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import argparse
import importlib
import os
import sys
import subprocess
from omegaconf import OmegaConf
import torch
import warnings
import json

import grpc

from opencda.version import __version__
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
    # create an argument parser
    parser = argparse.ArgumentParser(description="OpenCDA scenario runner.")
    # add arguments to the parser
    parser.add_argument('-t', "--test_scenario", required=False, type=str, default=None,
                            help='Define the name of the scenario you want to test. The given name must'
                             'match one of the testing scripts(e.g. single_2lanefree_carla) in '
                             'opencda/scenario_testing/ folder'
                             ' as well as the corresponding yaml file in opencda/scenario_testing/config_yaml.'
                             ' If not provided and --distributed is set, the scenario will be fetched from the ecloud server.')
    parser.add_argument('-d', "--distributed", action='store_true',
                            help='whether to run the scenario in distributed mode.')
    parser.add_argument("--record", action='store_true',
                            help='whether to record and save the simulation process to .log file')
    parser.add_argument("--apply_ml", action='store_true',
                            help='whether ml/dl framework such as sklearn/pytorch is needed in the testing. '
                             'Set it to true only when you have installed the pytorch/sklearn package.')
    parser.add_argument('-l', "--litserve", action='store_true',
                            help='Use LitServe for distributed ML inference (requires LitServe server on port 18000). '
                             'This offloads ML model inference to a separate process to reduce GPU memory per container.')
    parser.add_argument('-v', "--version", type=str, default='0.9.15',
                            help='Specify the CARLA simulator version, default'
                             'is 0.9.15')
    parser.add_argument("--verbose", action="store_true",
                            help="Make more noise")
    parser.add_argument('-q', "--quiet", action="store_true",
                            help="Make no noise")
    parser.add_argument('-b', "--build", action="store_true",
                            help="Rebuild gRPC proto files")
    parser.add_argument('-i', "--vehicle_index", type=int, default=-2,
                            help='Specify the vehicle index, default is -2; -1 means running all non-ego vehicles.')
    parser.add_argument("--output_dir", default=None)
    # parse the arguments and return the result
    opt = parser.parse_args()
    return opt


def main():
    # parse the arguments
    opt = arg_parse()

    if opt.verbose:
        logger.setLevel(logging.DEBUG)
    elif opt.quiet:
        logger.setLevel(logging.WARNING)

    # print the version of OpenCDA
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
    # set the default yaml file
        default_yaml = config_yaml = os.path.join(
            os.path.dirname(os.path.realpath(__file__)),
            'opencda/scenario_testing/config_yaml/default.yaml')
        # set the yaml file for the specific testing scenario
        config_yaml = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                'opencda/scenario_testing/config_yaml/%s.yaml' % opt.test_scenario)
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
        "opencda.scenario_testing.%s" % opt.test_scenario)
    # check if the yaml file for the specific testing scenario exists
    if not opt.distributed and not os.path.isfile(config_yaml):
        sys.exit(
            "opencda/scenario_testing/config_yaml/%s.yaml not found!" % opt.test_scenario)
    # eCLoud
    if opt.build:
        subprocess.run(['python','-m','grpc_tools.protoc','-I./opencda/protos','--python_out=.','--grpc_python_out=.','./opencda//protos/ecloud.proto'])

    if opt.vehicle_index == -2:
        # get the function for running the scenario from the testing script
        scenario_runner = getattr(testing_scenario, 'run_scenario')
        # run the scenario testing
        scenario_runner(opt, scene_dict)

    else: # we're running a specific vehicle
        assert(opt.distributed), "Must run in distributed mode when specifying vehicle index"
        # get the function for running the scenario from the testing script
        vehicle_runner = getattr(testing_scenario, 'run_vehicle')
        # run the scenario testing
        scene_dict.scenario_runner.vehicle_index = opt.vehicle_index
        vehicle_runner(opt, scene_dict)


if __name__ == '__main__':
    try:
        warnings.simplefilter(action='ignore', category=FutureWarning)

        sys.path.insert(0,'/opt/carla-simulator/PythonAPI/carla') 
        sys.path.insert(0, os.path.join(os.getcwd(), 'opencda'))
        sys.path.insert(0, os.path.join(os.getcwd(), 'scenario_runner'))
        sys.path.insert(0, os.getcwd())
        print("\nModule search paths:")
        for p in sys.path:
            print(p)
        main()
    except KeyboardInterrupt:
         logger.info('exited by user.')
    except SystemExit as exit:
         logger.info('system exit - %s', exit)
    except:
        logger.exception('unhandled exception')
