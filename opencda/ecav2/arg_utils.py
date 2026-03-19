# -*- coding: utf-8 -*-
"""Shared argument parser for opencda.py and ecloud_actor_client.py."""

import argparse


def build_arg_parser(description: str) -> argparse.ArgumentParser:
    """
    Return an ArgumentParser pre-populated with arguments common to both
    the root opencda.py entry-point and the distributed ecloud_actor_client.py.

    Callers may add additional arguments before calling parser.parse_args().
    """
    parser = argparse.ArgumentParser(description=description)

    parser.add_argument('-t', "--test_scenario", required=False, type=str, default=None,
                        help='Name of the scenario to test. Must match a script in '
                             'opencda/scenario_testing/ and a YAML in '
                             'opencda/scenario_testing/config_yaml/. '
                             'If omitted in distributed mode the scenario is fetched '
                             'from the ecloud server.')
    parser.add_argument('-d', "--distributed", action='store_true',
                        help='Enable distributed mode for multi-process vehicle/RSU simulation.')
    parser.add_argument("--record", action='store_true',
                        help='Record and save the simulation process to a .log file.')
    parser.add_argument("--apply_ml", action='store_true',
                        help='Enable ML/DL inference (requires pytorch/sklearn).')
    parser.add_argument('-l', "--litserve", action='store_true',
                        help='Use LitServe for distributed ML inference '
                             '(requires LitServe server on port 18000). '
                             'Offloads model inference to a separate process to reduce '
                             'per-container GPU memory.')
    parser.add_argument('-v', "--version", type=str, default='0.9.15',
                        help='CARLA simulator version. Default: 0.9.15.')
    parser.add_argument("--verbose", action="store_true",
                        help="Enable verbose logging.")
    parser.add_argument('-q', "--quiet", action="store_true",
                        help="Minimize logging output.")
    parser.add_argument('-b', "--build", action="store_true",
                        help="Rebuild gRPC proto files.")
    parser.add_argument('-i', "--vehicle_index", type=int, default=-2,
                        help='Vehicle index for distributed mode. '
                             '-2 (default) = scenario server, '
                             '-1 = all non-ego vehicles, '
                             '>=0 = specific vehicle.')
    parser.add_argument('-T', "--trafficManagerPort", type=int, default=None,
                        help='Override the CARLA Traffic Manager RPC port '
                             '(default: YAML value, typically 8000). '
                             'Each container must bind a unique port when using --network=host.')
    parser.add_argument("--output_dir", default=None,
                        help="Output directory for logs and evaluation results.")

    return parser
