# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import asyncio
import os
import time
from multiprocessing import Process

import carla
import ecloud_pb2 as ecloud

import scenario_runner.scenario_runner as sr
import ecav.scenario_testing.utils.sim_api as sim_api
from ecav.core.common.cav_world import CavWorld
from ecav.scenario_testing.evaluations.evaluate_manager import EvaluationManager
from ecav.scenario_testing.utils.yaml_utils import add_current_time
from ecav.scenario_testing.utils.edge_fusion_client import EdgeFusionClient
from ecav.scenario_testing.utils.edge_registration_server import EdgeRegistrationServer

MAX_STEP = 600
# SCENARIO_MAX_STEP overrides the cap for a run (a start farther south needs more ticks); test_runner records it.
_MAX_STEP_RUN = int(os.environ.get("SCENARIO_MAX_STEP", MAX_STEP))
SCENARIO_NAME = 'scp_edge_late_fusion'
scenario_runner = None


class SyncCrossLaunch:
    """CROSS_SYNC_LAUNCH="<lead_s>:<speed_mps>": this process launches the crossing car, in simulation time.

    The scenario runner process reads the simulator asynchronously, so a launch it decides comes a number
    of ticks late that depends on the wall-clock duration of a tick (tick 260 with one vehicle, 252 with
    four, 2026-11-16), and the encounter geometry changes with the load. With this option the main loop
    checks the connected vehicle's time to the conflict point after every world tick and, in the tick in
    which it falls to lead_s, gives the crossing car a constant velocity of speed_mps along its heading.
    The scenario runner's own trigger has to be off (CROSS_TRIGGER_TTA=<speed>:0).
    """
    CONFLICT = (-84.8, 127.9)
    START = (-35.0, 127.7)

    def __init__(self, run_dir=""):
        spec = os.environ.get("CROSS_SYNC_LAUNCH")
        self.on = bool(spec)
        self.done = False
        self.car = None
        if self.on:
            lead, _, speed = spec.partition(":")
            self.lead = float(lead)
            self.speed = float(speed)
            # CROSS_SYNC_JITTER=<s>: the lead of a run is drawn uniformly within plus or minus this value,
            # seeded by the run's repetition number (the rep_<n> or run_<n> part of its output directory), so that
            # repetitions sample the encounter geometry around the collision course and still repeat.
            jit = float(os.environ.get("CROSS_SYNC_JITTER", 0) or 0)
            if jit > 0:
                import random
                import re
                m = re.findall(r"(?:run|rep)_(\d+)", str(run_dir or ""))
                rep = int(m[-1]) if m else 0
                self.lead += random.Random(104729 * rep + 7).uniform(-jit, jit)
            print("[SYNC LAUNCH] lead %.3f s, crossing speed %.1f m/s" % (self.lead, self.speed), flush=True)

    def step(self, ego, step):
        if not self.on or self.done:
            return
        import math
        if self.car is None:
            cands = [a for a in ego.get_world().get_actors().filter("vehicle.tesla.model3")
                     if math.hypot(a.get_location().x - self.START[0], a.get_location().y - self.START[1]) < 15.0
                     and a.get_location().z > -50.0]
            if not cands:
                return
            self.car = min(cands, key=lambda a: math.hypot(a.get_location().x - self.START[0],
                                                             a.get_location().y - self.START[1]))
        loc = ego.get_location()
        vel = ego.get_velocity()
        spd = math.hypot(vel.x, vel.y)
        dist = math.hypot(loc.x - self.CONFLICT[0], loc.y - self.CONFLICT[1])
        if spd > 1.0 and dist / spd <= self.lead:
            # The check runs once per tick, so the time to the conflict point is already below the lead by up
            # to one tick when the launch fires. The car starts that much farther along its path, so the lead
            # of the run is the drawn value and not the value rounded to a tick.
            adv = max(0.0, self.lead - dist / spd) * self.speed
            if adv > 0.0:
                tf = self.car.get_transform()
                fwd = tf.get_forward_vector()
                tf.location.x += fwd.x * adv
                tf.location.y += fwd.y * adv
                self.car.set_transform(tf)
            self.car.set_simulate_physics(True)
            self.car.enable_constant_velocity(carla.Vector3D(self.speed, 0.0, 0.0))
            self.done = True
            print("[SYNC LAUNCH] step=%d ego=(%.1f,%.1f) v=%.2f tta=%.2f car=%d at (%.1f,%.1f) advanced %.2f m" % (
                step, loc.x, loc.y, spd, dist / spd, self.car.id, self.car.get_location().x,
                self.car.get_location().y, adv), flush=True)


def exec_scenario_runner(scenario_params):
    """
    Execute the ScenarioRunner process

    Parameters
    ----------
    scenario_params: Parameters of ScenarioRunner

    Returns
    -------
    """
    scenario_runner = sr.ScenarioRunner(scenario_params.scenario_runner)
    scenario_runner.run()
    scenario_runner.destroy()


def run_vehicle(opt, scenario_params):
    """
    Execute a distributed vehicle actor.

    This function is called when running in distributed mode with a specific
    vehicle index. Each vehicle runs in its own process/container.

    Parameters
    ----------
    opt: Command line options
    scenario_params: Parameters of ScenarioRunner

    Returns
    -------
    """
    assert opt.distributed, "Must run in distributed mode when specifying vehicle index"
    try:
        scenario_runner = sr.ScenarioRunner(scenario_params.scenario_runner)
        scenario_runner.run()
        scenario_runner.destroy()
    except Exception as e:
        print(f"vehicle_index: {scenario_params.scenario_runner.vehicle_index}")
        raise e


def run_scenario(opt, scenario_params):
    """
    Run the Late Fusion scenario, either in sequential or distributed mode.

    Late Fusion uses YOLO-based detection from each agent, fused at the edge
    via AB3DMOT tracking, with linear velocity prediction.

    Parameters
    ----------
    opt: Command line options (includes .distributed flag)
    scenario_params: Scenario configuration
    """
    global scenario_runner
    cav_world = None
    scenario_manager = None
    eval_manager = None
    sr_process = None
    edge_list = []
    step = 0

    fusion_clients = []  # EdgeFusionClient list — populated in -eo mode

    try:
        scenario_params = add_current_time(scenario_params)

        # Expose the run's output dir on the config so downstream components
        # (e.g. the conflict-kinematics logger in the edge manager) can write
        # their traces into this run's own directory instead of a shared path.
        if getattr(opt, 'output_dir', None):
            scenario_params['run_output_dir'] = opt.output_dir

        # Create CAV world with config for ML manager settings
        cav_world = CavWorld(
            apply_ml=opt.apply_ml,
            config=scenario_params,
            litserve=getattr(opt, 'litserve', False)
        )

        # Create scenario manager
        scenario_manager = sim_api.ScenarioManager(
            scenario_params,
            opt.apply_ml,
            opt.version,
            town=scenario_params.scenario_runner.town,
            cav_world=cav_world,
            distributed=opt.distributed
        )

        if opt.distributed:
            # Distributed mode: wait for actors to connect via gRPC
            asyncio.get_event_loop().run_until_complete(scenario_manager.run_comms())
        elif getattr(opt, 'edge_only', False):
            # Edge-only distributed mode: start registration server, wait for edge containers
            reg_server = EdgeRegistrationServer(
                scenario_params=dict(scenario_params),
                port=getattr(opt, 'edge_reg_port', 50055),
            )
            fusion_clients = asyncio.get_event_loop().run_until_complete(
                reg_server.start_and_wait(timeout_s=120.0)
            )
            for fc in fusion_clients:
                fc.connect(retry_timeout_s=60.0)
            print(f"[EDGE-ONLY] {len(fusion_clients)} edge(s) ready", flush=True)
            sr_process = Process(target=exec_scenario_runner, args=(scenario_params,))
            sr_process.start()
        else:
            # Sequential mode: launch ScenarioRunner in subprocess
            print("Scenario params Scenario Runner: %s" % scenario_params.scenario_runner)
            sr_process = Process(target=exec_scenario_runner,
                                 args=(scenario_params,))
            sr_process.start()

        world = scenario_manager.world
        ego_vehicle = None
        num_actors = 0

        while ego_vehicle is None or num_actors < scenario_params.scenario_runner.num_actors:
            print("Waiting for the actors")
            time.sleep(2)
            vehicles = world.get_actors().filter('vehicle.*')
            walkers = world.get_actors().filter('walker.*')
            for vehicle in vehicles:
                if vehicle.attributes['role_name'] == 'hero' and ego_vehicle is None:
                    print("Ego vehicle found")
                    ego_vehicle = vehicle
            num_actors = len(vehicles) + len(walkers)
        print(f'Found all {num_actors} actors')
        # driving background traffic under CARLA's traffic manager, created here because this process owns the tick
        # (carla_traffic_manager.range in the YAML, test_runner.py --tm-traffic; empty by default)
        if scenario_params.get('carla_traffic_manager', {}).get('range'):
            # spawn points, models and speeds are drawn per repetition (the run_<n> or rep_<n> part of the output
            # directory), so that the repetitions of a cell are different traffic scenes and each of them still repeats
            import random as _random
            import re as _re
            _m = _re.findall(r"(?:run|rep)_(\d+)", str(getattr(opt, 'output_dir', '') or ''))
            # the vehicle count enters the seed: a denser cell is then a new scene and not a superset of a sparser one
            _seed = 1000 * int(scenario_params['carla_traffic_manager']['range'][0][-1]) + (int(_m[-1]) if _m else 0)
            _state = _random.getstate()
            _random.seed(_seed)
            _tm, _bg = scenario_manager.create_traffic_carla()
            _tm.set_random_device_seed(_seed)
            _random.setstate(_state)
            print(f'[TM-TRAFFIC] {len(_bg)} background vehicles driving under the traffic manager, seed {_seed}', flush=True)

        other_vehicles = []

        world_dt = scenario_params['world']['fixed_delta_seconds']
        edge_dt = scenario_params['edge_base']['edge_dt']
        assert edge_dt % world_dt == 0, \
            "edge_dt must be an exact multiple of world_dt"

        try:
            edge_list = scenario_manager.create_edge_manager_from_scenario_runner(
                application=['edge'],
                edge_dt=edge_dt,
                world_dt=world_dt,
                ego_vehicle=ego_vehicle,
                other_vehicles=other_vehicles,
            )
        except AssertionError as err:
            import traceback, sys
            print("\n\n>>> ASSERTION INSIDE create_edge_manager_from_scenario_runner <<<")
            traceback.print_exc()
            sys.exit(1)
        except Exception:
            import traceback, sys
            traceback.print_exc()
            sys.exit(1)

        # Prefer the YAML's scenario_name so delegated runners (infra_only,
        # perception, etc. that reuse this run_scenario) tag their eval output
        # with the actual scenario, not this module's late_fusion constant.
        _script_name = scenario_params.get('scenario_name', SCENARIO_NAME) \
            if hasattr(scenario_params, 'get') else SCENARIO_NAME
        eval_manager = EvaluationManager(
            scenario_manager.cav_world,
            script_name=_script_name,
            scenario_params=scenario_params,
            current_time=scenario_params['current_time'],
            output_dir=opt.output_dir
        )

        spectator = ego_vehicle.get_world().get_spectator()
        # Bird view following
        spectator_altitude = 133
        spectator_bird_pitch = -90

        sync_launch = SyncCrossLaunch(getattr(opt, 'output_dir', ''))
        flag = True
        while flag:
            # Determine continue condition and command based on mode
            if opt.distributed:
                command = ecloud.Command.PULL_OBJECTS_AND_TICK if step > 0 else ecloud.Command.TICK
                flag = scenario_manager.broadcast_message(command)
                scenario_manager.tick_world()
            else:
                scenario_manager.tick()

            print("about to set ego cav")
            ego_cav = edge_list[0].vehicle_manager_list[0].vehicle
            sync_launch.step(ego_cav, step)
            print("bird view following")

            # Bird view following
            view_transform = carla.Transform()
            view_transform.location = ego_cav.get_transform().location
            print("ego_cav.get_transform().location: %s" % ego_cav.get_transform().location)
            if ego_cav.get_transform().location.x == 0 and ego_cav.get_transform().location.y == 0:
                break
            if opt.distributed and scenario_manager is not None and scenario_manager.all_vehicles_done:
                print(f"All vehicles reported done ({scenario_manager.num_completed_vehicles}/{scenario_manager.vehicle_count}), ending simulation")
                break
            view_transform.location.z = view_transform.location.z + spectator_altitude
            view_transform.rotation.pitch = spectator_bird_pitch
            spectator.set_transform(view_transform)

            # Fusion step:
            #   sequential:      edge manager runs perception + fusion + planning locally
            #   distributed (-d): fusion runs in edge_process containers; nothing to do here
            #   edge-only (-eo): collect detections locally, fuse via RPC, apply predictions
            if getattr(opt, 'edge_only', False):
                for edge, fc in zip(edge_list, fusion_clients):
                    batch = edge.collect_features(step)
                    result = fc.fuse(step, batch)
                    edge.apply_predictions(step, result)
            elif not opt.distributed:
                for edge in edge_list:
                    edge.run_step(step)

            step = step + 1
            if step >= _MAX_STEP_RUN:
                print("Reached maximum step limit, exiting")
                break

            time.sleep(0.001)

    except SystemExit as e:
        print(f"Caught SystemExit({e.code}) in run_scenario - proceeding to evaluation/cleanup")

    except Exception as e:
        print(f"Caught exception {type(e).__name__} in run_scenario: {e} - proceeding to evaluation/cleanup")
        import traceback
        print(traceback.format_exc())

    finally:
        # Terminate ScenarioRunner subprocess FIRST to avoid blocking
        if sr_process is not None:
            sr_process.terminate()
            sr_process.join(timeout=5)
            print("Joined scenario_runner process")

        if scenario_runner is not None:
            scenario_runner.destroy()
            print("Destroyed scenario_runner")

        for edge in edge_list:
            for i, vehicle_manager in enumerate(edge.vehicle_manager_list):
                for vid, step_number in vehicle_manager.vehicles_detected.items():
                    print("VID: %s found VID %s at step %s" % (vehicle_manager.vehicle.id, vid, step_number))

        for fc in fusion_clients:
            fc.end_scenario()
            fc.close()

        if opt.distributed and scenario_manager is not None:
            scenario_manager.end()

        if eval_manager is not None:
            eval_manager.evaluate()

        if cav_world is not None:
            cav_world.close()

        if scenario_manager is not None:
            scenario_manager.close()
            print("Destroyed scenario_manager")
