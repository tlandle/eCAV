# -*- coding: utf-8 -*-
"""
Evaluation manager.
"""

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import subprocess
import os
from datetime import datetime
from opencda.scenario_testing.evaluations.utils import lprint
import json
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
import numpy as np
from omegaconf import OmegaConf

class EvaluationManager(object):
    """
    Evaluation manager to manage the analysis of the
    results for different modules.

    Parameters
    ----------
    cav_world : opencda object
        The CavWorld object that contains all CAVs' information.

    script_name : str
        The current scenario testing name. E.g, single_town06_carla

    current_time : str
        Current timestamp, used to name the output folder.

    scenario_params : dict
        The scenario parameters used for the simulation.
        It is used to name the output folder.

    Attributes
    ----------
    eval_save_path : str
        The final output folder name.

    """

    def __init__(self, cav_world, script_name, current_time, scenario_params=None, output_dir=None):
        """
        Initialize the evaluation manager.
        """

        self.cav_world = cav_world
        self.scenario_params = scenario_params
        
        
        current_path = os.path.dirname(os.path.realpath(__file__))
        if scenario_params['scenario']['single_cav_list']:
            ego_cav_config = OmegaConf.merge(scenario_params["scenario"]["single_cav_list"][0], scenario_params["vehicle_base"])
        elif scenario_params['scenario']['edge_list'][0]['vehicles']:
            ego_cav_config = OmegaConf.merge(scenario_params["scenario"]["edge_list"][0]['vehicles'][0], scenario_params["vehicle_base"])

        if output_dir:                       # override from CLI / runner
            self.eval_save_path = output_dir
        else:
            if scenario_params is None:
                self.eval_save_path = os.path.join(
                    current_path, '../../../evaluation_outputs',
                    script_name + '_' + current_time)
            else:
                self.eval_save_path = os.path.join(
                    current_path, '../../../evaluation_outputs',
                    script_name + "_fog_" + str(scenario_params["world"]["weather"]["fog_density"]) + "_rain_" + str(scenario_params["world"]["weather"]["precipitation"]) + "_clouds_" + str(scenario_params["world"]["weather"]["cloudiness"]) + "_target_speed_ego_" + str(ego_cav_config["behavior"]["max_speed"]), current_time) 
        if not os.path.exists(self.eval_save_path):
            os.makedirs(self.eval_save_path)
        self.global_metrics = {}
        self.global_metrics["target_speed_kmh"] = ego_cav_config["behavior"]["max_speed"]


        #print(f"self.eval_save_path: {self.eval_save_path}") 

    def evaluate(self):
        """
        Evaluate performance of all modules by plotting and writing the
        statistics into the log file.
        """
        log_file = os.path.join(self.eval_save_path, 'log.txt')

        self.localization_eval(log_file)
        print('Localization Evaluation Done.')

        self.kinematics_eval(log_file)
        print('Kinematics Evaluation Done.')

        self.safety_eval(log_file)
        print('Safety Evaluation Done.')

        self.platooning_eval(log_file)
        print('Platooning Evaluation Done.')

        self.client_perception_tracking_eval(log_file)
        print('Client Perception Tracking Evaluation Done.')

        self.edge_eval(log_file)
        print('Edge Evaluation Done.')

        self.lane_invasion_eval(log_file)
        print("Lane Invasion Eval Done.")

        self.collision_eval(log_file)
        print('Collision Eval Done.')

        self.sim_timing_eval(log_file)
        print('Simulation Timing Evaluation Done.')

        self.simulation_eval(log_file)
        print('Simulation Evaluation Done.')

    def kinematics_eval(self, log_file):
        """
        vehicle kinematics related evaluation.

        Args:
            -log_file (File): The log file to write the data.

        """
        lprint(log_file, "***********Kinematics Module***********")
        for vid, vm in self.cav_world.get_vehicle_managers().items():
            actor_id = vm.vehicle.id
            lprint(log_file, 'Actor ID: %d' % actor_id)

            planning_metrics = vm.agent.planning_metrics
            figure, perform_txt, metrics = planning_metrics.evaluate()
            print(metrics)
            veh_dict = self.global_metrics.setdefault("vehicles", {}).setdefault(str(vm.vehicle.id), {})

            veh_dict.update(metrics)

            # save plotting
            figure_save_path = os.path.join(
                self.eval_save_path,
                '%d_kinematics_plotting.eps' %
                actor_id)
            figure.savefig(figure_save_path, format='eps', dpi=1200)
            plt.close(figure)


            lprint(log_file, perform_txt)

    def safety_eval(self, log_file):
        """
        Safety module evaluation using SafetyManager.get_metrics().

        Collects collision history, stuck status, off-road status,
        and traffic light violations per vehicle.

        Args:
            log_file (File): The log file to write the data.
        """
        lprint(log_file, "***********Safety Module***********")

        total_collisions = 0
        total_red_lights = 0
        stuck_vehicles = []
        offroad_vehicles = []

        for vid, vm in self.cav_world.get_vehicle_managers().items():
            actor_id = vm.vehicle.id
            lprint(log_file, f'Actor ID: {actor_id}')

            # Get safety metrics from SafetyManager
            safety_metrics = vm.safety_manager.get_metrics()

            # Store per-vehicle safety data
            veh_dict = self.global_metrics.setdefault("vehicles", {}).setdefault(str(actor_id), {})
            veh_dict.setdefault("safety", {}).update({
                'collision_count': safety_metrics['collision_count'],
                'collision_history': [(f, i) for f, i in safety_metrics['collision_history']],
                'stuck': safety_metrics['stuck'],
                'offroad': safety_metrics['offroad'],
                'red_light_violations': safety_metrics['red_light_violations'],
                'total_traffic_lights': safety_metrics['total_traffic_lights'],
            })

            # Aggregate totals
            total_collisions += safety_metrics['collision_count']
            total_red_lights += safety_metrics['red_light_violations']
            if safety_metrics['stuck']:
                stuck_vehicles.append(actor_id)
            if safety_metrics['offroad']:
                offroad_vehicles.append(actor_id)

            # Log per-vehicle
            lprint(log_file, f"  Collisions: {safety_metrics['collision_count']}")
            lprint(log_file, f"  Red Light Violations: {safety_metrics['red_light_violations']}/{safety_metrics['total_traffic_lights']}")
            lprint(log_file, f"  Stuck: {safety_metrics['stuck']}, Off-road: {safety_metrics['offroad']}")

        # Store scenario-level safety summary
        self.global_metrics['safety_summary'] = {
            'total_collisions': total_collisions,
            'total_red_light_violations': total_red_lights,
            'stuck_vehicle_count': len(stuck_vehicles),
            'offroad_vehicle_count': len(offroad_vehicles),
            'stuck_vehicle_ids': stuck_vehicles,
            'offroad_vehicle_ids': offroad_vehicles,
        }

        lprint(log_file, f"\nSafety Summary:")
        lprint(log_file, f"  Total Collisions: {total_collisions}")
        lprint(log_file, f"  Total Red Light Violations: {total_red_lights}")
        lprint(log_file, f"  Stuck Vehicles: {len(stuck_vehicles)}")
        lprint(log_file, f"  Off-road Vehicles: {len(offroad_vehicles)}")

    def localization_eval(self, log_file):
        """
        Localization module evaluation.

        Args:
            -log_file (File): The log file to write the data.
        """
        lprint(log_file, "***********Localization Module***********")
        for vid, vm in self.cav_world.get_vehicle_managers().items():
            actor_id = vm.vehicle.id
            lprint(log_file, 'Actor ID: %d' % actor_id)

            loc_debug_helper = vm.localizer.localization_metrics
            eval_result = loc_debug_helper.evaluate()

            # Handle both old (figure, text) and new (figure, text, metrics) return formats
            if len(eval_result) == 3:
                figure, perform_txt, metrics = eval_result
                # Store localization metrics in global_metrics
                if metrics:
                    veh_dict = self.global_metrics.setdefault("vehicles", {}).setdefault(str(actor_id), {})
                    veh_dict.setdefault("localization", {}).update(metrics)
            else:
                figure, perform_txt = eval_result

            # save plotting
            figure_save_path = os.path.join(
                self.eval_save_path,
                '%d_localization_plotting.eps' %
                actor_id)
            figure.savefig(figure_save_path, format='eps', dpi=1200)
            figure_save_path_png = os.path.join(
                self.eval_save_path,
                '%d_localization_plotting.png' %
                actor_id)
            figure.savefig(figure_save_path_png, format='png', dpi=150)
            plt.close(figure)

            # save log txt
            lprint(log_file, perform_txt)

    def platooning_eval(self, log_file):
        """
        Platooning evaluation.

        Args:
            -log_file (File): The log file to write the data.

        """
        lprint(log_file, "***********Platooning Analysis***********")

        for pmid, pm in self.cav_world.get_platoon_dict().items():
            lprint(log_file, 'Platoon ID: %s' % pmid)
            figure, perform_txt = pm.evaluate()

            # save plotting
            figure_save_path = os.path.join(
                self.eval_save_path,
                '%s_platoon_plotting.eps' %
                pmid)
            figure.savefig(figure_save_path, format='eps', dpi=1200)
            plt.close(figure)


            # save log txt
            lprint(log_file, perform_txt)

    def edge_eval(self, log_file):
        """
        Edge evaluation.

        Args:
            -log_file (File): The log file to write the data.

        """
        lprint(log_file, "***********Edge Analysis***********")

        if len(self.cav_world.get_edge_dict()) == 0:
            lprint(log_file, "No edge CAVs found, skipping edge evaluation.")
            return

        edge_cav_config = self.scenario_params['edge_base']
        if edge_cav_config is None:
            lprint(log_file, "No edge configuration provided, skipping edge evaluation.")
            return

        self.global_metrics.setdefault("edge_config", {})
        self.global_metrics["edge_config"]["latency"] = edge_cav_config["latency"]

        for pmid, pm in self.cav_world.get_edge_dict().items():
            lprint(log_file, 'Edge ID: %s' % pmid)
            eval_result = pm.evaluate()

            # Handle case where evaluate() returns None or incomplete result
            if eval_result is None:
                lprint(log_file, f'Edge {pmid} evaluate() returned None, skipping.')
                continue

            figure, perform_txt, metrics = eval_result

            # update global metrics with edge metrics
            self.global_metrics.setdefault("edges", {}).setdefault(pmid, {})
            if metrics:
                self.global_metrics["edges"][pmid].update(metrics)

            # save plotting
            if figure is not None:
                figure_save_path = os.path.join(
                    self.eval_save_path,
                    '%s_edge_plotting.eps' %
                    pmid)
                figure.savefig(figure_save_path, format='eps', dpi=1200)
                plt.close(figure)

            # save log txt
            if perform_txt:
                lprint(log_file, perform_txt)
            
    def client_perception_tracking_eval(self, log_file):
        """
        Measure client-side perception, tracking, localization, and control latency.

        Uses get_client_metrics() for aggregated timing statistics.

        Metrics stored:
            self.global_metrics["vehicles"][vid]["perception_mean_ms"]
            self.global_metrics["vehicles"][vid]["perception_std_ms"]
            self.global_metrics["vehicles"][vid]["tracking_mean_ms"]
            self.global_metrics["vehicles"][vid]["tracking_std_ms"]
            self.global_metrics["vehicles"][vid]["localization_mean_ms"]
            self.global_metrics["vehicles"][vid]["control_mean_ms"]
            self.global_metrics["perception_mean_ms"]   (scenario average)
            self.global_metrics["tracking_mean_ms"]
        """

        lprint(log_file, "***********Client Perception Tracking Module***********")

        per_vehicle = {}    # vid → dict with timing stats

        for vid, vm in self.cav_world.get_vehicle_managers().items():
            v_id = str(vm.vehicle.id)

            # Use get_client_metrics() for aggregated stats
            client_metrics = vm.client_metrics.get_client_metrics()

            per_vehicle[v_id] = {
                "perception_mean_ms": client_metrics.get('perception_time_avg_ms') or 0.0,
                "perception_std_ms": client_metrics.get('perception_time_std_ms') or 0.0,
                "tracking_mean_ms": client_metrics.get('tracking_time_avg_ms') or 0.0,
                "tracking_std_ms": client_metrics.get('tracking_time_std_ms') or 0.0,
                "localization_mean_ms": client_metrics.get('localization_time_avg_ms') or 0.0,
                "localization_std_ms": client_metrics.get('localization_time_std_ms') or 0.0,
                "control_mean_ms": client_metrics.get('control_time_avg_ms') or 0.0,
                "control_std_ms": client_metrics.get('control_time_std_ms') or 0.0,
            }

            # store in global metrics structure
            veh_dict = self.global_metrics.setdefault("vehicles", {}).setdefault(str(vm.vehicle.id), {})
            veh_dict.update(per_vehicle[v_id])

        # -------- scenario-level averages -----------------------------------
        if per_vehicle:
            self.global_metrics["perception_mean_ms"] = float(
                np.mean([v["perception_mean_ms"] for v in per_vehicle.values()]))
            self.global_metrics["tracking_mean_ms"] = float(
                np.mean([v["tracking_mean_ms"]   for v in per_vehicle.values()]))

        # -------- plotting ---------------------------------------------------
        if per_vehicle:
            df = pd.DataFrame.from_dict(per_vehicle, orient="index")
            df = df.reset_index(names="vehicle_id")

            fig, ax = plt.subplots(figsize=(7, 4))
            sns.barplot(data=df, x="vehicle_id", y="perception_mean_ms",
                        yerr=df["perception_std_ms"], label="Perception",
                        color="skyblue", ax=ax)
            sns.barplot(data=df, x="vehicle_id", y="tracking_mean_ms",
                        yerr=df["tracking_std_ms"], label="Tracking",
                        color="salmon", ax=ax, alpha=0.8)

            ax.set_xlabel("Vehicle ID")
            ax.set_ylabel("Latency (ms)")
            ax.set_title("Client Perception vs Tracking Latency")
            ax.yaxis.set_major_locator(MaxNLocator(integer=True))
            ax.legend()

            fig.tight_layout()
            for fmt in ("png", "eps"):
                fig.savefig(os.path.join(self.eval_save_path,
                                         f"perception_tracking_latency.{fmt}"),
                            dpi=300)
            plt.close(fig)

        # -------- log line ---------------------------------------------------
        lprint(log_file,
               f"Perception mean(ms): {self.global_metrics.get('perception_mean_ms', 'NA'):.1f}, "
               f"Tracking mean(ms): {self.global_metrics.get('tracking_mean_ms', 'NA'):.1f}")
               
    def collision_eval(self, log_file):
        """
        Collision evaluation.
        Writes metrics into self.global_metrics and saves a bar plot.
        """
        lprint(log_file, "***********Collision Module***********")

        per_vehicle = {}          # vid → count
        for vid, vm in self.cav_world.get_vehicle_managers().items():
            # Use get_client_metrics() for collision count
            client_metrics = vm.client_metrics.get_client_metrics()
            count = client_metrics.get('collision_count', 0)
            per_vehicle[str(vm.vehicle.id)] = count   # JSON needs str keys

        # ---------- store into global metrics -------------------------------
        self.global_metrics["collision_count"] = sum(per_vehicle.values())
        self.global_metrics.setdefault("vehicles", {})
        for vid, n in per_vehicle.items():
             # merge without clobbering existing metrics
            veh_dict = self.global_metrics.setdefault("vehicles", {}).setdefault(vid, {})
            veh_dict["collisions"] = n

        

        # ---------- plotting -------------------------------------------------
        fig, ax = plt.subplots(figsize=(6, 4))

        if per_vehicle:                              # at least one vehicle
            df = pd.DataFrame({
                "vehicle_id": list(per_vehicle.keys()),
                "collisions": list(per_vehicle.values())
            })
            sns.barplot(data=df, x="vehicle_id", y="collisions", ax=ax,
                        palette="Reds_d")
            ax.set_xlabel("Vehicle ID")
            ax.set_ylabel("Collision Count")
            ax.set_title("Collisions per Vehicle")
            ax.bar_label(ax.containers[0], padding=3, fmt="%d")
            ax.yaxis.set_major_locator(plt.MaxNLocator(integer=True))
        else:                                        # no vehicles? unlikely
            ax.text(0.5, 0.5, "No vehicles", ha="center", va="center")
            ax.axis("off")

        fig.tight_layout()
        for fmt in ("png", "eps"):
            fig.savefig(os.path.join(self.eval_save_path,
                                     f"collision_plotting.{fmt}"),
                        dpi=300)

        plt.close(fig)

        # ---------- log txt --------------------------------------------------
        lprint(log_file, f"Total collisions: {self.global_metrics['collision_count']}")

    def lane_invasion_eval(self, log_file):
        """
        Lane invasion evaluation.

        Args:
            -log_file (File): The log file to write the data.
        """
        lprint(log_file, "***********Lane Invasion Module***********")
        data = []
        vehicle_ids = []
        for vid, vm in self.cav_world.get_vehicle_managers().items():
            actor_id = vm.vehicle.id
            vehicle_ids.append(actor_id)

            # Use get_client_metrics() for lane invasion count
            client_metrics = vm.client_metrics.get_client_metrics()
            data.append(client_metrics.get('lane_invasion_count', 0))            
        
        d = {'vehicle_ids' : vehicle_ids, 'lane_invasions' : data }
        pdvehiclescollisions = pd.DataFrame(d)
        print(data)
        fig, ax = plt.subplots(figsize=(6, 4))
        sns.barplot(data=pdvehiclescollisions, x='vehicle_ids', y='lane_invasions', ax=ax)
        ax.set_xlabel("Vehicle ID")
        ax.set_ylabel("Lane Invasions")
        ax.set_title("Lane Invasions per Vehicle")
        # save plotting
        figure_save_path = os.path.join(
            self.eval_save_path,
            'lane_invasions_plotting.eps')
        fig.savefig(figure_save_path, format='eps', dpi=1200)
        figure_save_path = os.path.join(
            self.eval_save_path,
            'lane_invasions_plotting.png')
        fig.savefig(figure_save_path, format='png')
        plt.close(fig)


        # save log txt
        lprint(log_file, sum(data))

    def sim_timing_eval(self, log_file):
        """
        Simulation timing evaluation using SimMetrics.

        Collects world tick time, client tick time, and network latency metrics.

        Args:
            log_file (File): The log file to write the data.
        """
        lprint(log_file, "***********Simulation Timing Module***********")

        scenario_manager = self.cav_world.get_scenario_manager()
        if scenario_manager is None:
            lprint(log_file, "No scenario manager found, skipping simulation timing evaluation.")
            return

        sim_metrics = scenario_manager.sim_metrics.get_sim_metrics()

        # Store in global metrics
        self.global_metrics['simulation_timing'] = {
            'world_tick_avg_ms': sim_metrics.get('world_tick_avg_ms'),
            'world_tick_std_ms': sim_metrics.get('world_tick_std_ms'),
            'client_tick_avg_ms': sim_metrics.get('client_tick_avg_ms'),
            'client_tick_std_ms': sim_metrics.get('client_tick_std_ms'),
            'startup_time_ms': sim_metrics.get('startup_time_ms'),
            'shutdown_time_ms': sim_metrics.get('shutdown_time_ms'),
        }

        # Log timing metrics
        lprint(log_file, f"World Tick: {sim_metrics.get('world_tick_avg_ms', 'N/A'):.2f} ± {sim_metrics.get('world_tick_std_ms', 'N/A'):.2f} ms" if sim_metrics.get('world_tick_avg_ms') else "World Tick: N/A")
        lprint(log_file, f"Client Tick: {sim_metrics.get('client_tick_avg_ms', 'N/A'):.2f} ± {sim_metrics.get('client_tick_std_ms', 'N/A'):.2f} ms" if sim_metrics.get('client_tick_avg_ms') else "Client Tick: N/A")
        lprint(log_file, f"Startup Time: {sim_metrics.get('startup_time_ms', 'N/A')} ms")
        lprint(log_file, f"Shutdown Time: {sim_metrics.get('shutdown_time_ms', 'N/A')} ms")

        # Create timing visualization
        fig, axes = plt.subplots(1, 2, figsize=(10, 4))

        # World tick histogram
        world_ticks = [t for t in scenario_manager.sim_metrics.world_tick_time_list[0] if t is not None]
        if world_ticks:
            axes[0].hist(world_ticks, bins=30, color='steelblue', alpha=0.7, edgecolor='black')
            axes[0].axvline(np.mean(world_ticks), color='red', linestyle='--', label=f'Mean: {np.mean(world_ticks):.2f}ms')
            axes[0].set_xlabel('Time (ms)')
            axes[0].set_ylabel('Frequency')
            axes[0].set_title('World Tick Time Distribution')
            axes[0].legend()
        else:
            axes[0].text(0.5, 0.5, 'No world tick data', ha='center', va='center')
            axes[0].set_title('World Tick Time')

        # Client tick histogram
        client_ticks = [t for t in scenario_manager.sim_metrics.client_tick_time_list[0] if t is not None]
        if client_ticks:
            axes[1].hist(client_ticks, bins=30, color='coral', alpha=0.7, edgecolor='black')
            axes[1].axvline(np.mean(client_ticks), color='red', linestyle='--', label=f'Mean: {np.mean(client_ticks):.2f}ms')
            axes[1].set_xlabel('Time (ms)')
            axes[1].set_ylabel('Frequency')
            axes[1].set_title('Client Tick Time Distribution')
            axes[1].legend()
        else:
            axes[1].text(0.5, 0.5, 'No client tick data', ha='center', va='center')
            axes[1].set_title('Client Tick Time')

        fig.tight_layout()

        # Save figures
        for fmt in ('png', 'eps'):
            fig.savefig(os.path.join(self.eval_save_path, f'simulation_timing.{fmt}'), dpi=300)
        plt.close(fig)

    def simulation_eval(self, log_file):
        """
        Simulation evaluation.

        Args:
            -log_file (File): The log file to write the data.

        """
        lprint(log_file, "***********Simulation Analysis***********")

        # 1) safely call your evaluate()
        try:
            scenario_manager =self.cav_world.get_scenario_manager()
            figure, perform_txt = scenario_manager.evaluate()
            error = None

        except SystemExit as e:
            lprint(log_file, f" Caught SystemExit({e.code}), marking failure")
            figure = plt.figure()               # empty placeholder
            perform_txt = "success_rate: 0.0"    # you can extend this string
            error = f"SystemExit({e.code})"

        except Exception as e:
            lprint(log_file, f"Error during evaluate(): {type(e).__name__}: {e}")
            figure = plt.figure()
            perform_txt = "success_rate: 0.0"
            error = type(e).__name__



        # === SUCCESS definition ===
        coll = self.global_metrics["collision_count"]
        #dead = self.global_metrics["deadlock_detected"]
        self.global_metrics["success_rate"] = 1.0 if (coll == 0) else 0.0


        # 4) write JSON
        json.dump(self.global_metrics,
          open(os.path.join(self.eval_save_path, "simulation_metrics.json"), "w"),
          indent=2)

        # 5) Generate comprehensive text report
        self._generate_text_report()

        # save plotting
        figure_save_path = os.path.join(
            self.eval_save_path,
            'simulation_plotting.eps')
        figure.savefig(figure_save_path, format='eps', dpi=1200)
        figure_save_path = os.path.join(
            self.eval_save_path,
            'simulation_plotting.png')
        figure.savefig(figure_save_path, format='png')

        plt.close(figure)

        # save log txt
        lprint(log_file, perform_txt)

    def _generate_text_report(self):
        """
        Generate a comprehensive text report summarizing all evaluation metrics.
        Saves to evaluation_report.txt in the eval_save_path directory.
        """
        report_path = os.path.join(self.eval_save_path, "evaluation_report.txt")

        with open(report_path, 'w') as f:
            # Header
            f.write("=" * 80 + "\n")
            f.write("              AUTONOMOUS VEHICLE SIMULATION EVALUATION REPORT\n")
            f.write("=" * 80 + "\n\n")

            # Scenario Info
            f.write("SCENARIO CONFIGURATION\n")
            f.write("-" * 40 + "\n")
            f.write(f"Target Speed: {self.global_metrics.get('target_speed_kmh', 'N/A')} km/h\n")
            f.write(f"Success Rate: {self.global_metrics.get('success_rate', 0.0):.1%}\n")
            f.write(f"Total Collisions: {self.global_metrics.get('collision_count', 0)}\n")

            if 'edge_config' in self.global_metrics:
                ec = self.global_metrics['edge_config']
                f.write(f"Edge Latency: {ec.get('latency', 0.0)}s\n")
            f.write("\n")

            # Vehicle Metrics
            f.write("VEHICLE METRICS\n")
            f.write("-" * 40 + "\n")
            vehicles = self.global_metrics.get('vehicles', {})
            for vid, vdata in vehicles.items():
                f.write(f"\nVehicle ID: {vid}\n")
                f.write(f"  Kinematics:\n")
                f.write(f"    Speed: {vdata.get('avg_speed_mps', 0):.2f} ± {vdata.get('std_speed_mps', 0):.2f} m/s\n")
                f.write(f"    Acceleration: {vdata.get('avg_acceleration_mps2', vdata.get('avg_acceleration_mps', 0)):.2f} ± {vdata.get('std_acceleration_mps2', vdata.get('std_acceleration_mps', 0)):.2f} m/s²\n")
                if 'avg_jerk_mps3' in vdata:
                    f.write(f"    Jerk: {vdata.get('avg_jerk_mps3', 0):.2f} ± {vdata.get('std_jerk_mps3', 0):.2f} m/s³ (max: {vdata.get('max_jerk_mps3', 0):.2f})\n")
                if 'avg_lateral_deviation_m' in vdata:
                    f.write(f"    Lateral Deviation: {vdata.get('avg_lateral_deviation_m', 0):.2f} ± {vdata.get('std_lateral_deviation_m', 0):.2f} m (max: {vdata.get('max_lateral_deviation_m', 0):.2f})\n")
                ttc = vdata.get('avg_ttc_s')
                if ttc is not None and not (isinstance(ttc, float) and np.isnan(ttc)):
                    f.write(f"    TTC: {ttc:.2f} ± {vdata.get('std_ttc_s', 0):.2f} s\n")

                f.write(f"  Perception:\n")
                f.write(f"    Perception Latency: {vdata.get('perception_mean_ms', 0):.1f} ± {vdata.get('perception_std_ms', 0):.1f} ms\n")
                f.write(f"    Tracking Latency: {vdata.get('tracking_mean_ms', 0):.1f} ± {vdata.get('tracking_std_ms', 0):.1f} ms\n")

                # Braking Attribution
                if vdata.get('total_brake_events', 0) > 0:
                    f.write(f"  Braking Attribution:\n")
                    f.write(f"    Total Brake Events: {vdata.get('total_brake_events', 0)}\n")
                    f.write(f"    Ghost Brake Events: {vdata.get('ghost_brake_events', 0)}\n")
                    f.write(f"    False Brake Rate: {vdata.get('false_brake_rate', 0):.3f}\n")

                f.write(f"  Safety:\n")
                f.write(f"    Collisions: {vdata.get('collisions', 0)}\n")
                # Include detailed safety metrics if available
                if 'safety' in vdata:
                    safety = vdata['safety']
                    f.write(f"    Red Light Violations: {safety.get('red_light_violations', 0)}/{safety.get('total_traffic_lights', 0)}\n")
                    f.write(f"    Stuck: {safety.get('stuck', False)}\n")
                    f.write(f"    Off-road: {safety.get('offroad', False)}\n")

                # Localization metrics if available
                if 'localization' in vdata:
                    loc = vdata['localization']
                    f.write(f"  Localization:\n")
                    f.write(f"    ATE (filtered): {loc.get('ate_filter_m', 'N/A')} m\n")
                    f.write(f"    ATE (GNSS raw): {loc.get('ate_gnss_m', 'N/A')} m\n")
                    f.write(f"    RPE (filtered): {loc.get('rpe_filter_m', 'N/A')} m\n")
                    f.write(f"    RPE (GNSS raw): {loc.get('rpe_gnss_m', 'N/A')} m\n")

            f.write("\n")

            # Edge Metrics
            f.write("EDGE SERVER METRICS\n")
            f.write("-" * 40 + "\n")
            edges = self.global_metrics.get('edges', {})
            for eid, edata in edges.items():
                f.write(f"\nEdge ID: {eid}\n")

                # Performance
                f.write(f"  Performance:\n")
                f.write(f"    Total Frames: {edata.get('total_frames', 0)}\n")
                f.write(f"    FPS: {edata.get('frames_per_second', 0):.2f}\n")
                f.write(f"    Timing: avg={edata.get('timing_avg_ms', 0):.1f}ms, p95={edata.get('timing_p95_ms', 0):.1f}ms, p99={edata.get('timing_p99_ms', 0):.1f}ms\n")

                # Component breakdown
                f.write(f"  Component Timing (ms):\n")
                f.write(f"    Feature Collection: {edata.get('feature_collection_avg_ms', 0):.1f}\n")
                f.write(f"    Fusion: {edata.get('fusion_avg_ms', 0):.1f}\n")
                f.write(f"    Detection: {edata.get('detection_avg_ms', 0):.1f}\n")
                f.write(f"    Tracking: {edata.get('tracking_avg_ms', 0):.1f}\n")
                f.write(f"    Prediction: {edata.get('prediction_avg_ms', 0):.1f}\n")
                f.write(f"    Distribution: {edata.get('distribution_avg_ms', 0):.1f}\n")

                # Resource Usage
                f.write(f"  Resource Usage:\n")
                f.write(f"    GPU Memory: avg={edata.get('gpu_memory_avg_mb', 0):.1f}MB, peak={edata.get('gpu_memory_peak_mb', 0):.1f}MB\n")
                f.write(f"    GPU Utilization: {edata.get('gpu_utilization_avg_pct', 0):.1f}%\n")
                f.write(f"    CPU: {edata.get('cpu_avg_pct', 0):.1f}%\n")

                # Detection Metrics
                f.write(f"  Detection Metrics:\n")
                f.write(f"    TP: {edata.get('detection_total_tp', 0)}, FP: {edata.get('detection_total_fp', 0)}, FN: {edata.get('detection_total_fn', 0)}\n")
                f.write(f"    Precision: {edata.get('detection_precision', 0):.3f}\n")
                f.write(f"    Recall: {edata.get('detection_recall', 0):.3f}\n")
                f.write(f"    F1 Score: {edata.get('detection_f1', 0):.3f}\n")
                f.write(f"    AP@50: {edata.get('detection_ap_50', 0):.3f}\n")
                f.write(f"    AP@75: {edata.get('detection_ap_75', 0):.3f}\n")
                f.write(f"    mAP: {edata.get('detection_map', 0):.3f}\n")

                # Tracking Metrics
                f.write(f"  Tracking Metrics:\n")
                f.write(f"    MOTA: {edata.get('tracking_avg_mota', 0):.3f}\n")
                f.write(f"    MOTP: {edata.get('tracking_avg_motp', 0):.2f} m\n")
                f.write(f"    IDF1: {edata.get('tracking_avg_idf1', 0):.3f}\n")
                f.write(f"    ID Switches: {edata.get('tracking_total_id_switches', 0)}\n")
                f.write(f"    Fragmentations: {edata.get('tracking_total_fragmentations', 0)}\n")

                # Prediction Metrics
                f.write(f"  Prediction Metrics:\n")
                f.write(f"    ADE @1s: {edata.get('prediction_ade_1s_m', 0):.2f} m\n")
                f.write(f"    ADE @2s: {edata.get('prediction_ade_2s_m', 0):.2f} m\n")
                f.write(f"    ADE @3s: {edata.get('prediction_ade_3s_m', 0):.2f} m\n")
                f.write(f"    FDE: {edata.get('prediction_fde_m', 0):.2f} m\n")
                f.write(f"    Miss Rate: {edata.get('prediction_miss_rate', 0):.1%}\n")
                f.write(f"    minADE: {edata.get('prediction_min_ade_m', 0):.2f} m\n")
                f.write(f"    minFDE: {edata.get('prediction_min_fde_m', 0):.2f} m\n")

                # Ego-Uniqueness Metrics
                eu = edata.get('ego_uniqueness', {})
                if eu:
                    f.write(f"  Ego-Uniqueness:\n")
                    f.write(f"    Total Violations: {eu.get('total_violations', 0)}\n")
                    f.write(f"    Violations/min: {eu.get('violations_per_min', 0):.2f}\n")
                    f.write(f"    Violation Tick Fraction: {eu.get('violation_tick_fraction', 0):.3f}\n")
                    f.write(f"    Duplicate Track Rate: {eu.get('duplicate_track_rate', 0):.4f}\n")
                    f.write(f"    Duplicate Lifetime (mean): {eu.get('duplicate_lifetime_mean_ticks', 0):.1f} ticks\n")
                    f.write(f"    Duplicate Lifetime (p95): {eu.get('duplicate_lifetime_p95_ticks', 0):.1f} ticks\n")
                    f.write(f"    Ego Ghost Violations: {eu.get('ego_ghost_violation_count', 0)}\n")

            # Simulation Timing
            f.write("\n")
            f.write("SIMULATION TIMING\n")
            f.write("-" * 40 + "\n")
            sim_timing = self.global_metrics.get('simulation_timing', {})
            if sim_timing:
                wt_avg = sim_timing.get('world_tick_avg_ms')
                wt_std = sim_timing.get('world_tick_std_ms')
                ct_avg = sim_timing.get('client_tick_avg_ms')
                ct_std = sim_timing.get('client_tick_std_ms')
                f.write(f"World Tick: {wt_avg:.2f} ± {wt_std:.2f} ms\n" if wt_avg else "World Tick: N/A\n")
                f.write(f"Client Tick: {ct_avg:.2f} ± {ct_std:.2f} ms\n" if ct_avg else "Client Tick: N/A\n")
                f.write(f"Startup Time: {sim_timing.get('startup_time_ms', 'N/A')} ms\n")
                f.write(f"Shutdown Time: {sim_timing.get('shutdown_time_ms', 'N/A')} ms\n")
            else:
                f.write("No simulation timing data available.\n")

            # Safety Summary
            f.write("\n")
            f.write("SAFETY SUMMARY\n")
            f.write("-" * 40 + "\n")
            safety_summary = self.global_metrics.get('safety_summary', {})
            if safety_summary:
                f.write(f"Total Collisions: {safety_summary.get('total_collisions', 0)}\n")
                f.write(f"Total Red Light Violations: {safety_summary.get('total_red_light_violations', 0)}\n")
                f.write(f"Stuck Vehicles: {safety_summary.get('stuck_vehicle_count', 0)}\n")
                f.write(f"Off-road Vehicles: {safety_summary.get('offroad_vehicle_count', 0)}\n")
            else:
                f.write("No safety summary data available.\n")

            # Summary Table
            f.write("\n")
            f.write("=" * 80 + "\n")
            f.write("                           METRICS SUMMARY TABLE\n")
            f.write("=" * 80 + "\n\n")

            f.write("| Category       | Metric              | Value          | Unit    |\n")
            f.write("|----------------|---------------------|----------------|--------|\n")

            # Overall
            f.write(f"| Overall        | Success Rate        | {self.global_metrics.get('success_rate', 0):.1%}           |         |\n")
            f.write(f"| Overall        | Collisions          | {self.global_metrics.get('collision_count', 0)}              |         |\n")

            # Detection (from first edge)
            if edges:
                e = list(edges.values())[0]
                f.write(f"| Detection      | Precision           | {e.get('detection_precision', 0):.3f}          |         |\n")
                f.write(f"| Detection      | Recall              | {e.get('detection_recall', 0):.3f}          |         |\n")
                f.write(f"| Detection      | mAP                 | {e.get('detection_map', 0):.3f}          |         |\n")
                f.write(f"| Tracking       | MOTA                | {e.get('tracking_avg_mota', 0):.3f}          |         |\n")
                f.write(f"| Tracking       | MOTP                | {e.get('tracking_avg_motp', 0):.2f}           | m       |\n")
                f.write(f"| Tracking       | IDF1                | {e.get('tracking_avg_idf1', 0):.3f}          |         |\n")
                f.write(f"| Prediction     | ADE @1s             | {e.get('prediction_ade_1s_m', 0):.2f}           | m       |\n")
                f.write(f"| Prediction     | FDE                 | {e.get('prediction_fde_m', 0):.2f}           | m       |\n")
                f.write(f"| Prediction     | Miss Rate           | {e.get('prediction_miss_rate', 0):.1%}          |         |\n")

            # Kinematics (from first vehicle)
            if vehicles:
                v = list(vehicles.values())[0]
                f.write(f"| Kinematics     | Avg Speed           | {v.get('avg_speed_mps', 0):.2f}           | m/s     |\n")
                if 'avg_jerk_mps3' in v:
                    f.write(f"| Kinematics     | Max Jerk            | {v.get('max_jerk_mps3', 0):.2f}           | m/s³    |\n")

            f.write("\n")
            f.write("=" * 80 + "\n")
            f.write("Report generated by eCAV Evaluation Manager\n")
            f.write("=" * 80 + "\n")

        print(f"Comprehensive evaluation report saved to: {report_path}")


def create_bar_plot(data, x, y, labels):
    """
    Create a bar plot using seaborn.

    Args:
    data (pd.DataFrame): The DataFrame containing the data to be plotted.
    x (str): The column name for the x-axis variable.                                                                                                                                                                                                                              
    y (str): The column name for the y-axis variable.                                                                                                                                                                                                                              
    labels (dict): A dictionary containing the labels for the plot (xlabel, ylabel, title).                                                                                                                                                                                        
                                                                                                                                                                                                                                                                                   
    Returns:                                                                                                                                                                                                                                                                       
    Axes: The axis object containing the box plot.                                                                                                                                                                                                                                 
    """                                                                                                                                                                                                                                                                            
    ax = sns.barplot(data=data, x=x, y=y)                                                                                                                                                                                                                                          
    ax.set(xlabel=labels['xlabel'],                                                          
           ylabel=labels['ylabel'],                                                                                                     
           title=labels['title'])                                                                                                       
    return ax
