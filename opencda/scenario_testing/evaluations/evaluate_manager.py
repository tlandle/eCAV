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

            loc_debug_helper = vm.agent.debug_helper
            figure, perform_txt, metrics = loc_debug_helper.evaluate()
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

    def localization_eval(self, log_file):
        """
        Localization module evaluation.

        Args:
            -log_file (File): The log file to write the data.
        """
        lprint(log_file, "***********Localization Module***********")
        for vid, vm in self.cav_world.get_vehicle_managers().items():
            actor_id = vm.vehicle.id
            #print("actor_id = vm.vehicle.id")
            lprint(log_file, 'Actor ID: %d' % actor_id)

            loc_debug_helper = vm.localizer.debug_helper
            #print("loc_debug_helper = vm.localizer.debug_helper")
            figure, perform_txt = loc_debug_helper.evaluate()
            #print("figure, perform_txt = loc_debug_helper.evaluate()")

            # save plotting
            figure_save_path = os.path.join(
                self.eval_save_path,
                '%d_localization_plotting.eps' %
                actor_id)
            figure.savefig(figure_save_path, format='eps', dpi=1200)
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
        Measure client-side perception + tracking latency.

        For every vehicle:
            perception_list = list[float]  (ms)
            tracking_list   = list[float]  (ms)

        Metrics stored:
            self.global_metrics["vehicles"][vid]["perception_mean_ms"]
            self.global_metrics["vehicles"][vid]["perception_std_ms"]
            self.global_metrics["vehicles"][vid]["tracking_mean_ms"]
            self.global_metrics["vehicles"][vid]["tracking_std_ms"]
            self.global_metrics["perception_mean_ms"]   (scenario average)
            self.global_metrics["tracking_mean_ms"]
        """

        lprint(log_file, "***********Client Perception Tracking Module***********")

        per_vehicle = {}    # vid → dict with four stats

        for vid, vm in self.cav_world.get_vehicle_managers().items():
            v_id = str(vm.vehicle.id)

            perc = vm.debug_helper.get_debug_data()["client_perception_time"]
            track = vm.debug_helper.get_debug_data()["client_tracking_time"]

            perc_mean = float(np.mean(perc)) if perc else 0.0
            perc_std  = float(np.std (perc)) if perc else 0.0
            track_mean= float(np.mean(track)) if track else 0.0
            track_std = float(np.std (track)) if track else 0.0

            per_vehicle[v_id] = {
                "perception_mean_ms": perc_mean,
                "perception_std_ms":  perc_std,
                "tracking_mean_ms":   track_mean,
                "tracking_std_ms":    track_std,
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
            count = len(vm.debug_helper.get_debug_data()["client_collisions_list"])
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
        Collision evaluation.

        Args:
            -log_file (File): The log file to write the data.
        """
        lprint(log_file, "***********Lane Invasion Module***********")
        data = []
        vehicle_ids = []
        for vid, vm in self.cav_world.get_vehicle_managers().items():
            actor_id = vm.vehicle.id
            print("actor_id = vm.vehicle.id")
            vehicle_ids.append(actor_id)

            client_debug_helper = vm.debug_helper
            data.append(len(client_debug_helper.get_debug_data()["client_lane_invasions_list"]))            
        
        d = {'vehicle_ids' : vehicle_ids, 'lane_invasions' : data }
        pdvehiclescollisions = pd.DataFrame(d)
        print(data)
        ax = sns.barplot(data=pdvehiclescollisions, x = 'vehicle_ids', y='lane_invasions')
        ax.get_figure().show()
        # save plotting
        figure_save_path = os.path.join(
            self.eval_save_path,
            'lane_invasions_plotting.eps')
        ax.get_figure().savefig(figure_save_path, format='eps', dpi=1200)
        figure_save_path = os.path.join(
            self.eval_save_path,
            'lane_invasions_plotting.png')
        ax.get_figure().savefig(figure_save_path, format='png')
        plt.close(ax.get_figure())


        # save log txt
        lprint(log_file, sum(data))

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
