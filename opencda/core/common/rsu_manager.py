# -*- coding: utf-8 -*-
"""
Basic class for RSU(Roadside Unit) management.
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import pickle

from opencda.core.common.data_dumper import DataDumper
from opencda.core.sensing.perception.perception_manager import \
    PerceptionManager
from opencda.core.sensing.localization.rsu_localization_manager import \
    LocalizationManager
from opencda.core.sensing.tracking.tracking_manager \
    import TrackingManager


class RSUManager(object):
    """
    A class manager for RSU. Currently a RSU only has perception and
    localization modules to dump sensing information.
    TODO: add V2X module to it to enable sharing sensing information online.

    Parameters
    ----------
    carla_world : carla.World
        CARLA simulation world, we need this for blueprint creation.

    config_yaml : dict
        The configuration dictionary of the RSU.

    carla_map : carla.Map
        The CARLA simulation map.

    cav_world : opencda object
        CAV World for simulation V2X communication.

    current_time : str
        Timestamp of the simulation beginning, this is used for data dump.

    data_dumping : bool
        Indicates whether to dump sensor data during simulation.

    Attributes
    ----------
    localizer : opencda object
        The current localization manager.

    perception_manager : opencda object
        The current V2X perception manager.

    data_dumper : opencda object
        Used for dumping sensor data.
    """
    def __init__(
            self,
            carla_world,
            config_yaml,
            carla_map,
            cav_world,
            current_time='',
            data_dumping=False):

        self.rid = config_yaml['id']
        # The id of rsu is always a negative int
        if self.rid > 0:
            self.rid = -self.rid

        # read map from the world everytime is time-consuming, so we need
        # explicitly extract here
        self.carla_map = carla_map

        # retrieve the configure for different modules
        # todo: add v2x module to rsu later
        sensing_config = config_yaml['sensing']
        sensing_config['localization']['global_position'] = \
            config_yaml['spawn_position']
        sensing_config['perception']['global_position'] = \
            config_yaml['spawn_position']

        # localization module
        self.localizer = LocalizationManager(carla_world,
                                             sensing_config['localization'],
                                             self.carla_map)
        # tracking module

        self.tracking_manager = TrackingManager(None, cav_world, data_dumping, carla_world=carla_world, infra_id=self.rid, tracker_type = "SORT")
        # perception module
        self.perception_manager = PerceptionManager(vehicle=None,
                                                    config_yaml=sensing_config['perception'],
                                                    cav_world=cav_world,
                                                    carla_world = carla_world,
                                                    data_dump=data_dumping,
                                                    infra_id=self.rid,
                                                    tracking_manager=self.tracking_manager)
        self.objects = {}

        if data_dumping:
            self.data_dumper = DataDumper(self.perception_manager,
                                          self.rid,
                                          save_time=current_time)
        else:
            self.data_dumper = None

        cav_world.update_rsu_manager(self)

    def update_info(self):
        """
        Call perception and localization module to
        retrieve surrounding info an ego position.
        """
        # localization
        self.objects.clear()
        self.localizer.localize()

        ego_pos = self.localizer.get_ego_pos()
        ego_spd = self.localizer.get_ego_spd()

        # object detection todo: pass it to other CAVs for V2X percetion
        self.objects = self.perception_manager.detect(ego_pos)
        def recursive_print_object(obj, indent=0, visited=None):
            try:
                pickle.dumps(obj)  # Test if the object is picklable
            except Exception as e:
                print(f"{'  ' * indent}<Unpicklable {type(obj).__name__} object at {hex(id(obj))}>", flush=True)
                            
            if visited is None:
                visited = set()

            # Prevent infinite recursion for circular references
            if id(obj) in visited:
                print(f"{'  ' * indent}<Circular Reference to {type(obj).__name__} object at {hex(id(obj))}>", flush=True)
                return
            visited.add(id(obj))

            print(f"{'  ' * indent}{type(obj).__name__} object at {hex(id(obj))}:", flush=True)
            indent += 1

            if isinstance(obj, dict):
                for key, value in obj.items():
                    print(f"{'  ' * indent}{key}:", flush=True)
                    recursive_print_object(value, indent + 1, visited)
            elif isinstance(obj, list):
                for i, item in enumerate(obj):
                    print(f"{'  ' * indent}[{i}]:", flush=True)
                    recursive_print_object(item, indent + 1, visited)
            elif hasattr(obj, '__dict__'):
                for attr_name, attr_value in obj.__dict__.items():
                    if hasattr(attr_value, '__dict__'):  # Check if the attribute is another object
                        print(f"{'  ' * indent}{attr_name}:", flush=True)
                        recursive_print_object(attr_value, indent + 1, visited)
                    else:
                        print(f"{'  ' * indent}{attr_name}: {attr_value}", flush=True)
            else:
                print(f"{'  ' * indent}{obj}", flush=True)

        recursive_print_object(self.objects)

    def run_step(self):
        """
        Currently only used for dumping data.
        """
        return
        # dump data
        if self.data_dumper:
            self.data_dumper.run_step(self.perception_manager,
                                      self.localizer,
                                      None)

    def destroy(self):
        """
        Destroy the actor vehicle
        """
        self.perception_manager.destroy()
        self.localizer.destroy()
