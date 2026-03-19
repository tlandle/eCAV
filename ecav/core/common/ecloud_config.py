from enum import Enum
import logging

class eLocationType(Enum):
    RANDOM = 0
    EXPLICIT = 1
    COUNT = 2

class eDoneBehavior(Enum):
    DESTROY = 0
    CONTROL = 1
    COUNT = 2

class EcloudConfig(object):

    RANDOM = "random"
    EXPLICIT = "explicit"
    DESTROY = "destroy"
    CONTROL = "control"

    location_types = { RANDOM : eLocationType.RANDOM,
                       EXPLICIT : eLocationType.EXPLICIT }

    done_behavior_types = { DESTROY : eDoneBehavior.DESTROY,
                            CONTROL : eDoneBehavior.CONTROL }


    def __init__(self, config_json, logger=None):

        # logger.debug("main - test_scenario: %s", config_json)

        self.config_json = config_json
        self.logger = logger if logger is not None else logging.getLogger(__name__)
        self.ecloud_base = {
            "num_servers" : 2, # % num_cars to choose which port to connect to. 2nd - nth server port: p = 50053 + ( n - 1 ) - really # of server THREADS
            "num_ports" : 32,
            "server_ping_time_s" : 0.005, # 5ms
            "client_world_time_factor" : 0.9, # what percentage of last world time to wait initially
            "client_ping_spawn_s" : 0.05, # sleep to wait between pings after spawn
            "client_ping_tick_s" : 0.01, # minimum sleep to wait between pings after spawn
        }

        self.ecloud_scenario = {
           "num_cars" : 0,
           "location_type" : self.EXPLICIT,
           "done_behavior" : self.DESTROY,
           "step_count" : 250, # number of steps to take before breaking
           "verbose_updates" : False, # all vehicles send data every tick, used for edge *comparison* testing
        }

        if 'ecloud' in config_json:
            self.logger.info("'ecloud' found in config_base")
            for k in self.ecloud_base.keys():
                logger.info("looking for key %s in config_base", k)
                if k in config_json['ecloud']:
                    logger.debug("overriding base_config %s with %s", k, config_json['ecloud'][k])
                    self.ecloud_base[k] = config_json['ecloud'][k]
        else:
            self.logger.info("'ecloud' not found in config_base")

        if 'ecloud' in config_json['scenario']:
            self.logger.info("'ecloud' found in config_scenario")
            for k in self.ecloud_scenario.keys():
                logger.info("looking for key %s in config_scenario", k)
                if k in config_json['scenario']['ecloud']:
                    logger.debug("overriding scenario_config %s with %s", k, config_json['scenario']['ecloud'][k])
                    self.ecloud_scenario[k] = config_json['scenario']['ecloud'][k]
        else:
            self.logger.info("'ecloud' not found in config_scenario")

        return

    def get_num_servers(self):
        self.logger.debug("num_servers: %s", self.ecloud_base['num_servers'])
        return self.ecloud_base['num_servers']

    def get_num_ports(self):
        self.logger.debug("num_ports: %s", self.ecloud_base['num_ports'])
        return self.ecloud_base['num_ports']

    def get_server_ping_time_s(self):
        self.logger.debug("server_ping_time_s: %s", self.ecloud_base['server_ping_time_s'])
        return self.ecloud_base['server_ping_time_s']

    def get_client_tick_ping_time_s(self):
        self.logger.debug("client_ping_tick_s: %s", self.ecloud_base['client_ping_tick_s'])
        return self.ecloud_base['client_ping_tick_s']

    def get_client_spawn_ping_time_s(self):
        self.logger.debug("client_ping_spawn_s: %s", self.ecloud_base['client_ping_spawn_s'])
        return self.ecloud_base['client_ping_spawn_s']

    def get_client_world_tick_factor(self):
        self.logger.debug("client_world_time_factor: %s", self.ecloud_base['client_world_time_factor'])
        return self.ecloud_base['client_world_time_factor']

    def get_num_cars(self):
        self.logger.debug("num_cars: %s", self.ecloud_scenario['num_cars'] if self.ecloud_scenario['num_cars'] != 0 else len(self.config_json['scenario']['single_cav_list']))
        return self.ecloud_scenario['num_cars'] if self.ecloud_scenario['num_cars'] != 0 else \
                len(self.config_json['scenario']['single_cav_list'])

    def get_location_type(self):
        self.logger.debug("location_type: %s", self.ecloud_scenario['location_type'])
        return EcloudConfig.location_types[self.ecloud_scenario['location_type']]

    def get_done_behavior(self):
        self.logger.debug("done_behavior: %s", self.ecloud_scenario['done_behavior'])
        return EcloudConfig.done_behavior_types[self.ecloud_scenario['done_behavior']]

    def get_step_count(self):
        self.logger.debug("step_count: %s", self.ecloud_scenario['step_count'])
        return self.ecloud_scenario['step_count']
    
    def do_verbose_update(self):
        self.logger.debug("verbose_updates: %s", self.ecloud_scenario['verbose_updates'])
        return self.ecloud_scenario['verbose_updates']
