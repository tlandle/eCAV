import logging
import sys
from datetime import datetime
import os
from opencda.scenario_testing.utils.yaml_utils import load_yaml

if not os.path.exists('log'):
    os.makedirs('log')

LOG_FILENAME = datetime.now().strftime('log/logfile_%Y_%m_%d_%H_%M_%S.log')
WAYPOINTS_FILENAME = datetime.now().strftime('log/waypoints_%Y_%m_%d_%H_%M_%S.log')

cloud_config = load_yaml("cloud_config.yaml")


def _resolve_level(name, default=logging.INFO):
    return getattr(logging, str(name).upper(), default)


_console_level = _resolve_level(cloud_config.get("log_level", "info"))
_file_level = _resolve_level(cloud_config.get("file_log_level", "debug"), logging.DEBUG)

# ANSI color codes for per-module coloring on console
_RESET = "\033[0m"
_BOLD = "\033[1m"
_MODULE_COLORS = {
    "behavior_agent":           "\033[96m",   # cyan
    "collision_check":          "\033[93m",   # yellow
    "perception_manager":       "\033[95m",   # magenta
    "o3d_lidar_libs":           "\033[35m",   # dark magenta
    "vehicle_manager":          "\033[94m",   # blue
    "EdgeManager":              "\033[92m",   # green
    "sim_api":                  "\033[97m",   # white
    "sort_tracker":             "\033[33m",   # dark yellow
    "networking":               "\033[36m",   # dark cyan
    "ecloud_comms":             "\033[34m",   # dark blue
    "ecloud_actor_client":      "\033[91m",   # red
    "distributed_actor_client": "\033[31m",   # dark red
    "edge_process":             "\033[32m",   # dark green
    "astar_edge_manager":       "\033[90m",   # gray
}

_LEVEL_COLORS = {
    "DEBUG":    "\033[37m",    # light gray
    "INFO":     "\033[97m",    # white
    "WARNING":  "\033[93m",    # yellow
    "ERROR":    "\033[91m",    # red
    "CRITICAL": "\033[1;91m",  # bold red
}


class _ColoredConsoleFormatter(logging.Formatter):
    """Formatter that colors the module name and level for console output."""

    def format(self, record):
        # Find module color by matching the last component of the logger name
        module_short = record.name.rsplit(".", 1)[-1]
        mod_color = _MODULE_COLORS.get(module_short, "\033[37m")
        lvl_color = _LEVEL_COLORS.get(record.levelname, "")

        colored_name = f"{mod_color}{record.name}{_RESET}"
        colored_level = f"{lvl_color}{record.levelname}{_RESET}"

        msg = (f"{self.formatTime(record)} - {colored_name} - "
               f"{colored_level} - {record.getMessage()}")
        if record.exc_info and not record.exc_text:
            record.exc_text = self.formatException(record.exc_info)
        if record.exc_text:
            msg += "\n" + record.exc_text
        return msg


# Root logger at DEBUG so nothing is filtered before reaching handlers
root = logging.getLogger()
root.setLevel(logging.DEBUG)

plain_formatter = logging.Formatter(
    '%(asctime)s - %(name)s - %(levelname)s - %(message)s')

# File handler — captures everything at file_log_level to timestamped log
fh = logging.FileHandler(LOG_FILENAME)
fh.setLevel(_file_level)
fh.setFormatter(plain_formatter)
root.addHandler(fh)

# Waypoints file handler — WARNING+ only (legacy)
wh = logging.FileHandler(WAYPOINTS_FILENAME)
wh.setLevel(logging.WARNING)
wh.setFormatter(plain_formatter)
root.addHandler(wh)

# Console handler — colored output, level from cloud_config.yaml
ch = logging.StreamHandler(sys.stdout)
ch.setLevel(_console_level)
ch.setFormatter(_ColoredConsoleFormatter())
root.addHandler(ch)

# Short name → full dotted logger name mapping
# Config uses short names for readability; Python logging needs full paths
_SHORT_TO_FULL = {
    "behavior_agent":           "opencda.core.plan.behavior_agent",
    "collision_check":          "opencda.core.plan.collision_check",
    "perception_manager":       "opencda.core.sensing.perception.perception_manager",
    "o3d_lidar_libs":           "opencda.core.sensing.perception.o3d_lidar_libs",
    "sort_tracker":             "opencda.core.sensing.tracking.sort_tracker",
    "vehicle_manager":          "opencda.core.common.vehicle_manager",
    "sim_api":                  "opencda.scenario_testing.utils.sim_api",
    "EdgeManager":              "EdgeManager",
    "networking":               "opencda.core.application.edge.networking",
    "astar_edge_manager":       "opencda.core.application.edge.a_star_algorithm.astar_edge_manager",
    "ecloud_comms":             "opencda.ecloud_server.ecloud_comms",
    "ecloud_actor_client":      "opencda.ecav2.ecloud_actor_client",
    "distributed_actor_client": "opencda.distributed_client.distributed_actor_client",
    "edge_process":             "opencda.ecav2.edge_process",
}

# Per-module log level overrides from cloud_config.yaml
_module_levels = cloud_config.get("module_log_levels", {})
for short_name, level_str in _module_levels.items():
    full_name = _SHORT_TO_FULL.get(short_name, short_name)
    mod_logger = logging.getLogger(full_name)
    mod_logger.setLevel(_resolve_level(level_str))
