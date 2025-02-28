

import logging
from datetime import datetime
import os
from opencda.scenario_testing.utils.yaml_utils import load_yaml

if not os.path.exists('log'):
   os.makedirs('log')

LOG_FILENAME = datetime.now().strftime('log/logfile_%Y_%m_%d_%H_%M_%S.log')
WAYPOINTS_FILENAME = datetime.now().strftime('log/waypoints_%Y_%m_%d_%H_%M_%S.log')

logger = logging.getLogger()
logger.setLevel(logging.DEBUG)

cloud_config = load_yaml("cloud_config.yaml")

if cloud_config["log_level"] == "error":
    logger.setLevel(logging.ERROR)
elif cloud_config["log_level"] == "warning":
    logger.setLevel(logging.WARNING)
elif cloud_config["log_level"] == "info":
    logger.setLevel(logging.INFO)

# for handler in logging.root.handlers[:]:
#     logging.root.removeHandler(handler)

# logging.basicConfig(filename=LOG_FILENAME,level=logging.DEBUG)

fh = logging.FileHandler(LOG_FILENAME)
fh.setLevel(logging.DEBUG)

# create console handler with a higher log level
ch = logging.FileHandler(WAYPOINTS_FILENAME)
ch.setLevel(logging.WARNING)

# create formatter and add it to the handlers
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')

ch.setFormatter(formatter)
fh.setFormatter(formatter)

# add the handlers to logger
logger.addHandler(ch)
logger.addHandler(fh)

