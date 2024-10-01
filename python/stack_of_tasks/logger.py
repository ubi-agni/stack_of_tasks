import logging
from logging import LogRecord

import colorlog

formatter = colorlog.ColoredFormatter(
    "%(log_color)s%(levelname)-8s%(reset)s %(blue)s%(message)s",
    datefmt=None,
    reset=True,
    log_colors={
        "DEBUG": "cyan",
        "INFO": "green",
        "WARNING": "yellow",
        "ERROR": "red",
        "CRITICAL": "red,bg_white",
    },
    secondary_log_colors={},
    style="%",
)


class RosFilter(logging.Filter):
    def filter(self, record: LogRecord) -> bool:
        return True


def fix_rospy_logging(logger: logging.Logger):

    console = logging.StreamHandler()
    # formatter = logging.Formatter("%(levelname)s:%(name)s:%(message)s")
    console.setFormatter(formatter)
    logger.root.addHandler(console)
    logger.root.setLevel(logging.WARNING)


sot_logger = logging.getLogger("sot")
logging.root = sot_logger
