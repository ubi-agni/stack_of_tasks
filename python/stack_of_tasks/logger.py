import logging
from logging import LogRecord


class RosFilter(logging.Filter):
    def filter(self, record: LogRecord) -> bool:
        print(vars(record))
        return True


def fix_rospy_logging(logger: logging.Logger):
    console = logging.StreamHandler()
    formatter = logging.Formatter("%(levelname)s:%(name)s:%(message)s")
    console.setFormatter(formatter)
    logger.root.addHandler(console)
    logger.root.setLevel(logging.WARNING)


sot_logger = logging.getLogger("sot")
