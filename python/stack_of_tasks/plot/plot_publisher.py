from random import getrandbits
from time import time

from typing import List

from plotjuggler_msgs.msg import StatisticsNames, StatisticsValues

from rospy import Duration, Publisher
from rospy.timer import Timer, TimerEvent

from stack_of_tasks.tasks.hierarchy import TaskHierarchy


class PlotPublisher:
    def __init__(self, sot: TaskHierarchy) -> None:
        self._prefix = "stack_of_tasks"
        self._stack_of_tasks = sot

        self.name_publisher = Publisher(
            f"{self._prefix}/names", StatisticsNames, latch=True, queue_size=1
        )

        self.data_publisher = Publisher(
            f"{self._prefix}", StatisticsValues, latch=True, queue_size=10
        )

        self._send_names()
        self._stack_of_tasks.observe(self._resend, "levels:items:items:residual")

    def _resend(self, _):
        self._send_data()

    def _names(self):
        names = []
        for i, level in enumerate(self._stack_of_tasks.levels):
            prefx = f"level {i}"
            for task in level:
                name = f"{prefx}/{task.name}"
                names.extend([f"{name}/residual[{i}]" for i in range(task.task_size)])

        return names

    def _send_names(self):
        mssg = StatisticsNames()
        mssg.names = self._names()
        mssg.names_version = 0

        self.name_publisher.publish(mssg)

    def _send_data(self):
        mssg = StatisticsValues()
        mssg.values = self._values()
        mssg.names_version = 0

        self.data_publisher.publish(mssg)

    def _values(self):
        values = []
        for level in self._stack_of_tasks.levels:
            for task in level:
                values.extend(task.residual)

        return values
