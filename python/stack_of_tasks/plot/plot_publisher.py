from random import getrandbits
from time import time

from typing import List

import traits.api as ta
from plotjuggler_msgs.msg import StatisticsNames, StatisticsValues

from rospy import Duration, Publisher
from rospy.timer import Timer, TimerEvent
from tf.transformations import euler_from_matrix

from stack_of_tasks.controller import Controller
from stack_of_tasks.ref_frame import RefFrame
from stack_of_tasks.tasks.hierarchy import TaskHierarchy


class PlotPublisher(ta.HasTraits):
    frames = ta.List(RefFrame)

    def __init__(self, controller: Controller, prefix="stack_of_tasks", **traits) -> None:
        super().__init__(**traits)

        self._prefix = prefix
        self._controller = controller

        self.name_publisher = Publisher(
            f"{self._prefix}/names", StatisticsNames, latch=True, queue_size=1
        )

        self.data_publisher = Publisher(f"{self._prefix}", StatisticsValues, latch=True, queue_size=10)

        self._version = 0
        self._send_names()

        self._controller.observe(self._send_data, "_updated")
        observing = "levels, levels:items, levels:items:items"
        self._controller.task_hierarchy.observe(self._send_names, observing)

    def _names(self):
        names = []
        joint_names = {j.name for j in self._controller.robot_model.active_joints}
        names.extend([f"q/{name}" for name in joint_names])
        names.extend([f"dq/{name}" for name in joint_names])

        frame_variables = ["x", "y", "z", "rx", "ry", "rz"]
        for frame in self.frames:
            names.extend([f"frame/{frame.name}/{name}" for name in frame_variables])

        for i, level in enumerate(self._controller.task_hierarchy.levels):
            prefix = f"level {i}"
            for task in level:
                name = f"{prefix}/{task.name}"
                names.extend([f"{name}/residual[{i}]" for i in range(task.task_size)])

        return names

    def _values(self):
        values = []
        values.extend(self._controller.robot_state.joint_values)
        values.extend(self._controller.dq)

        for frame in self.frames:
            pos = list(frame.T[0:3, 3])
            rpy = list(euler_from_matrix(frame.T[0:3, 0:3]))
            values.extend(pos + rpy)

        for level in self._controller.task_hierarchy.levels:
            for task in level:
                values.extend(task.residual)

        return values

    @ta.observe("frames, frames:items")
    def _send_names(self, _=None):
        self._version += 1
        msg = StatisticsNames()
        msg.names = self._names()
        msg.names_version = self._version

        self.name_publisher.publish(msg)

    def _send_data(self, _):
        msg = StatisticsValues()
        msg.values = self._values()
        msg.names_version = self._version

        self.data_publisher.publish(msg)
