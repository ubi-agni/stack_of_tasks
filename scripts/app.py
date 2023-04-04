from __future__ import annotations

from typing import Callable

from stack_of_tasks.controller import Controller
from stack_of_tasks.marker.marker_server import MarkerServer
from stack_of_tasks.robot_model.jointstate_publisher import JointStatePublisher

SetupSignature = Callable[["Application"], None]


class Application:
    def __init__(
        self, setup: SetupSignature, solverClass, publish_joints=True, **args
    ) -> None:
        self.marker_server = MarkerServer()
        self.controller = Controller(solverClass, **args)
        self.task_hierarchy = self.controller.hierarchy
        if publish_joints:
            self._jcp = JointStatePublisher(self.controller.robot_state)

        setup(self)
