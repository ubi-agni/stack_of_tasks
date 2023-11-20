from __future__ import annotations

from typing import Callable

from stack_of_tasks.controller import Controller
from stack_of_tasks.marker.marker_server import MarkerServer
from stack_of_tasks.robot_model.actuators import JointStatePublisherActuator

SetupSignature = Callable[["Application"], None]


class Application:
    def __init__(
        self, setup: SetupSignature, solverClass, publish_joints=True, **args
    ) -> None:
        self.marker_server = MarkerServer()
        self.controller = c = Controller(solverClass, **args)
        self.task_hierarchy = c.task_hierarchy
        if publish_joints:
            c.actuator = JointStatePublisherActuator(c.robot_state)

        setup(self)
