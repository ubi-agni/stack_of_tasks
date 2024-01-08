from __future__ import annotations

from typing import Callable

from stack_of_tasks.config import Parameter
from stack_of_tasks.controller import Controller
from stack_of_tasks.marker.marker_server import MarkerServer
from stack_of_tasks.robot_model.actuators import JointStatePublisherActuator

SetupSignature = Callable[["Application"], None]


class Application:
    def __init__(self, config: Parameter, setup: SetupSignature) -> None:
        self.marker_server = MarkerServer()
        self.controller = Controller(config)

        setup(self)

    @property
    def task_hierarchy(self):
        return self.controller.task_hierarchy
