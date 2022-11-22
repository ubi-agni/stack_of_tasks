from abc import ABC, abstractmethod

import numpy as np
from numpy.typing import NDArray

from stack_of_tasks.robot_model import RobotState

from . import HasJacobian, HasTransform, Jacobian, Transform


class RefFrame(HasTransform, ABC):
    @property
    @abstractmethod
    def T(self) -> NDArray:
        ...


class World(RefFrame):
    __instance: "World" = None
    __origin: Transform = np.identity(4)

    def __new__(cls):
        if cls.__instance is None:
            cls.__instance = super().__new__(cls)
            cls.__origin.setflags(write=False)

        return cls.__instance

    @property
    def T(self) -> Transform:
        return self.__origin


class JointFrame(RefFrame, HasJacobian):
    def __init__(self, robot_state: RobotState, joint: str):
        self.robot_state = robot_state
        self.joint = joint

    @property
    def J(self) -> Jacobian:
        _, J = self.robot_state.fk(self.joint)
        return J

    @property
    def T(self) -> Transform:
        T, _ = self.robot_state.fk(self.joint)
        return T
