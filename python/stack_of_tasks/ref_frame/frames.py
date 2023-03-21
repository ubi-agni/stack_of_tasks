from __future__ import annotations

import numpy as np
import traits.api as ta

from stack_of_tasks.robot_model.robot_state import RobotState

from . import Transform


class RefFrame(ta.ABCHasTraits):
    T: Transform = ta.Any


class Origin(RefFrame):
    T: Transform = ta.ReadOnly(np.identity(4))

    def __init__(self) -> None:
        super().__init__()
        self.T.flags.writeable = False


class RobotRefFrame(RefFrame):
    robot_state: RobotState = ta.Instance(RobotState)
    link: str = ta.ReadOnly()

    T = ta.Property(observe="_fk")
    J = ta.Property(observe="_fk")

    _fk = ta.Property(observe="robot_state.joint_values.items")

    def __init__(self, state: RobotState, link: str) -> None:
        super().__init__(robot_state=state, link=link)

    @ta.cached_property
    def _get__fk(self):
        return self.robot_state.fk(self.link)

    def _get_T(self):
        return self._fk[0]

    def _get_J(self):
        return self._fk[1]
