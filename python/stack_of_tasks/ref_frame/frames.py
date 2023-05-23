from __future__ import annotations

import numpy as np
import traits.api as ta

from stack_of_tasks.ref_frame import RefFrame, Transform
from stack_of_tasks.robot_model.robot_state import RobotState


class Origin(RefFrame):
    """Defines the origin. T is the identity and can not be changed."""

    T: Transform = ta.ReadOnly(np.identity(4))

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.T.flags.writeable = False


class RobotRefFrame(RefFrame):
    """A reference to a robot frame. Contains current transform (T) and Jacobian (J) of the given robot link. All attributes are read only and will be calculated dynamically.

    Attributes:
        robot_state (RobotState): A read only reference to the frames robot state.
        link_name (str): The frames robot link name. Read only.
        T (Transform): The current robot link transform.
        J (Jacobian): The current robot link jacobian.
    """

    robot_state: RobotState = ta.Instance(RobotState)
    link_name: str = ta.Str()

    T = ta.Property(observe="_fk")
    J = ta.Property(observe="_fk")

    _fk = ta.Property(observe="robot_state.joint_values")

    def __init__(self, robot_state: RobotState, link_name: str, **kwargs) -> None:
        super().__init__(robot_state=robot_state, link_name=link_name, **kwargs)

    @ta.cached_property
    def _get__fk(self):
        return self.robot_state.fk(self.link_name)

    def _get_T(self):
        return self._fk[0]

    def _get_J(self):
        return self._fk[1]
