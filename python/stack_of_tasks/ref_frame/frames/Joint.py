from stack_of_tasks.robot_model import RobotState

from stack_of_tasks.ref_frame import HasJacobian, Jacobian, Transform
from stack_of_tasks.ref_frame.offset import OffsetWithJacobian
from stack_of_tasks.ref_frame.frames import RefFrame


class JointFrame(RefFrame, HasJacobian):
    def __init__(self, robot_state: RobotState, joint: str):
        super().__init__()
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

    def transform(self, matrix: Transform) -> OffsetWithJacobian:
        return OffsetWithJacobian(self).transform(matrix)
