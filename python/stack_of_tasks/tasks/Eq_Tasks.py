import numpy as np
import traits.api as ta

from tf.transformations import rotation_from_matrix

from stack_of_tasks.robot_model.robot_state import RobotState
from stack_of_tasks.tasks.Task import A, Bound, EqTask, RelativeTask, TargetTask, Tuple
from stack_of_tasks.utils.transform_math import skew


class PositionTask(RelativeTask, EqTask):
    name = "Position"
    task_size: int = 3

    def compute(self) -> Tuple[A, Bound]:
        return self._J[:3], self.refB.T[:3, 3] - self.refA.T[:3, 3]


class OrientationTask(RelativeTask, EqTask):
    name = "Orientation"
    task_size: int = 3

    def compute(self) -> Tuple[A, Bound]:
        tA = self.refA.T
        delta = np.identity(4)
        delta[0:3, 0:3] = tA[0:3, 0:3].T.dot(self.refB.T[0:3, 0:3])
        angle, axis, _ = rotation_from_matrix(delta)
        return self._J[3:], tA[0:3, 0:3].dot(angle * axis)


class DistanceTask(RelativeTask, EqTask):
    """Keep distance to target position, not considering (approach) direction"""

    name = "Distance"
    task_size: int = 1

    distance = ta.Range(0.0, value=1.0, exclude_low=True)

    def compute(self) -> Tuple[A, Bound]:
        delta = self.refA.T[0:3, 3] - self.refB.T[0:3, 3]

        return np.array([delta.dot(self._J[:3])]), self.distance - np.linalg.norm(delta)


class PlaneTask(TargetTask, EqTask):
    name = "Plane"
    task_size: int = 1

    def compute(self) -> Tuple[A, Bound]:
        """Move eef within plane given by normal vector and distance to origin"""
        Tc = self.robot.T
        Tt = self.target.T

        normal = Tt[0:3, 2]
        dist = normal.dot(Tt[0:3, 3])

        return np.array([normal.T.dot(self._J[:3])]), np.array(
            [-(normal.dot(Tc[0:3, 3]) - dist)]
        )


RobotAxis = ta.Array(shape=(3,), value=np.array([0, 0, 1]))


class ParallelTask(RelativeTask, EqTask):
    name = "Parallel"
    task_size: int = 3

    robot_axis = RobotAxis()
    target_axis = RobotAxis()

    def compute(self) -> Tuple[A, Bound]:
        """Align axis in eef frame to be parallel to reference axis in base frame"""

        # transform axis from eef frame to base frame
        axis = self.refA.T[0:3, 0:3].dot(self.robot_axis)
        ref = self.refB.T[0:3, 0:3].dot(self.target_axis)
        return (skew(ref).dot(skew(axis))).dot(self._J[3:]), np.cross(ref, axis)


class JointTask(EqTask):
    name = "Joint Position"

    _robot_state: RobotState = ta.Instance(RobotState)
    mask = ta.Array()
    target = ta.Array()

    def __init__(self, robot_state: RobotState, joints=None, **traits) -> None:
        super().__init__(_robot_state=robot_state, **traits)
        m = self._robot_state.robot_model
        self.task_size = m.N
        if joints is None:
            self.mask = np.ones(self.task_size)
            self.target = self._robot_state.joint_values
        else:
            self.mask = [1.0 if j.name in joints else 0.0 for j in m.active_joints]
            self.target = [joints[j.name] for j in m.active_joints]

    def compute(self) -> Tuple[A, Bound]:
        return (
            np.diag(self.mask),
            np.multiply(self.mask, self._robot_state.joint_values - self.target),
        )


#
#
# class LineTask(EqTask):
#    name = "Line"
#    task_size: int = 3
#
#    def _compute(self, line_point: OffsetTransform):
#        Tc, Jc = self.fk(self.frame)
#        Tt, Jt = self.fk(line_point)
#
#        normal = line_point[0:3, 2]
#        sw = skew(normal)
#
#        d = Tc[0:3, 3] - line_point[0:3, 3]
#
#        self.A = sw.dot(Jc[:3] - Jt[:3])
#        self.bound = -sw.dot(d)
#
#
