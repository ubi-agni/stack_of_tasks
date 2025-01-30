import numpy as np
import traits.api as ta

from tf.transformations import rotation_from_matrix

from stack_of_tasks.ref_frame import Jacobian
from stack_of_tasks.ref_frame.frames import RefFrame
from stack_of_tasks.tasks.base import A, Bound, EqTask, RelativeTask, TargetTask, TaskSoftnessType, Tuple
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


class RotationTask(EqTask):
    name = "Rotation"
    task_size: int = 3

    robot: RefFrame = ta.Instance(RefFrame)
    center: RefFrame = ta.Instance(RefFrame)
    axis = ta.Array(shape=(3,), dtype="float", value=np.array([0, 0, 1]))
    omega = ta.Float(0.01)

    _J: Jacobian = ta.DelegatesTo("robot", "J")

    def __init__(
        self,
        center: RefFrame,
        robot: RefFrame,
        softness_type: TaskSoftnessType,
        weight: float = 1,
        **traits,
    ) -> None:
        super().__init__(softness_type, weight, robot=robot, center=center, **traits)
        self.observe(self._trigger_recompute, "robot:T, _J, axis, omega")

    def compute(self) -> Tuple[A, Bound]:
        r = self.robot.T[0:3, 3] - self.center.T[0:3, 3]
        w = self.omega * self.center.T[:3, :3].dot(self.axis)
        return self._J[:3], np.cross(w, r)


class DistanceTask(RelativeTask, EqTask):
    """Keep distance to target position, not considering (approach) direction"""

    name = "Distance"
    task_size: int = 1

    distance = ta.Range(0.0, value=1.0, exclude_low=True)

    def compute(self) -> Tuple[A, Bound]:
        delta = self.refA.T[0:3, 3] - self.refB.T[0:3, 3]

        return delta.dot(self._J[:3]), 0.1 * (self.distance - np.linalg.norm(delta))


class PlaneTask(TargetTask, EqTask):
    name = "Plane"
    task_size: int = 1

    def compute(self) -> Tuple[A, Bound]:
        """Move eef within plane given by normal vector and distance to origin"""
        Tc = self.robot.T
        Tt = self.target.T

        normal = Tt[0:3, 2]
        dist = normal.dot(Tt[0:3, 3])
        current = normal.dot(Tc[0:3, 3])

        return normal.T.dot(self._J[:3]), dist - current


Axis = ta.Array(shape=(3,), value=np.array([0, 0, 1]))


class ParallelTask(RelativeTask, EqTask):
    name = "Parallel"
    task_size: int = 3

    refA_axis = Axis()
    refB_axis = Axis()

    def compute(self) -> Tuple[A, Bound]:
        """Align axis in eef frame to be parallel to reference axis in base frame"""

        # transform axis from eef frame to base frame
        axis = self.refA.T[0:3, 0:3].dot(self.refA_axis)
        ref = self.refB.T[0:3, 0:3].dot(self.refB_axis)
        return (skew(ref).dot(skew(axis))).dot(self._J[3:]), np.cross(ref, axis)


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
# class JointPos(JointTask, EqTask):
#    name = "Joint Position"
#
#    def _compute(self, current_joint_pose, desired_joint_pose):
#        self.bound = (current_joint_pose - desired_joint_pose) * self.weight
