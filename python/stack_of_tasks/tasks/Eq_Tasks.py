import numpy as np

from tf.transformations import rotation_from_matrix

from stack_of_tasks.tasks.Task import EqTask, TaskSoftnessType
from stack_of_tasks.utils import OffsetTransform, skew


class PositionTask(EqTask):
    name = "Position"
    task_size: int = 3

    def _compute(self, target: OffsetTransform):
        Tc, Jc = self.fk(self.frame)
        Tt, Jt = self.fk(target)

        self.A = Jc[:3] - Jt[:3]
        self.bound = Tt[:3, 3] - Tc[:3, 3]


class OrientationTask(EqTask):
    name = "Orientation"
    task_size: int = 3

    def _compute(self, target: OffsetTransform):
        Tc, Jc = self.fk(self.frame)
        Tt, Jt = self.fk(target)

        self.A = Jc[3:] - Jt[3:]
        delta = np.identity(4)
        delta[0:3, 0:3] = Tc[0:3, 0:3].T.dot(Tt[0:3, 0:3])
        angle, axis, _ = rotation_from_matrix(delta)

        self.bound = Tc[0:3, 0:3].dot(angle * axis)


class DistanceTask(EqTask):
    """Keep distance to target position, not considering (approach) direction"""

    name = "Distance"
    task_size: int = 1

    def _compute(self, target: OffsetTransform, distance: float):
        Tc, Jc = self.fk(self.frame)
        Tt, Jt = self.fk(target)

        delta = Tc[0:3, 3] - Tt[0:3, 3]
        self.A = np.array([delta.T.dot(Jc[:3] - Jt[:3])])
        self.bound = np.array([-(np.linalg.norm(delta) - distance)])


class PlaneTask(EqTask):
    name = "Plane"
    task_size: int = 1

    def _compute(self, target: OffsetTransform):
        """Move eef within plane given by normal vector and distance to origin"""
        Tc, Jc = self.fk(self.frame)
        Tt, Jt = self.fk(target)

        normal = Tt[0:3, 2]
        dist = normal.dot(Tt[0:3, 3])

        self.A = np.array([normal.T.dot(Jc[:3] - Jt[:3])])
        self.bound = np.array([-(normal.dot(Tc[0:3, 3]) - dist)])


class ParallelTask(EqTask):
    name = "Parallel"
    task_size: int = 3

    def _compute(self, target: OffsetTransform, robot_axis, target_axis):
        """Align axis in eef frame to be parallel to reference axis in base frame"""
        Tc, Jc = self.fk(self.frame)
        Tt, Jt = self.fk(target)

        # transform axis from eef frame to base frame
        axis = Tc[0:3, 0:3].dot(robot_axis)
        ref = Tt[0:3, 0:3].dot(target_axis)
        self.A = (skew(ref).dot(skew(axis))).dot(Jc[3:] - Jt[3:])
        self.bound = np.cross(ref, axis)


class LineTask(EqTask):
    name = "Line"
    task_size: int = 3

    def _compute(self, line_point: OffsetTransform):
        Tc, Jc = self.fk(self.frame)
        Tt, Jt = self.fk(line_point)

        normal = line_point[0:3, 2]
        sw = skew(normal)

        d = Tc[0:3, 3] - line_point[0:3, 3]

        self.A = sw.dot(Jc[:3] - Jt[:3])
        self.bound = -sw.dot(d)


class JointPos(EqTask):
    name = "Joint Position"

    def __init__(
        self, weight: float, softnessType: TaskSoftnessType, number_of_joints: int
    ) -> None:
        self.task_size = number_of_joints
        super().__init__(weight, softnessType)

    def _compute(self, desired_joint_pose):
        self.A = np.identity(desired_joint_pose.size)
        self.bound = self.controller._joint_position - desired_joint_pose


if __name__ == "__main__":
    p = PositionTask(1, TaskSoftnessType.hard)
    print(p.argmap.keys())
