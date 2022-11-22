import numpy as np

from tf.transformations import rotation_from_matrix

from stack_of_tasks.tasks.Task import EqTask, RelativeTask
from stack_of_tasks.utils.transform_math import skew


class PositionTask(RelativeTask, EqTask):
    name = "Position"
    task_size: int = 3

    def _compute(self):
        self.A = self.J()[:3]
        self.bound = self.frameB.T[:3, 3] - self.frameA.T[:3, 3]


#
class OrientationTask(RelativeTask, EqTask):

    name = "Orientation"
    task_size: int = 3

    def _compute(self):

        self.A = self.J()[3:]
        tA = self.frameA.T
        delta = np.identity(4)
        delta[0:3, 0:3] = tA[0:3, 0:3].T.dot(self.frameB.T[0:3, 0:3])
        angle, axis, _ = rotation_from_matrix(delta)
        self.bound = tA[0:3, 0:3].dot(angle * axis)


class DistanceTask(RelativeTask, EqTask):
    """Keep distance to target position, not considering (approach) direction"""

    name = "Distance"
    task_size: int = 1

    def _compute(self, distance: float):

        delta = self.frameA.T[0:3, 3] - self.frameB.T[0:3, 3]

        self.A = np.array([delta.T.dot(self.J()[:3])])
        self.bound = distance - np.linalg.norm(delta)


class PlaneTask(RelativeTask, EqTask):
    name = "Plane"
    task_size: int = 1

    def _compute(self):
        """Move eef within plane given by normal vector and distance to origin"""
        Tc = self.frameA.T
        Tt = self.frameB.T

        normal = Tt[0:3, 2]
        dist = normal.dot(Tt[0:3, 3])

        self.A = np.array([normal.T.dot(self.J()[:3])])
        self.bound = np.array([-(normal.dot(Tc[0:3, 3]) - dist)])


class ParallelTask(RelativeTask, EqTask):
    name = "Parallel"
    task_size: int = 3

    def _compute(self, robot_axis, target_axis):
        """Align axis in eef frame to be parallel to reference axis in base frame"""

        # transform axis from eef frame to base frame
        axis = self.frameA.T[0:3, 0:3].dot(robot_axis)
        ref = self.frameB.T[0:3, 0:3].dot(target_axis)
        self.A = (skew(ref).dot(skew(axis))).dot(self.J()[3:])
        print(self.A.shape, self.bound.shape)
        self.bound = np.cross(ref, axis)


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
