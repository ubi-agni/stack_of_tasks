import numpy as np

from tf.transformations import rotation_from_matrix

from stack_of_tasks.tasks.Task import EqTask, TaskSoftnessType
from stack_of_tasks.utils import skew


class PositionTask(EqTask):
    name = "Position"
    task_size: int = 3

    def _compute(self, J: np.ndarray, current: np.ndarray, target: np.ndarray):
        self.A = J[:3]
        self.bound = target[:3, 3] - current[:3, 3]


class OrientationTask(EqTask):
    name = "Orientation"
    task_size: int = 3

    def _compute(self, J, current, target):
        self.A = J[3:]

        delta = np.identity(4)
        delta[0:3, 0:3] = current[0:3, 0:3].T.dot(target[0:3, 0:3])
        angle, axis, _ = rotation_from_matrix(delta)

        self.bound = current[0:3, 0:3].dot(angle * axis)


class DistanceTask(EqTask):
    """Keep distance to target position, not considering (approach) direction"""

    name = "Distance"
    task_size: int = 1

    def _compute(self, J, current, target, distance):
        delta = current[0:3, 3] - target[0:3, 3]
        self.A = np.array([delta.T.dot(J[:3])])
        self.bound = np.array([-(np.linalg.norm(delta) - distance)])


class PlaneTask(EqTask):
    name = "Plane"
    task_size: int = 1

    def _compute(self, J, current, target):
        """Move eef within plane given by normal vector and distance to origin"""

        normal = target[0:3, 2]
        dist = normal.dot(target[0:3, 3])

        self.A = np.array([normal.T.dot(J[:3])])
        self.bound = np.array([-(normal.dot(current[0:3, 3]) - dist)])


class ParallelTask(EqTask):
    name = "Parallel"
    task_size: int = 3

    def _compute(self, J, current, target, robot_axis, target_axis):
        """Align axis in eef frame to be parallel to reference axis in base frame"""

        # transform axis from eef frame to base frame
        axis = current[0:3, 0:3].dot(robot_axis)
        ref = target[0:3, 0:3].dot(target_axis)
        self.A = (skew(ref).dot(skew(axis))).dot(J[3:])
        self.bound = np.cross(ref, axis)


class LineTask(EqTask):
    name = "Line"
    task_size: int = 3

    def _compute(self, J, current, line_point):
        normal = line_point[0:3, 2]
        sw = skew(normal)

        d = current[0:3, 3] - line_point[0:3, 3]

        self.A = sw.dot(J[:3])
        self.bound = -sw.dot(d)


class JointPos(EqTask):
    name = "Joint Position"

    def __init__(
        self, wheight: float, softnessType: TaskSoftnessType, number_of_joints: int
    ) -> None:
        self.task_size = number_of_joints
        super().__init__(wheight, softnessType)

    def _compute(self, current_joint_pose, desired_joint_pose):
        self.A = np.identity(current_joint_pose.size)
        self.bound = current_joint_pose - desired_joint_pose


if __name__ == "__main__":
    p = PositionTask(1, TaskSoftnessType.hard)
    print(p.args)
