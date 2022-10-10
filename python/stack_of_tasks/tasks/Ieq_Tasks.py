import numpy as np

from stack_of_tasks.utils import skew

from .Task import IeqTask


class ConeTask(IeqTask):
    name = "Cone"
    task_size: int = 1

    def _compute(self, J, current, target, threshold, target_axis, robot_axis):
        """Align axis in eef frame to lie in cone spanned by reference axis and opening angle acos(threshold)"""

        threshold = np.cos(threshold)
        # transform axis from eef frame to base frame
        axis = current[0:3, 0:3].dot(robot_axis)
        reference = target[0:3, 0:3].dot(target_axis)

        self.A = np.array([reference.T.dot(skew(axis)).dot(J[3:])])
        self.upper_bound = np.array([reference.T.dot(axis) - threshold]) * 10
        self.lower_bound = np.array([-10e20])
