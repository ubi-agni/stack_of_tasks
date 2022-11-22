import numpy as np

from stack_of_tasks.utils.transform_math import skew

from .Task import IeqTask, RelativeTask


class ConeTask(RelativeTask, IeqTask):
    name = "Cone"
    task_size: int = 1

    def _compute(self, angle, robot_axis, target_axis):
        """Align axis in eef frame to lie in cone spanned by reference axis and opening angle"""

        threshold = np.cos(angle)
        # transform axis from eef frame to base frame
        uk = self.frameA.T[0:3, 0:3].dot(robot_axis)
        uref = self.frameB.T[0:3, 0:3].dot(target_axis)

        self.A = -np.array([uref.T.dot(skew(uk)).dot(self.J()[:3])])
        self.lower_bound = np.array([-10e20])
        self.upper_bound = np.array([(uref.T.dot(uk) - threshold)])
