import numpy as np
import traits.api as ta

from stack_of_tasks.utils.transform_math import skew

from .Task import A, IeqTask, LowerBound, RelativeTask, Tuple, UpperBound


class ConeTask(RelativeTask, IeqTask):
    name = "Cone"
    task_size: int = 1

    robot_axis = ta.Array(shape=(1, 3))
    target_axis = ta.Array(shape=(1, 3))
    angle = ta.Float(default_value=0.0)

    def compute(self) -> Tuple[A, LowerBound, UpperBound]:
        """Align axis in eef frame to lie in cone spanned by reference axis and opening angle"""

        threshold = np.cos(self.angle)
        # transform axis from eef frame to base frame
        uk = self.refA.T[0:3, 0:3].dot(self.robot_axis)
        uref = self.refB.T[0:3, 0:3].dot(self.target_axis)

        return (
            -np.array([uref.T.dot(skew(uk)).dot(self._J[:3])]),
            np.array([-10e20]),
            np.array([(uref.T.dot(uk) - threshold)]),
        )
