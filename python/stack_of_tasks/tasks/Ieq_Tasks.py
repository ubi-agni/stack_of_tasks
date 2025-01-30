import numpy as np
import traits.api as ta

from stack_of_tasks.tasks.base import (
    A,
    IeqTask,
    LowerBound,
    RelativeTask,
    TaskSoftnessType,
    Tuple,
    UpperBound,
)
from stack_of_tasks.utils.transform_math import skew

Axis = ta.Array(shape=(3,), value=np.array([0, 0, 1]))


class ConeTask(RelativeTask, IeqTask):
    name = "Cone"
    task_size: int = 1
    softness_type = TaskSoftnessType.hard

    refA_axis = Axis
    refB_axis = Axis
    angle = ta.Float(default_value=0.5)

    def compute(self) -> Tuple[A, LowerBound, UpperBound]:
        """Align axis in eef frame to lie in cone spanned by reference axis and opening angle"""

        threshold = np.cos(self.angle)
        # transform axes into base frame
        u = self.refA.T[0:3, 0:3].dot(self.refA_axis)
        uref = self.refB.T[0:3, 0:3].dot(self.refB_axis)

        return (
            np.array([uref.T.dot(skew(u)).dot(self._J[3:])]),
            -np.array([(uref.T.dot(u) - threshold)]),
            np.array([10e20]),
        )
