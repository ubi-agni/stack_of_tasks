import numpy as np
import traits.api as ta

from tf.transformations import rotation_from_matrix

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


class BoxTask(RelativeTask, IeqTask):
    name = "Position"
    task_size: int = 3
    lower_tol: LowerBound = ta.Array(shape=(3,), value=np.array([0.1, 0.1, 0.1]))
    upper_tol: UpperBound = ta.Array(shape=(3,), value=np.array([0.1, 0.1, 0.1]))

    def compute(self) -> Tuple[A, LowerBound, UpperBound]:
        e = self.refB.T[:3, 3] - self.refA.T[:3, 3]
        return self._JA[:3] - self._JB[:3], e - self.lower_tol, e + self.upper_tol


class ConeTask(RelativeTask, IeqTask):
    name = "Cone"
    task_size: int = 1
    softness_type = TaskSoftnessType.hard

    refA_axis = Axis
    refB_axis = Axis
    angle = ta.Float(default_value=0.5)

    def compute(self) -> Tuple[A, LowerBound, UpperBound]:
        """Constrain axes in refA and refB to have an angle smaller than self.angle"""

        threshold = np.cos(self.angle)
        # transform axes into base frame
        axisA = self.refA.T[0:3, 0:3].dot(self.refA_axis)
        axisB = self.refB.T[0:3, 0:3].dot(self.refB_axis)
        J = -(axisB.T @ skew(axisA)) @ self._JA[3:] - (axisA.T @ skew(axisB)) @ self._JB[3:]
        return J, threshold - axisA.T @ axisB.T, 10e20
