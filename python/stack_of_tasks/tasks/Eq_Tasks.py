import time

import numpy as np
import traits.api as ta

from tf.transformations import rotation_from_matrix

from stack_of_tasks import syringe
from stack_of_tasks.ref_frame import Jacobian
from stack_of_tasks.ref_frame.frames import RefFrame
from stack_of_tasks.robot_model.robot_state import RobotState
from stack_of_tasks.tasks.base import A, Bound, EqTask, RelativeTask, TargetTask, TaskSoftnessType, Tuple
from stack_of_tasks.utils.transform_math import skew


class PositionTask(RelativeTask, EqTask):
    name = "Position"
    task_size: int = 3

    def compute(self) -> Tuple[A, Bound]:
        return self._JA[:3] - self._JB[:3], self.refB.T[:3, 3] - self.refA.T[:3, 3]


class OrientationTask(RelativeTask, EqTask):
    name = "Orientation"
    task_size: int = 3

    def compute(self) -> Tuple[A, Bound]:
        tA = self.refA.T
        delta = np.identity(4)
        delta[0:3, 0:3] = tA[0:3, 0:3].T.dot(self.refB.T[0:3, 0:3])
        angle, axis, _ = rotation_from_matrix(delta)
        return self._JA[:3] - self._JB[:3], tA[0:3, 0:3].dot(angle * axis)


class LissajousTask(RelativeTask, EqTask):
    name = "Lissajous"
    task_size: int = 3
    amplitude = ta.Array(shape=(2,), dtype="float", value=np.array([0.1, 0.1]))
    ratio = ta.Array(shape=(2,), dtype="float", value=np.array([1, 2]))
    omega = ta.Float(0.1)
    phi = ta.Float(0)

    def compute(self) -> Tuple[A, Bound]:
        t = time.time()
        p = self.amplitude * np.sin(self.omega * self.ratio * t + np.array([0, self.phi * np.pi]))
        posA = self.refA.T @ np.array([p[0], p[1], 0, 1])
        return self._JA[:3] - self._JB[:3], self.refB.T[:3, 3] - posA[:3]


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
        return delta.dot(self._JA[:3] - self._JB[:3]), 0.1 * (self.distance - np.linalg.norm(delta))


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
        """Align axis in refA to be parallel to axis in refB"""

        # transform axes into base frame
        axisA = self.refA.T[0:3, 0:3].dot(self.refA_axis)
        axisB = self.refB.T[0:3, 0:3].dot(self.refB_axis)
        J = (skew(axisB) @ skew(axisA)) @ self._JA[3:] - (skew(axisA) @ skew(axisB)) @ self._JB[3:]
        return J, -np.cross(axisA, axisB)


class LineTask(RelativeTask, EqTask):
    name = "Line"
    task_size: int = 3
    refA_axis = Axis()

    def compute(self) -> Tuple[A, Bound]:
        """Constrain refB to lie on a line going through refA and along refA_axis"""
        # constraint: axis x dp == 0 where dp = refB - refA
        axis = self.refA.T[0:3, 0:3].dot(self.refA_axis)
        dp = self.refB.T[0:3, 3] - self.refA.T[0:3, 3]
        dJ = self._JB[:3] - self._JA[:3]
        return skew(axis) @ dJ + (skew(dp) @ skew(axis)) @ self._JA[3:], np.cross(dp, axis)


class RayTask(RelativeTask, EqTask):
    name = "Ray"
    task_size: int = 3
    refA_axis = Axis()
    mode = ta.Enum("acos", "axis + dp", "axis only")

    def compute(self) -> Tuple[A, Bound]:
        """Constrain refB to lie on the ray pointing from refA along refA_axis"""
        # constraint: axis.T @ (dp) / ∥dp∥ == 1 where dp = refB - refA
        axis = self.refA.T[0:3, 0:3].dot(self.refA_axis)
        dp = self.refB.T[0:3, 3] - self.refA.T[0:3, 3]
        norm = np.linalg.norm(dp)

        if norm < 1e-6:  # refA is close enough to refB
            return np.zeros((1, np.maximum(self._JA.shape[1], self._JB.shape[1]))), 0

        if self.mode == "axis only":  # axis.T @ dp = ∥dp∥, ignoring derivative of dp
            return (dp.T @ skew(axis)) @ self._JA[3:], axis.T @ dp - norm

        elif self.mode == "axis + dp":  # axis.T @ dp = ∥dp∥, considering derivative of dp
            dJp = self._JB[:3] - self._JA[:3]
            return (dp.T @ skew(axis)) @ self._JA[3:] - (axis.T - dp.T / norm) @ dJp, axis.T @ dp - norm

        elif self.mode == "acos":  # acos(axis.T @ dp / norm) == 0
            dJp = self._JB[:3] - self._JA[:3]
            dpn = dp / norm
            sp = axis.T @ dpn
            acos_scale = -1.0 / np.sqrt(max(1.0 - sp**2, 1e-8))
            J = dpn.T @ skew(axis) @ self._JA[3:] + (((axis.T / norm) @ skew(dpn)) @ skew(dpn)) @ dJp
            return acos_scale * J, np.arccos(sp)


class JointTask(EqTask):
    name = "Joint"
    _robot: RobotState = ta.Instance(RobotState)
    target = ta.Array(dtype="float", comparison_mode=ta.ComparisonMode.none)

    @syringe.inject
    def __init__(self, robot_state: RobotState, **traits) -> None:
        super().__init__(_robot=robot_state, **traits)
        self.task_size = self._robot.robot_model.N
        if len(self.target) != self.task_size:
            self.target = 0.5 * (self._robot.robot_model.mins + self._robot.robot_model.maxs)
        self.observe(self._trigger_recompute, "_robot:joint_values")

    def compute(self) -> Tuple[A, Bound]:
        return np.eye(self.task_size), self.target - self._robot.joint_values
