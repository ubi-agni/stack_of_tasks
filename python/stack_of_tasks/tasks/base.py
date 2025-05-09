#!/usr/bin/env python3
from __future__ import annotations

from abc import ABC, abstractmethod
from enum import Enum
from functools import partial

from numpy.typing import NDArray
from typing import Tuple

import numpy as np
import traits.api as ta
import traits.observation.expression as te

from stack_of_tasks import syringe
from stack_of_tasks.ref_frame import Jacobian
from stack_of_tasks.ref_frame.frames import RefFrame, RobotRefFrame
from stack_of_tasks.robot_model.robot_model import RobotModel
from stack_of_tasks.utils import ClassRegister
from stack_of_tasks.utils.traits import ABCSoTHasTraits

UpperBound = NDArray
LowerBound = NDArray
Bound = NDArray
A = NDArray


def maybe_child(parent, name):
    return te.trait(parent).match(lambda n, _: n == name)


def maybe_trait(name):  # needed? te.trait(..., optional=True)?
    return te.match(lambda n, _: n == name)


class TaskSoftnessType(Enum):
    hard = 1
    linear = 2
    quadratic = 3


class RelativeType(Enum):
    A_FIXED = 0
    B_FIXED = 1
    RELATIVE = 2


def task_kind_check(task: Task, *types):
    for t in types:
        if isinstance(t, TaskSoftnessType) and task.softness_type is not t:
            return False
        elif isinstance(t, type) and not isinstance(task, t):
            return False
    return True


TaskRegister = ClassRegister("TaskRegister")


@TaskRegister.base
class Task(ABCSoTHasTraits):
    name = ta.Str()
    task_size = -1

    # universal task properties
    softness_type = ta.Enum(TaskSoftnessType.linear, TaskSoftnessType)

    weight = ta.Range(
        low=0.0,
        value=1.0,
        exclude_low=True,
        desc="weight of this task w.r.t. other tasks in the same level",
    )
    rho = ta.Array()

    A: A = ta.Property(depends_on="_recompute", trait=ta.Array, visible=False)

    def _get_A(self):
        return np.atleast_2d(self._compute_val[0])

    @syringe.inject
    def __init__(self, robot_model: RobotModel, **traits) -> None:
        super().__init__(**traits)
        self.N = robot_model.N
        if len(self.rho) != self.N:
            self.rho = np.ones(self.N)
        # define which properties trigger recomputation
        self.observe(self._trigger_recompute, "weight")

    _recompute = ta.Event()
    _residual_update = ta.Event()

    # function to trigger computation
    def _trigger_recompute(self, *args):
        self._recompute = True
        self._residual_update = True

    _compute_val = ta.Property()

    @ta.property_depends_on("_recompute")  # cache the compute property
    def _get__compute_val(self):
        return tuple(map(partial(np.multiply, self.weight), self.compute()))

    @abstractmethod
    def compute(self) -> Tuple[A, ...]:
        pass


class RelativeTask(Task, ABC):
    relType: RelativeType = ta.Enum(RelativeType.RELATIVE, RelativeType)

    refA: RefFrame = ta.Instance(RefFrame, label="frame A")
    refB: RefFrame = ta.Instance(RefFrame, label="frame B")

    _JA: Jacobian = ta.Property(observe=(maybe_child("refA", "J") | te.trait("relType")))
    _JB: Jacobian = ta.Property(observe=(maybe_child("refB", "J") | te.trait("relType")))

    def __init__(
        self,
        refA: RefFrame,
        refB: RefFrame,
        **traits,
    ) -> None:
        super().__init__(refA=refA, refB=refB, **traits)
        self.observe(self._trigger_recompute, "refA:T, refB:T, relType")

    @ta.observe("refA, refB, relType")
    def _validate(self, event):
        if self.refA is None or self.refB is None:
            return

        a_fixed = "J" not in self.refA.trait_names()
        b_fixed = "J" not in self.refB.trait_names()
        if a_fixed and b_fixed:
            raise ValueError("Both frames are fixed, no motion possible")
        if self.relType is RelativeType.A_FIXED and b_fixed:
            print("Falling back to RELATIVE motion as frame B is fixed")
            self.relType = RelativeType.RELATIVE
        if self.relType is RelativeType.B_FIXED and a_fixed:
            print("Falling back to RELATIVE motion as frame A is fixed")
            self.relType = RelativeType.RELATIVE

    @ta.cached_property
    def _get__JA(self):
        if "J" not in self.refA.trait_names() or self.relType is RelativeType.A_FIXED:
            return np.zeros((6, 1))
        return self.refA.J

    @ta.cached_property
    def _get__JB(self):
        if "J" not in self.refB.trait_names() or self.relType is RelativeType.B_FIXED:
            return np.zeros((6, 1))
        return self.refB.J


class TargetTask(Task, ABC):
    target: RefFrame = ta.Instance(RefFrame)
    robot: RefFrame = ta.Instance(RefFrame)

    _J: Jacobian = ta.DelegatesTo("robot", "J")

    def __init__(
        self,
        target: RefFrame,
        robot: RefFrame,
        softness_type: TaskSoftnessType,
        weight: float = 1,
        **traits,
    ) -> None:
        super().__init__(softness_type, weight, target=target, robot=robot, **traits)
        self.observe(self._trigger_recompute, "target:T, robot:T, _J")


class EqTask(Task, ABC):
    bound = ta.Property(trait=ta.Array, depends_on="_recompute", visible=False)
    residual = ta.Property(trait=ta.Array, visible=False, depends_on="_residual_update")

    def _get_bound(self):
        return np.atleast_1d(self._compute_val[1])

    def _get_residual(self):
        return self.bound

    @abstractmethod
    def compute(self) -> Tuple[A, Bound]:
        pass


class IeqTask(Task, ABC):
    upper_bound = ta.Property(observe="_compute_val")
    lower_bound = ta.Property(observe="_compute_val")
    residual = ta.Property(trait=ta.Array, visible=False, depends_on="_residual_update")

    def _get_lower_bound(self) -> LowerBound:
        return self._compute_val[1]

    def _get_upper_bound(self) -> UpperBound:
        return self._compute_val[2]

    def _get_residual(self) -> Bound:
        return np.atleast_1d(np.maximum(0, np.maximum(self.lower_bound, -self.upper_bound)))

    @abstractmethod
    def compute(self) -> Tuple[A, LowerBound, UpperBound]:
        pass


# TODO Joint-Task
