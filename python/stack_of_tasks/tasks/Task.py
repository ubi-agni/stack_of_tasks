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

from stack_of_tasks.ref_frame import Jacobian
from stack_of_tasks.ref_frame.frames import RefFrame, RobotRefFrame
from stack_of_tasks.ui.utils.class_register import Register
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


TaskRegister = Register("TaskRegister", register_base=False)


@TaskRegister.register_base
class Task(ABCSoTHasTraits):
    name = "BaseClass"
    task_size = -1

    # universal task properties
    softness_type = ta.Enum(TaskSoftnessType)

    weight = ta.Range(
        low=0.0,
        value=1.0,
        exclude_low=True,
        desc="The weight of this task in its task-level.",
    )

    A: A = ta.Property(depends_on="_recompute", trait=ta.Array, visible=False)

    def _get_A(self):
        return self._compute_val[0]

    # TODO task optimization info
    residual = ta.Array(visible=True)
    violation = ta.Array(visible=True)
    importance = None

    def __init__(
        self, softness_type: TaskSoftnessType, weight: float = 1.0, **traits
    ) -> None:
        super().__init__(softness_type=softness_type, weight=weight, **traits)
        self.observe(
            self._trigger_recompute, "weight"
        )  # what properties should trigger recomputation

    _recompute = ta.Event()

    # function to trigger computation
    def _trigger_recompute(self, event_data=None):
        self._recompute = event_data

    _compute_val = ta.Property()

    @ta.property_depends_on("_recompute")  # cache the compute property
    def _get__compute_val(self):
        return tuple(map(partial(np.multiply, self.weight), self.compute()))

    @abstractmethod
    def compute(self) -> Tuple[A, ...]:
        pass


class RelativeTask(Task, ABC):
    relType: RelativeType = ta.Enum(RelativeType, default=0)

    refA: RefFrame = ta.Instance(RefFrame)
    refB: RefFrame = ta.Instance(RefFrame)

    _J: Jacobian = ta.Property(
        observe=(maybe_child("refA", "J") | maybe_child("refB", "J") | te.trait("relType"))
    )

    def __init__(
        self,
        refA: RefFrame,
        refB: RefFrame,
        softness_type: TaskSoftnessType,
        weight: float = 1,
        **traits,
    ) -> None:
        super().__init__(softness_type, weight, refA=refA, refB=refB, **traits)
        self.observe(self._trigger_recompute, "refA:T, refB:T, _J")

    @ta.cached_property
    def _get__J(self):
        a_is_J = "J" in self.refA.trait_names()
        b_is_J = "J" in self.refB.trait_names()

        if a_is_J and (not b_is_J):
            J = self.refA.J

        elif (not a_is_J) and b_is_J:
            J = -self.refB.J

        elif a_is_J and b_is_J:
            if self.relType is RelativeType.A_FIXED:
                J = -self.refB.J
            elif self.relType is RelativeType.B_FIXED:
                J = self.refA.J
            else:
                J = self.refA.J - self.refB.J
        else:
            raise ValueError("Either REF A or REF B has to have J")

        return J


class TargetTask(Task, ABC):
    target: RefFrame = ta.Instance(RefFrame)
    robot: RobotRefFrame = ta.Instance(RobotRefFrame)

    _J: Jacobian = ta.DelegatesTo("robot", "J")

    def __init__(
        self,
        target: RefFrame,
        robotFrame: RobotRefFrame,
        softness_type: TaskSoftnessType,
        weight: float = 1,
    ) -> None:
        super().__init__(softness_type, weight, target=target, robot=robotFrame)
        self.observe(self._trigger_recompute, "target:T, robot:T, _J")


class EqTask(Task, ABC):
    bound = ta.Property(trait=ta.Array, depends_on="_recompute", visible=False)

    def _get_bound(self):
        return self._compute_val[1]

    @abstractmethod
    def compute(self) -> Tuple[A, Bound]:
        pass


class IeqTask(Task, ABC):
    upper_bound = ta.Property(observe="_compute_val")
    lower_bound = ta.Property(observe="_compute_val")

    def _get_lower_bound(self) -> LowerBound:
        return self._compute_val[1]

    def _get_upper_bound(self) -> UpperBound:
        return self._compute_val[2]

    @abstractmethod
    def compute(self) -> Tuple[A, LowerBound, UpperBound]:
        pass


# TODO Joint-Task
