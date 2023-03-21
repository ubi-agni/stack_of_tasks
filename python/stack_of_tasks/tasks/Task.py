#!/usr/bin/env python3
from __future__ import annotations

from abc import ABC, abstractmethod
from enum import Enum

from numpy.typing import NDArray
from typing import Tuple

import traits.api as ta
import traits.observation.expression as te

from stack_of_tasks.ref_frame import HasJacobian, Jacobian
from stack_of_tasks.ref_frame.frames import RefFrame

UpperBound = NDArray
LowerBound = NDArray
Bound = NDArray
A = NDArray


def maybe_child(parent, name):
    return te.trait(parent).match(lambda n, _: n == name)


def maybe_trait(name):
    return te.match(lambda n, _: n == name)


class TaskSoftnessType(Enum):
    hard = 1
    linear = 2
    quadratic = 3


class RelativeType(Enum):
    A_FIXED = 0
    B_FIXED = 1
    RELATIVE = 2


class Task(ta.ABCHasTraits):
    # Task Constants
    name = "BaseClass"
    task_size = -1

    # universal task properties
    softness_type = ta.Enum(TaskSoftnessType)
    weight = ta.Range(
        0.0, value=1.0, exclude_low=True, desc="The weight of this task in its task-level."
    )

    A: A = ta.Property(observe="_compute_val")

    def _get_A(self):
        return self._compute_val[0]

    # TODO task optimization info
    residual = None
    violation = None
    importance = None

    def __init__(self, softnessType: TaskSoftnessType, weight: float = 1.0, **traits) -> None:
        super().__init__(softness_type=softnessType, weight=weight, **traits)
        self.observe(
            self._trigger_recompute, "weight"
        )  # what properties should trigger recomputation

    # (re)computation mechanism
    _recompute = ta.Event()

    # function to trigger computation
    def _trigger_recompute(self, event_data=None):
        self._recompute = event_data

    _compute_val = ta.Property(observe="_recompute")

    @ta.cached_property  # cache the compute property
    def _get__compute_val(self):
        return self.compute()

    @abstractmethod
    def compute(self) -> Tuple[A, ...]:
        pass


class RelativeTask(Task, ABC):
    relType: RelativeType = ta.Enum(RelativeType, default=0)

    refA: RefFrame = ta.Instance(RefFrame)
    refB: RefFrame = ta.Instance(RefFrame)

    def __init__(
        self,
        refA: RefFrame,
        refB: RefFrame,
        softnessType: TaskSoftnessType,
        weight: float = 1,
    ) -> None:
        super().__init__(softnessType, weight, refA=refA, refB=refB)
        self.observe(self._trigger_recompute, "refA:T, refB:T, _J")

    _J: Jacobian = ta.Property(
        observe=(maybe_child("refA", "J") | maybe_child("refB", "J") | te.trait("relType"))
    )

    @ta.cached_property
    def _get__J(self):
        # TODO can be optimized.
        if isinstance(self.refA, HasJacobian) and isinstance(self.refB, HasJacobian):
            if self.relType is RelativeType.A_FIXED:
                J = -self.refB.J
            elif self.relType is RelativeType.B_FIXED:
                J = self.refA.J
            else:
                J = self.refA.J - self.relType.J

        elif isinstance(self.refA, HasJacobian):
            J = self.refA.J

        elif isinstance(self.refA, HasJacobian):
            J = -self.refB.J

        return J


class EqTask(Task, ABC):
    bound = ta.Property(observe="_compute_val")

    def _get_bound(self):
        return self._compute_val[1]

    @abstractmethod
    def compute(self) -> Tuple[A, Bound]:
        return super().compute()


class IeqTask(Task, ABC):
    upper_bound = ta.Property(observe="_compute_val")
    lower_bound = ta.Property(observe="_compute_val")

    def _get_bound(self):
        return self._compute_val[1]

    def _get_bound(self):
        return self._compute_val[2]

    @abstractmethod
    def compute(self) -> Tuple[A, LowerBound, UpperBound]:
        return super().compute()


# TODO Joint-Task
