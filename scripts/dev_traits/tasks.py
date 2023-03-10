#!/usr/bin/env python3

from abc import ABC, abstractmethod
from enum import Enum

import traits.api as ta
import traits.observation.expression as te
from ref import HasJacobian, RefFrame


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
    name = "BaseClass"
    task_size = -1

    softness_type = ta.Enum(TaskSoftnessType)
    weight = ta.Range(
        0.0, value=1.0, exclude_low=True, desc="The weight of this task in its task-level."
    )

    residual = None
    violation = None
    importance = None

    def __init__(self, softnessType: TaskSoftnessType, weight: float = 1.0, **traits) -> None:
        super().__init__(softness_type=softnessType, weight=weight, **traits)

    # TODO somehow trigger compute when parameters change, this needs to be extended in subclasses
    @abstractmethod
    def compute(self):
        pass


class RelativeTask(Task, ABC):
    relType = ta.Enum(RelativeType, default=0)

    refA = ta.Instance(RefFrame)
    refB = ta.Instance(RefFrame)

    def __init__(
        self,
        refA: RefFrame,
        refB: RefFrame,
        softnessType: TaskSoftnessType,
        weight: float = 1,
    ) -> None:
        super().__init__(softnessType, weight, refA=refA, refB=refB)

    _J = ta.Property(
        observe=(maybe_child("refA", "J") | maybe_child("refB", "J") | te.trait("relType"))
    )

    @ta.cached_property
    def _get__J(self):
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
    # TODO add bound and add dynamic listeners
    pass


class PositionTask(EqTask, RelativeTask):
    name = "Position"
    task_size: int = 3

    def compute(self):
        return self._J[:3], self.refA.T[:3, 3] - self.refB.T[:3, 3]
