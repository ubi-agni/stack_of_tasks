from abc import ABC, abstractmethod
from enum import Enum
from inspect import signature
from typing import Any, NoReturn, Optional, Tuple

import numpy as np
from numpy.typing import ArrayLike


class TaskSoftnessType(Enum):
    hard = 1
    linear = 2
    quadratic: 3


class Task(ABC):
    name: str
    task_size: int

    def __init__(self, wheight: float, softnessType: TaskSoftnessType) -> None:
        super().__init__()

        self.args = frozenset(
            [
                (p.name, None if p.annotation is p.empty else p.annotation)
                for p in signature(self._compute).parameters.values()
            ]
        )
        self.argmap = dict([(i, i) for i in self.args])

        self.weight = wheight
        self.softnessType = softnessType

        self.A: ArrayLike = np.zeros((0, self.task_size))

        self.residual = None
        self.violation = None
        self.importance = None

    def set_argument_mapping(self, argument: str, mapping_value: str) -> Optional[NoReturn]:
        if argument not in self.args:
            raise LookupError(
                f"No such argument '{argument}' in task {self.__class__.name}.\n  Possible mappings are: {self.args}"
            )

        self.argmap[argument] = mapping_value

    def _map_args(self, data: dict):
        return {k: data.get(v) for k, v in self.argmap.items()}

    @abstractmethod
    def _compute(self, *args, **kwargs):
        pass

    @abstractmethod
    def compute(self, data) -> Any:
        mapped = self._map_args(data)
        self._compute(**mapped)


class EqTask(Task):
    def __init__(self, wheight: float, softnessType: TaskSoftnessType) -> None:
        super().__init__(wheight, softnessType)
        self.bound = np.zeros((1, 0))

    @abstractmethod
    def _compute(self, *args, **kwargs):
        pass

    def compute(self, data) -> Tuple[ArrayLike, ArrayLike]:
        super().compute(data)
        return self.A, self.bound


class IeqTask(Task):
    def __init__(self, wheight: float, softnessType: TaskSoftnessType) -> None:
        super().__init__(wheight, softnessType)

        self.lower_bound = np.zeros((1, 0))
        self.upper_bound = np.zeros((1, 0))

    def compute(self, data) -> Tuple[ArrayLike, ArrayLike, ArrayLike]:
        super().compute(data)
        return self.A, self.lower_bound, self.upper_bound
