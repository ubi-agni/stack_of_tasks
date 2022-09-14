import typing
from abc import ABC, abstractmethod
from enum import Enum
from inspect import signature
from typing import Any, NoReturn, Optional, Tuple

import numpy as np
from numpy.typing import ArrayLike


class TaskSoftnessType(Enum):
    hard = 1
    linear = 2
    quadratic = 3


class Task(ABC):
    name: str
    task_size: int

    def __init__(self, weight: float, softnessType: TaskSoftnessType) -> None:
        super().__init__()

        # extract arguments of _compute() method
        args = {p.name for p in signature(self._compute).parameters.values()}
        # initialize argmap as i -> i
        self.argmap = dict([(i, i) for i in args])

        self.weight = weight
        self.softnessType = softnessType

        self.A: ArrayLike = np.zeros((0, self.task_size))

        self.residual = None
        self.violation = None
        self.importance = None

    def set_argument_mapping(self, argument: str, mapping_name: str) -> Optional[NoReturn]:
        if argument not in self.argmap.keys():
            args = ", ".join(self.argmap.keys())
            raise LookupError(
                f"No argument '{argument}' in task '{self.__class__.name}'.\n"
                f"Possible values are: {args}"
            )

        self.argmap[argument] = mapping_name

    def _map_args(self, data: dict):
        return {k: data.get(v) for k, v in self.argmap.items()}

    @abstractmethod
    def _compute(self, *args, **kwargs):
        pass

    def compute(self, data) -> Any:
        mapped = self._map_args(data)
        self._compute(**mapped)


class EqTask(Task):
    def __init__(self, weight: float, softnessType: TaskSoftnessType) -> None:
        super().__init__(weight, softnessType)
        self.bound = np.zeros((1, 0))

    def compute(self, data) -> Tuple[ArrayLike, ArrayLike]:
        super().compute(data)
        return self.A, self.bound


class IeqTask(Task):
    def __init__(self, weight: float, softnessType: TaskSoftnessType) -> None:
        super().__init__(weight, softnessType)
        self.lower_bound = np.zeros((1, 0))
        self.upper_bound = np.zeros((1, 0))

    def compute(self, data) -> Tuple[ArrayLike, ArrayLike, ArrayLike]:
        super().compute(data)
        return self.A, self.lower_bound, self.upper_bound


TaskType = typing.Union[EqTask, IeqTask]
TaskHierachyType = typing.List[typing.List[TaskType]]
