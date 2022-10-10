import typing
from abc import ABC, abstractmethod
from enum import Enum, auto
from inspect import signature
from typing import Any, NoReturn, Optional, Tuple

import numpy as np
from numpy.typing import ArrayLike


class TaskTypes(Enum):
    HARD = auto()
    HARD_EQ = auto()
    HARD_IEQ = auto()

    LINEAR = auto()
    LINEAR_EQ = auto()
    LINEAR_IEQ = auto()

    QUADRATIC = auto()
    QUADRATIC_EQ = auto()
    QUADRATIC_IEQ = auto()


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

    def is_task_type(self, taskType: TaskTypes) -> bool:
        if taskType is TaskTypes.HARD:
            return self.softnessType is TaskSoftnessType.hard
        elif taskType is TaskTypes.HARD_EQ:
            return self.softnessType is TaskSoftnessType.hard and isinstance(self, EqTask)
        elif taskType is TaskTypes.HARD_IEQ:
            return self.softnessType is TaskSoftnessType.hard and isinstance(self, IeqTask)

        elif taskType is TaskTypes.LINEAR:
            return self.softnessType is TaskSoftnessType.linear
        elif taskType is TaskTypes.LINEAR_EQ:
            return self.softnessType is TaskSoftnessType.linear and isinstance(self, EqTask)
        elif taskType is TaskTypes.LINEAR_IEQ:
            return self.softnessType is TaskSoftnessType.linear and isinstance(self, IeqTask)

        elif taskType is TaskTypes.QUADRATIC:
            return self.softnessType is TaskSoftnessType.quadratic
        elif taskType is TaskTypes.QUADRATIC_EQ:
            return self.softnessType is TaskSoftnessType.quadratic and isinstance(
                self, EqTask
            )
        elif taskType is TaskTypes.QUADRATIC_IEQ:
            return self.softnessType is TaskSoftnessType.quadratic and isinstance(
                self, IeqTask
            )


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
TaskHierarchyType = typing.List[typing.List[TaskType]]
