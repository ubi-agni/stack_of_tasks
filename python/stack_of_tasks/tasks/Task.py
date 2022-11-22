from abc import ABC, abstractmethod
from enum import Enum, auto
from inspect import signature
from typing import Any, List, NoReturn, Optional, Tuple, Union, overload

import numpy as np
from numpy.typing import ArrayLike, NDArray

from stack_of_tasks.ref_frame import HasJacobian, HasTransform
from stack_of_tasks.ref_frame.frames import JointFrame, RefFrame
from stack_of_tasks.ref_frame.offset import OffsetJointFrame, OffsetRefFrame
from stack_of_tasks.robot_model import RobotModel


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

    def __init__(self, softnessType: TaskSoftnessType, weight: float) -> None:
        args = {p.name for p in signature(self._compute).parameters.values()}
        # initialize argmap as i -> i

        self.argmap = dict([(i, i) for i in args])
        # extract arguments of _compute() method

        self.weight = weight
        self.softnessType = softnessType

        self.residual = None
        self.violation = None
        self.importance = None

        self.A: NDArray = np.zeros((0, self.task_size))

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
    def _compute(self):
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


class RelativeTask(Task):
    class RelativeType(Enum):
        A_FIXED = 0
        B_FIXED = 1
        RELATIVE = 2

    @overload
    def __init__(
        self,
        frameA: Union[JointFrame, OffsetJointFrame],
        frameB: Union[JointFrame, OffsetJointFrame],
        softness: TaskSoftnessType,
        relType: RelativeType,
        weight: float = ...,
    ):
        ...

    @overload
    def __init__(
        self,
        frameA: Union[JointFrame, OffsetJointFrame],
        frameB: Union[RefFrame, OffsetRefFrame],
        softness: TaskSoftnessType,
        weight: float = ...,
    ):
        ...

    def __init__(
        self,
        frameA: Union[RefFrame, OffsetRefFrame],
        frameB: Union[JointFrame, OffsetJointFrame],
        softness: TaskSoftnessType = TaskSoftnessType.linear,
        relType: Optional[RelativeType] = None,
        weight: float = 1.0,
    ) -> None:

        super(RelativeTask, self).__init__(softness, weight)

        assert isinstance(frameA, HasTransform) and isinstance(frameB, HasTransform)

        self.frameA = frameA
        self.frameB = frameB
        self.relType = relType

        if isinstance(self.frameA, HasJacobian) and isinstance(self.frameB, HasJacobian):
            assert relType is not None

            if relType is RelativeTask.RelativeType.A_FIXED:
                self.J = lambda: -self.frameB.J
            elif relType is RelativeTask.RelativeType.B_FIXED:
                self.J = lambda: self.frameA.J
            else:
                self.J = lambda: self.frameA.J - self.frameB.J

        elif isinstance(self.frameA, HasJacobian):
            print("FrameA")
            self.J = lambda: self.frameA.J

        elif isinstance(self.frameB, HasJacobian):
            print("FrameB")
            self.J = lambda: -self.frameB.J

        else:
            raise ValueError("Either frameA or frameB has to be a JointOffsetTransform!")


class JointTask:
    def __init__(
        self,
        robot_model: RobotModel,
    ) -> None:
        pass
        # self.task_size = len(robot_model.active_joints)
        # self.A = np.identity(self.task_size)
        # self.bound = np.zeros((1, self.task_size))


#


class EqTask(Task):
    def __init_subclass__(cls) -> None:
        cls.bound = np.zeros((1, cls.task_size))

    def compute(self, data) -> Tuple[ArrayLike, ArrayLike]:
        super().compute(data)
        return self.A, self.bound


class IeqTask(Task):
    def __init_subclass__(cls) -> None:
        cls.lower_bound = np.zeros((1, cls.task_size))
        cls.upper_bound = np.zeros((1, cls.task_size))

    def compute(self, data) -> Tuple[ArrayLike, ArrayLike, ArrayLike]:
        super().compute(data)
        return self.A, self.lower_bound, self.upper_bound


TaskType = Union[EqTask, IeqTask]
TaskHierarchyType = List[List[TaskType]]
