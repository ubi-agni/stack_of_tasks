from abc import abstractmethod

from typing import Optional

from stack_of_tasks import syringe
from stack_of_tasks.robot_model.robot_model import RobotModel
from stack_of_tasks.tasks import TaskHierarchy
from stack_of_tasks.utils import ClassRegister
from stack_of_tasks.utils.traits import ABCSoTHasTraits

SolverRegister = ClassRegister("SolverRegister")


@SolverRegister.base
class Solver(ABCSoTHasTraits):
    @syringe.inject
    def __init__(
        self,
        robot_model: RobotModel,
        task_hierarchy: Optional[TaskHierarchy] = None,
        **options
    ) -> None:

        super().__init__(**options)

        self.N = robot_model.N
        self._task_hierarchy = task_hierarchy
        if self._task_hierarchy is not None:
            self.tasks_changed()

    def set_task_hierarchy(self, task_hierarchy: TaskHierarchy):
        self._task_hierarchy = task_hierarchy
        self.tasks_changed()

    @abstractmethod
    def tasks_changed(self):
        ...

    @abstractmethod
    def solve(self, lower_dq, upper_dq, **options):
        ...
