from abc import abstractmethod

import traits.api as ta

from stack_of_tasks.tasks import TaskHierarchy
from stack_of_tasks.utils import ClassRegister

SolverRegister = ClassRegister("SolverRegister")


@SolverRegister.base
class Solver(ta.ABCHasTraits):
    def __init__(
        self, number_of_joints: int, task_hierarchy: TaskHierarchy, **options
    ) -> None:
        super().__init__(**options)
        self.N = number_of_joints
        self._task_hierarchy = task_hierarchy

    def set_task_hierarchy(self, task_hierarchy: TaskHierarchy):
        self._task_hierarchy = task_hierarchy
        self.tasks_changed()

    def tasks_changed(self):
        ...

    @abstractmethod
    def solve(self, lower_dq, upper_dq, **options):
        ...
