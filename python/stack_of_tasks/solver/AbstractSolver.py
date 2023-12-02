from abc import abstractmethod

import traits.api as ta

from stack_of_tasks.tasks.TaskHierarchy import TaskHierarchy
from stack_of_tasks.ui.utils.class_register import Register

SolverRegister = Register("SolverRegister")


@SolverRegister.register_base
class Solver(ta.ABCHasTraits):
    def __init__(
        self, number_of_joints: int, stack_of_tasks: TaskHierarchy, **options
    ) -> None:
        super().__init__(**options)
        self.N = number_of_joints
        self._stack_of_tasks = stack_of_tasks

    def set_stack_of_tasks(self, stack_of_tasks: TaskHierarchy):
        self._stack_of_tasks = stack_of_tasks
        self.stack_changed()

    @abstractmethod
    def stack_changed(self):
        ...

    @abstractmethod
    def solve(self, lower_dq, upper_dq, **options):
        ...
