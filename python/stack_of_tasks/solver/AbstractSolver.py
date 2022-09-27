from abc import ABC, abstractmethod

from stack_of_tasks.tasks.TaskHierarchy import TaskHierarchy


class Solver(ABC):
    @abstractmethod
    def __init__(
        self, number_of_joints: int, stack_of_tasks: TaskHierarchy, **options
    ) -> None:
        self.N = number_of_joints
        self._stack_of_tasks = stack_of_tasks
        self._options = options

    def set_stack_of_tasks(self, stack_of_tasks: TaskHierarchy):
        self._stack_of_tasks = stack_of_tasks
        self.stack_changed()

    @abstractmethod
    def stack_changed(self):
        pass

    def solve(self, lower_dq, upper_dq, **options):
        pass
