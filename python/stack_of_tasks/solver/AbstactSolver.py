from abc import ABC, abstractmethod


class Solver(ABC):
    def __init__(self, number_of_joints, **options) -> None:
        self.N = number_of_joints

    @abstractmethod
    def solve(self, stack_of_tasks, lower_dq, upper_dq, **options):
        pass
