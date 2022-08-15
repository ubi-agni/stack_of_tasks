import typing
from abc import ABC, abstractmethod

import numpy as np


class TaskDesc:
    def __init__(self, A, upper, lower, name, is_hard=True) -> None:

        self.name = name
        self.is_hard = is_hard
        self.A = A
        self.upper = upper
        self.lower = lower

    @property
    def size(self):
        return self.A.shape[0]

    def unpack(self):
        """returns A, upper, lower bounds"""
        return self.A, self.upper, self.lower


class EQTaskDesc(TaskDesc):
    def __init__(self, A, bound, name=None, is_hard=False) -> None:
        super().__init__(A, bound, bound, name, is_hard)


class Task(ABC):
    name: str
    args: typing.List

    def __init__(self, scale: float = 1.0, hard_task: bool = False) -> None:
        self._scale = scale
        self._hard = hard_task

        self.argmap = {a: a for a in self.args}

    def _map_args(self, data: dict):
        return tuple(map(data.get, self.argmap.values()))

    @abstractmethod
    def compute(self, data):
        pass

    @staticmethod
    def skew(w):
        return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
