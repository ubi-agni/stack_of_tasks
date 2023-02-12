import numpy as np

from stack_of_tasks.ref_frame import Transform
from stack_of_tasks.ref_frame.frames import RefFrame
from stack_of_tasks.ref_frame.offset import Offset


class World(RefFrame):
    __instance: "World" = None
    __origin: Transform = np.identity(4)

    def __new__(cls):
        if cls.__instance is None:
            cls.__instance = super().__new__(cls)
            cls.__origin.setflags(write=False)

        return cls.__instance

    @property
    def T(self) -> Transform:
        return self.__origin

    def transform(self, matrix: Transform) -> Offset:
        return Offset(self).transform(matrix)
