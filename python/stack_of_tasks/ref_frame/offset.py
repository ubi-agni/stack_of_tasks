from typing import List, Union, overload

import numpy as np
from numpy.typing import NDArray
from typing_extensions import Self

from stack_of_tasks.utils import Callback

from . import HasJacobian, HasTransform, Jacobian, Transform
from .frames import RefFrame


class OffsetRefFrame(HasTransform):
    @overload
    def __init__(self, frame: RefFrame):
        ...

    @overload
    def __init__(self, frame: RefFrame, offset: Transform):
        ...

    def __init__(self, frame: RefFrame, offset: NDArray = None) -> None:
        self.frame = frame
        self.callback = Callback()

        if offset is None:
            self._offset = np.identity(4)
        else:
            self._offset = offset

    @property
    def offset(self):
        return self._offset

    @offset.setter
    def offset(self, offset: Transform):
        self._offset = offset
        self.callback(self._offset)

    @property
    def T(self):
        return self.frame.T.dot(self._offset)

    @overload
    def translate(self, /, x: float) -> Self:
        pass

    @overload
    def translate(self, /, y: float) -> Self:
        pass

    @overload
    def translate(self, /, z: float) -> Self:
        pass

    @overload
    def translate(self, /, x: float, y: float) -> Self:
        pass

    @overload
    def translate(self, /, x: float, z: float) -> Self:
        pass

    @overload
    def translate(self, /, y: float, z: float) -> Self:
        pass

    @overload
    def translate(self, /, x: float, y: float, z: float) -> Self:
        pass

    @overload
    def translate(self, /, vector: List[float]) -> Self:
        pass

    def translate(
        self,
        /,
        x: float = None,
        y: float = None,
        z: float = None,
        vector: Union[NDArray, List[float]] = None,
    ) -> Self:
        t = np.identity(4)
        if vector is not None:
            t[:3, 3] = vector

        if x is not None:
            t[0, 3] = x

        if y is not None:
            t[1, 3] = y

        if z is not None:
            t[2, 3] = z

        self.offset = self.offset.dot(t)

        return self

    def transform(self, matrix: Transform) -> Self:
        self.offset = self.offset.dot(matrix)
        return self


class OffsetJointFrame(OffsetRefFrame, HasJacobian):
    @property
    def J(self) -> Jacobian:
        return self.frame.J
