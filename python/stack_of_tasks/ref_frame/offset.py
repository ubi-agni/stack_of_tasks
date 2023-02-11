from typing import Optional, overload
from typing_extensions import Self

import numpy as np

from stack_of_tasks.utils import Callback

from . import HasJacobian, HasTransform, Jacobian, Transform, Transformable


class Offset(HasTransform, Transformable):
    @overload
    def __init__(self, frame: HasTransform):
        ...

    @overload
    def __init__(self, frame: HasTransform, offset: Transform):
        ...

    def __init__(self, frame: HasTransform, offset: Optional[Transform] = None) -> None:
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
        self._offset[:] = offset
        self.callback(self._offset)

    @property
    def T(self):
        return self.frame.T.dot(self._offset)

    def transform(self: Self, matrix: Transform) -> Self:
        self.offset = self.offset.dot(matrix)
        return self


class OffsetWithJacobian(Offset, HasJacobian):
    def __init__(self, frame, offset: Optional[Transform] = None):
        assert isinstance(frame, HasTransform) and isinstance(frame, HasJacobian)
        super().__init__(frame, offset)
        self.frame: HasJacobian

    @property
    def J(self) -> Jacobian:
        return self.frame.J
