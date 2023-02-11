from __future__ import annotations

from typing import Sequence, overload
import numpy as np
from numpy.typing import NDArray
from typing_extensions import Protocol, Self, runtime_checkable

from tf.transformations import quaternion_matrix
from abc import abstractmethod

Transform = NDArray
Jacobian = NDArray


@runtime_checkable
class HasTransform(Protocol):
    @property
    @abstractmethod
    def T(self) -> Transform:
        pass


@runtime_checkable
class HasJacobian(Protocol):
    @property
    @abstractmethod
    def J(self) -> Jacobian:
        pass


@runtime_checkable
class Transformable(Protocol):
    @overload
    def transform(self, matrix: Transform) -> Transformable:
        pass

    @overload
    def transform(self: Self, matrix: Transform) -> Transformable:
        pass

    @overload
    def translate(self, /, x: float) -> Transformable:
        pass

    @overload
    def translate(self, /, y: float) -> Transformable:
        pass

    @overload
    def translate(self, /, z: float) -> Transformable:
        pass

    @overload
    def translate(self, /, x: float, y: float) -> Transformable:
        pass

    @overload
    def translate(self, /, x: float, z: float) -> Transformable:
        pass

    @overload
    def translate(self, /, y: float, z: float) -> Transformable:
        pass

    @overload
    def translate(self, /, x: float, y: float, z: float) -> Transformable:
        pass

    @overload
    def translate(self, /, vector: Sequence[float]) -> Transformable:
        pass

    def translate(self, /, x=None, y=None, z=None, vector=None) -> Transformable:
        t = np.identity(4)
        if vector is not None:
            t[:3, 3] = vector

        if x is not None:
            t[0, 3] = x

        if y is not None:
            t[1, 3] = y

        if z is not None:
            t[2, 3] = z

        return self.transform(t)

    def rotate(self, x: float, y: float, z: float, w: float) -> Transformable:
        return self.transform(quaternion_matrix([x, y, z, w]))
