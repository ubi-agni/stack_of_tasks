from typing_extensions import Protocol, runtime_checkable

import numpy as np

from tf.transformations import quaternion_matrix

Transform = np.typing.NDArray
Jacobian = np.typing.NDArray


@runtime_checkable
class HasTransform(Protocol):
    @property
    def T(self) -> Transform:
        pass


@runtime_checkable
class HasJacobian(Protocol):
    @property
    def J(self) -> Jacobian:
        pass


@runtime_checkable
class Transformable(Protocol):
    def transform(self, matrix: Transform) -> Transformable:
        pass

    def translate(self, x=0.0, y=0.0, z=0.0) -> Transformable:
        t = np.identity(4)
        t[0:3, 3] = [x, y, z]
        return self.transform(t)

    def rotate(self, x: float, y: float, z: float, w: float) -> Transformable:
        return self.transform(quaternion_matrix([x, y, z, w]))
