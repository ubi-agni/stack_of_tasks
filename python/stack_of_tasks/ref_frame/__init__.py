from abc import ABC, abstractmethod

import numpy as np
from numpy.typing import NDArray

Transform = NDArray
Jacobian = NDArray


class HasTransform(ABC):
    @property
    @abstractmethod
    def T(self) -> Transform:
        pass


class HasJacobian(ABC):
    @property
    @abstractmethod
    def J(self) -> Jacobian:
        pass
