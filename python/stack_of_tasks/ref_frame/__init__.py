from __future__ import annotations

from typing import Protocol, runtime_checkable

import numpy as np

Transform = np.ndarray
Jacobian = np.ndarray


@runtime_checkable
class HasJacobian(Protocol):
    J: Jacobian


@runtime_checkable
class HasTransform(Protocol):
    T: Transform
