from __future__ import annotations

from typing import Protocol, runtime_checkable

import numpy as np
import traits.api as ta


@runtime_checkable
class HasJacobian(Protocol):
    J: np.ndarray


@runtime_checkable
class HasTransform(Protocol):
    T: np.ndarray


class RefFrame(ta.ABCHasTraits):
    T: np.ndarray


class Offset(RefFrame):
    offset = ta.Array(dtype="float")
    frame = ta.ReadOnly()
    T = ta.Property(ta.Array)

    def __init__(self, frame: HasTransform) -> None:
        super(Offset, self).__init__(frame=frame, offset=np.identity(4))

        if isinstance(self.frame, HasJacobian):
            self.add_trait("J", ta.Delegate("frame"))

    @ta.property_depends_on("frame.T, offset")
    def _get_T(self):
        return self.offset @ self.frame.T


class World(RefFrame):
    T = ta.ReadOnly(np.identity(4))

    def __init__(self) -> None:
        super().__init__()
        self.T.flags.writeable = False

    def __call__(self) -> World:
        return self


World = World()  # singelton
