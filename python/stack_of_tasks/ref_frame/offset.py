from __future__ import annotations

import numpy as np
import traits.api as ta

from . import HasJacobian, HasTransform, Transform
from .frames import RefFrame


class Offset(RefFrame):
    offset: Transform = ta.Array(dtype="float")
    frame: RefFrame = ta.Instance(RefFrame)  # should be readonly.
    T: Transform = ta.Property(ta.Array)

    def __init__(self, frame: HasTransform) -> None:
        super(Offset, self).__init__(frame=frame, offset=np.identity(4))

        if isinstance(self.frame, HasJacobian):
            self.add_trait("J", ta.Delegate("frame"))

    @ta.property_depends_on("frame.T, offset")
    def _get_T(self):
        return self.offset @ self.frame.T
