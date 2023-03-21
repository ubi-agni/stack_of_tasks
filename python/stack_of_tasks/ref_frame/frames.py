from __future__ import annotations

import numpy as np
import traits.api as ta

from . import Transform


class RefFrame(ta.ABCHasTraits):
    T: Transform = ta.Any


class Origin(RefFrame):
    T: Transform = ta.ReadOnly(np.identity(4))

    def __init__(self) -> None:
        super().__init__()
        self.T.flags.writeable = False
