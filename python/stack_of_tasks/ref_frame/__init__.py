from __future__ import annotations

from typing import Protocol, runtime_checkable

import numpy as np
import traits.api as ta

import tf.transformations as tfs

from stack_of_tasks.utils import ClassRegister
from stack_of_tasks.utils.traits import ABCSoTHasTraits, matrix_edit

Transform = np.ndarray
Jacobian = np.ndarray


RefFrameRegister = ClassRegister("RefFrameRegister")


@runtime_checkable
class HasJacobian(Protocol):
    """Typing protocol to check wether a class has a Jacobian member.

    This protocol is runtime checkable.
    """

    J: Jacobian


@runtime_checkable
class HasTransform(Protocol):
    """Typing protocol to check wether a class has a Transform member.

    This protocol is runtime checkable.
    """

    T: Transform


@RefFrameRegister.base
class RefFrame(ABCSoTHasTraits):
    """The base class for all reference frames."""

    name: str = ta.Str(default_value="")
    T: Transform

    def transform(self, T_matrix: Transform) -> Offset:
        return Offset(self, T_matrix)

    def translate(self, x=0.0, y=0.0, z=0.0) -> Offset:
        return self.transform(tfs.translation_matrix([x, y, z]))

    def rotate_quaternion(self, quaternion: np.ndarray) -> Offset:
        return self.transform(tfs.quaternion_matrix(quaternion))

    def rotate_axis_angle(self, axis: np.ndarray, angle: float) -> Offset:
        return self.transform(tfs.rotation_matrix(angle, axis))


class Offset(RefFrame):
    """Defines an offset to the given frame."""

    offset: Transform = ta.Array(
        dtype="float",
        value=np.identity(4),
        comparison_mode=ta.ComparisonMode.none,
    )
    frame: RefFrame = ta.Instance(RefFrame)
    T: Transform = ta.Property(ta.Array, visible=False)

    def __init__(self, frame: HasTransform, offset=None, **kwargs) -> None:
        if offset is None:
            offset = np.identity(4)

        super(Offset, self).__init__(frame=frame, offset=offset, **kwargs)

        # hasattr does not work see: https://github.com/enthought/traits/issues/1753
        if "J" in self.frame.trait_names():
            self.add_trait("J", ta.Delegate("frame", visible=False))

    @ta.property_depends_on("frame.T, offset")
    def _get_T(self):
        return self.frame.T @ self.offset

    def transform(self, T_matrix: Transform) -> Offset:
        with matrix_edit(self, "offset"):
            self.offset[:] = self.offset @ T_matrix
        return self


from .frames import Origin, RobotRefFrame
from .marker_frame import MarkerFrame
