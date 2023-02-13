from abc import ABC

from stack_of_tasks.ref_frame import HasTransform, Transformable


class RefFrame(HasTransform, Transformable, ABC):
    registered_classes = frozenset()

    def __init_subclass__(cls, *args, **kwargs) -> None:
        # register the new sub class: Why?
        RefFrame.registered_classes = frozenset([*RefFrame.registered_classes, cls])
        return super().__init_subclass__(*args, **kwargs)


from .Joint import JointFrame
from .World import World
