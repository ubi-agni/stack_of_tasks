from abc import ABC
from stack_of_tasks.ref_frame import HasTransform, Transformable


class RefFrame(HasTransform, Transformable, ABC):
    registered_classes = frozenset()

    def __init_subclass__(cls, *args, **kwargs) -> None:
        RefFrame.registered_classes = frozenset([*RefFrame.registered_classes, cls])
        return super().__init_subclass__(*args, **kwargs)


from .World import World
from .Joint import JointFrame
