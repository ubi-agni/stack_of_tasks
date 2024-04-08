import enum

from stack_of_tasks.utils.class_register import ClassRegister

# Monkeypatch Enum to have a sublcass register.
cl = ClassRegister("Enums")


@classmethod
def __init_subclass__(cls, **kwargs):
    super.__init_subclass__(**kwargs)
    for f in cls.__init_subclass_hooks__:
        f(cls)


enum.Enum.__init_subclass__ = __init_subclass__
enum.Enum = cl.base(enum.Enum)

from stack_of_tasks.utils.syringe import Syringe

syringe = Syringe()


# Load all classes so they can be found while loading
import stack_of_tasks.marker.abstract_marker
import stack_of_tasks.ref_frame.frames
import stack_of_tasks.tasks.Eq_Tasks
from stack_of_tasks.robot_model.actuators import *
from stack_of_tasks.solver import *

# print(stack_of_tasks.tasks.base.TaskRegister.classes)
