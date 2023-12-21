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


from .controller import Controller
