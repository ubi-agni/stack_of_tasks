from __future__ import annotations

import inspect
from functools import wraps

from typing import Any, Generic, Type


class RegisterBaseError(Exception):
    def __init__(
        self, first_cls: Type[object], second_cls: Type[object], register: Register
    ) -> None:
        tr = f"{register.name} can't set {second_cls.__name__} as base.\n"
        tr += f"Base was already set in {inspect.getfile(first_cls)}, line {inspect.getsourcelines(first_cls)[1]}."
        super().__init__(tr)


from typing import TypeVar

ClassType = TypeVar("ClassType")


class Register(Generic[ClassType]):
    def __init__(self, name: str, register_abstract=False, register_base=True) -> None:
        self.name: str = name
        self._base_cls: ClassType = None
        self._register: list[ClassType] = []

        self._abstract = register_abstract
        self._base = register_base

    def register_base(self, cls: ClassType):
        if self._base_cls is not None:
            raise RegisterBaseError(self._base_cls, cls, self)

        self._base_cls = cls

        old_init = getattr(cls, "__init_subclass__")

        @classmethod
        @wraps(old_init)
        def _init_subclass(sub_cls) -> None:
            old_init()

            if self._abstract or not inspect.isabstract(sub_cls):
                self._register.append(sub_cls)

        cls.__init_subclass__ = _init_subclass
        if self._base:
            cls.__init_subclass__()

        return cls

    def skip_register(self, cls):
        if cls in self._register:
            self._register.remove(cls)
        return cls

    @property
    def class_names(self):
        return [cls.__name__ for cls in self._register]

    @property
    def classes(self):
        return [cls for cls in self._register]

    @property
    def concrete_class_names(self):
        if self._abstract:
            return [cls.__name__ for cls in self._register if not inspect.isabstract(cls)]
        return self.class_names

    @property
    def concrete_classes(self):
        if self._abstract:
            return [cls for cls in self._register if not inspect.isabstract(cls)]
        return self.classes