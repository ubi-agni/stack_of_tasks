from __future__ import annotations

import inspect
from functools import wraps

from typing import Generic, Type, TypeVar


class ClassRegisterBaseError(Exception):
    def __init__(
        self, first_cls: Type[object], second_cls: Type[object], register: ClassRegister
    ) -> None:
        tr = f"{register.name} can't set {second_cls.__name__} as base.\n"
        tr += f"Base was already set in {inspect.getfile(first_cls)}, line {inspect.getsourcelines(first_cls)[1]}."
        super().__init__(tr)


ClassType = TypeVar("ClassType")


class ClassRegister(Generic[ClassType]):
    def __init__(self, name: str, *, include_abstract=False, include_base=False) -> None:
        self.name: str = name
        self._base_cls: ClassType = None
        self._register: list[ClassType] = []

        self._include_abstract = include_abstract  # register abstract classes too?
        self._include_base = include_base  # register the base class too?

    def base(self, cls: ClassType):
        """decorator to mark a base class (and all its subclasses) for registration with this register"""
        if self._base_cls is not None:
            raise ClassRegisterBaseError(self._base_cls, cls, self)

        self._base_cls = cls

        def hook(sub_cls):
            if (
                self._include_abstract or not inspect.isabstract(sub_cls)
            ) and sub_cls not in self._register:
                self._register.append(sub_cls)

        parent_hooks = getattr(self._base_cls, "__init_subclass_hooks__", [])
        cls.__init_subclass_hooks__ = [*parent_hooks]

        cls.__init_subclass_hooks__.append(hook)
        cls.register = self
        return cls

    def skip(self, cls):
        """decorator to skip a class from registration with this register"""
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
        if self._include_abstract:
            return [cls.__name__ for cls in self._register if not inspect.isabstract(cls)]
        return self.class_names

    @property
    def concrete_classes(self):
        if self._include_abstract:
            return [cls for cls in self._register if not inspect.isabstract(cls)]
        return self.classes

    def find_class_by_name(self, name: str):
        for c in self.classes:
            if c.__name__ == name:
                return c
