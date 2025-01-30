from __future__ import annotations

import inspect
import sys
from functools import partial, update_wrapper
from inspect import unwrap

from typing import Any, Callable, Dict, ForwardRef, Generic, Optional, Set, Type, TypeVar, _eval_type

_MethodType = TypeVar("_MethodType", bound=Callable)


def _resolve_annotations(
    raw_annotations: Dict[str, Type[Any]], module_name: Optional[str] = None
) -> Dict[str, Type[Any]]:
    """
    Taken from pydantic
    Partially taken from typing.get_type_hints.

    Resolve string or ForwardRef annotations into type objects if possible.
    """
    if module_name:
        base_globals: Optional[Dict[str, Any]] = sys.modules[module_name].__dict__
    else:
        base_globals = None
    annotations = {}
    for name, value in raw_annotations.items():
        if isinstance(value, str):
            if sys.version_info >= (3, 7):
                value = ForwardRef(value)
            else:
                value = ForwardRef(value)
        try:
            value = _eval_type(value, base_globals, None)
        except NameError:
            pass
        annotations[name] = value
    return annotations


class Syringe:
    def __init__(self) -> None:
        self._data: Dict[Type, Any] = {}

    def __getitem__(self, key: Type | str):
        # print("searching for inj for ", key, " ", isinstance(key, Type), isinstance(key, str))
        if isinstance(key, str):
            return self._data.get(key, None)
        # print("looking in data", self._data.items())

        for dkey, i in self._data.items():
            # print(f"checking {key} against {dkey}... {issubclass(key, dkey)}")
            if issubclass(key, dkey):
                return i

    def __setitem__(self, key, val):
        self._data[key] = val

    def inject(self, function: _MethodType) -> _InjectionDescriptor[_MethodType]:
        return _InjectionDescriptor(function, self)


class _InjectionDescriptor(Generic[_MethodType]):
    def __init__(self, method: _MethodType, syringe: Syringe):
        self.syringe = syringe
        self._method: _MethodType = method
        unwrapped: _MethodType = unwrap(self._method)
        code = unwrapped.__code__

        self._type_annotations = _resolve_annotations(unwrapped.__annotations__, unwrapped.__module__)
        self._args = inspect.getargs(code)
        self._p_n = code.co_argcount

        self._pp_n = code.co_posonlyargcount
        self._k_n = code.co_kwonlyargcount
        self._pk_n = self._p_n - self._pp_n

        self.kw_only_args = set(self._args.args[self._p_n :])

        update_wrapper(self, self._method)

    def _get_arg(self, key: str):
        # print(f"resolving annotation for {key}....")
        if (type := self._type_annotations.get(key, None)) is not None:
            # print(f".. annotation is {type}....")
            if isinstance(type, ForwardRef):
                type = _resolve_annotations({"type": type}, self._method.__module__)["type"]

            item = self.syringe[type]
            # print(f".. and item is {item}....")
            return item

    def _map_args(self, args: tuple[Any], kwargs: dict[str, Any]):
        n_args = len(args)

        new_args = list(args)

        # missing positional_only arguments:
        for key in self._args.args[n_args : self._pp_n]:
            if (inject_item := self._get_arg(key)) is not None:
                new_args.append(inject_item)

        # missing positional or kw arguments:
        for key in self._args.args[max(self._pp_n, n_args) : self._p_n] - kwargs.keys():
            # print(f"searching for {key}....")
            if (inject_item := self._get_arg(key)) is not None:
                # print(f"... setting {key} to {inject_item}")
                kwargs[key] = inject_item

        # missing positional or kw arguments:
        for key in self.kw_only_args - kwargs.keys():
            if (inject_item := self._get_arg(key)) is not None:
                kwargs[key] = inject_item

        # print(self._args)
        # print(" ", new_args, kwargs)
        return new_args, kwargs

    def __get__(self, obj=None, objtype=None) -> _MethodType:
        if obj is None:
            return self.__call__

        return partial(self.__call__, obj)

    def __call__(self, *args: Any, **kwargs: Any) -> Any:
        args, kwargs = self._map_args(args, kwargs)

        return self._method(*args, **kwargs)
