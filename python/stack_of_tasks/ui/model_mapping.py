from __future__ import annotations

from typing import Hashable


class SingeltonKeys:
    def __init__(self, decorated):
        self._register = {}
        self._decorated = decorated

    def __call__(self, item):
        if item in self._register:
            return self._register[item]
        else:
            key = self._decorated(item)
            self._register[item] = key
            return key


@SingeltonKeys
class ClassKey:
    def __init__(self, item) -> None:
        self.item = item


class ModelMapping:
    _instance = None

    @classmethod
    def _get_instance(cls):
        if ModelMapping._instance is None:
            ModelMapping._instance = ModelMapping()
        return ModelMapping._instance

    def __init__(self) -> None:
        self._mapping = {}

    @classmethod
    def add_mapping(cls: ModelMapping, key: Hashable, val):
        cls._get_instance()._mapping[key] = val

    @classmethod
    def get_mapping(cls: ModelMapping, key):
        return cls._get_instance()._mapping.get(key, None)


class InjectionArg:
    pass
