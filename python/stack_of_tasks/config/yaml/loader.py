from __future__ import annotations

from collections import namedtuple
from collections.abc import Mapping
from enum import Enum
from weakref import ref

from typing import Any, Type

import numpy as np
import yaml
from scipy.spatial.transform.rotation import Rotation

from stack_of_tasks.config.config import Configuration
from stack_of_tasks.utils.traits import BaseSoTHasTraits, register_all


class SoTInstancingData:
    def __init__(self, cls: Type[BaseSoTHasTraits], data: dict) -> None:
        self.cls = cls
        self.data = data

        self._instance = None

    def _instanciate(self) -> BaseSoTHasTraits:
        kwargs = {
            k: v.instance if isinstance(v, SoTInstancingData) else v
            for k, v in self.data.items()
        }

        return self.cls(**kwargs)

    @property
    def instance(self) -> BaseSoTHasTraits:
        if self._instance is None:
            i = self._instanciate()
            self._instance = i

        return self._instance


class SotYamlLoader(yaml.Loader):
    def __init__(self, stream) -> None:
        super().__init__(stream)

        self.add_multi_constructor("!SOT_", SotYamlLoader.sot_constructor)
        self.add_constructor("!enum", SotYamlLoader.enum_constructor)

        self.add_constructor("!arr", SotYamlLoader.array_constructor)
        self.add_constructor("transform", SotYamlLoader.transform_constructor)

    @staticmethod
    def sot_constructor(loader: SotYamlLoader, tag_suffix: str, node: yaml.nodes.MappingNode):
        cls: Type[BaseSoTHasTraits] = loader.find_sot_cls(tag_suffix)

        if cls is None:
            raise Exception("CLS NOT FOUND")
        else:
            data = loader.construct_mapping(node)

            return SoTInstancingData(cls, data)

    def construct_document(self, node) -> Configuration:
        data = super().construct_document(node)

        settings = data.pop("settings")

        return Configuration.from_data(settings, data)

    @staticmethod
    def find_sot_cls(cls_name: str):
        return register_all.find_class_by_name(cls_name)

    def enum_constructor(self, node: yaml.nodes.ScalarNode):
        enum_class, value = node.value.split(".")
        cls = Enum.register.find_class_by_name(enum_class)

        return cls[value]

    def transform_constructor(self, node: yaml.nodes.MappingNode):
        t = self.construct_mapping(node, True)

        T = np.identity(4)
        T[:3, 3] = t["xyz"]

        T[:3, :3] = Rotation.from_euler("ZYX", t["rpy"]).as_matrix()
        return T

    def array_constructor(self, node: yaml.nodes.SequenceNode):
        s = self.construct_sequence(node, True)
        dtype = s
        while isinstance(dtype, list):
            dtype = s[0]

        return np.fromiter(s, dtype=type(dtype))


def load(toml: str) -> Configuration:
    return yaml.load(toml, SotYamlLoader)
