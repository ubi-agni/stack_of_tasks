from __future__ import annotations

from collections import namedtuple
from collections.abc import Mapping
from enum import Enum
from weakref import ref

from typing import Any, Type

import numpy as np
import yaml
from scipy.spatial.transform.rotation import Rotation
from traits.has_traits import __newobj__

from stack_of_tasks.config.config import Configuration
from stack_of_tasks.utils.traits import BaseSoTHasTraits, register_all


class SoTInstancingData:
    def __init__(self, cls: Type[BaseSoTHasTraits], data: dict) -> None:
        self.cls = cls
        self.data = data

        self._instance = None

    def _instanciate(self) -> BaseSoTHasTraits:
        # print(self.cls, self.data)
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
    def construct_document(self, node):

        data = self.construct_mapping(node, deep=True)

        config = Configuration.from_data(data)

        return config

    def _sot_cls_contructor(self, data: yaml.ScalarNode):
        cls = self.find_sot_cls(data.value)
        if cls is None:
            raise Exception(f"Class {data.value} cloud not be found in sot classes!")

        return cls

    def _sot_obj_constructor(self, cls_name, data: yaml.MappingNode):
        cls: BaseSoTHasTraits = self.find_sot_cls(cls_name)
        state = self.construct_mapping(data, deep=True)

        return SoTInstancingData(cls, state)

    def find_sot_cls(self, cls_name: str):
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


SotYamlLoader.add_constructor("!SOT_cls", SotYamlLoader._sot_cls_contructor)
SotYamlLoader.add_constructor("!SOT_enum", SotYamlLoader.enum_constructor)
SotYamlLoader.add_multi_constructor("!SOT_object:", SotYamlLoader._sot_obj_constructor)

SotYamlLoader.add_constructor("!SOT_arr", SotYamlLoader.array_constructor)
SotYamlLoader.add_constructor("!SOT_transform", SotYamlLoader.transform_constructor)


def load(toml: str) -> Configuration:
    return yaml.load(toml, SotYamlLoader)
