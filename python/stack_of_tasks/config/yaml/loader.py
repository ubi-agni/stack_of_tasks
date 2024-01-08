from __future__ import annotations

from collections.abc import Mapping
from enum import Enum

from typing import Any

import numpy as np
import yaml
from scipy.spatial.transform.rotation import Rotation

from stack_of_tasks.ui.utils.dependency_injection import DependencyInjection
from stack_of_tasks.utils.traits import BaseSoTHasTraits, register_all


class SotYamlLoader(yaml.Loader):
    def __init__(self, stream) -> None:
        super().__init__(stream)

        self.add_multi_constructor("!SOT_", SotYamlLoader.sot_constructor)
        self.add_constructor("!enum", SotYamlLoader.enum_constructor)

        self.add_constructor("!arr", SotYamlLoader.array_constructor)
        self.add_constructor("transform", SotYamlLoader.transform_constructor)

    @staticmethod
    def sot_constructor(loader: SotYamlLoader, tag_suffix: str, node: yaml.nodes.MappingNode):
        cls = loader.find_sot_cls(tag_suffix)
        if cls is None:
            return "CANNOT FIND CLS"
        else:
            data = loader.construct_mapping(node)

            inst: BaseSoTHasTraits = DependencyInjection.create_instance(cls, data)
            return inst

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


def load(toml: str):
    return yaml.load(toml, SotYamlLoader)
