from __future__ import annotations

from collections.abc import Mapping
from enum import Enum

from typing import Any

import yaml

from stack_of_tasks.utils.traits import BaseSoTHasTraits, register_all


class SotYamlLoader(yaml.Loader):
    def __init__(self, stream: _ReadStream) -> None:
        super().__init__(stream)

        self.add_multi_constructor("!SOT_", SotYamlLoader.sot_constructor)
        self.add_constructor("!enum", SotYamlLoader.enum_constructor)

    @staticmethod
    def sot_constructor(loader: SotYamlLoader, tag_suffix: str, node: yaml.nodes.MappingNode):
        cls = loader.find_sot_cls(tag_suffix)
        if cls is None:
            return "CANNOT FIND CLS"
        else:
            inst: BaseSoTHasTraits = cls.__new__(cls)
            print(inst)
            data = loader.construct_mapping(node)
            inst.__setstate__(data)
            return inst

    @staticmethod
    def find_sot_cls(cls_name: str):
        print(register_all.class_names)
        return register_all.find_class_by_name(cls_name)

    @staticmethod
    def enum_constructor(loader: yaml.Loader, node: yaml.nodes.ScalarNode):
        enum_class, value = node.value.split(".")
        return (enum_class, value)


def load(toml: str):
    return yaml.load(toml, SotYamlLoader)
