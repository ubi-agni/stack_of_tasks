from __future__ import annotations

from collections.abc import Mapping
from enum import Enum

from typing import Any

import yaml

from stack_of_tasks.ref_frame import RefFrame
from stack_of_tasks.utils.traits import BaseSoTHasTraits


class SotDumper(yaml.Dumper):
    def __init__(
        self,
        stream: _WriteStream[Any],
        default_style: str | None = None,
        default_flow_style: bool | None = False,
        canonical: bool | None = None,
        indent: int | None = None,
        width: int | _Inf | None = None,
        allow_unicode: bool | None = None,
        line_break: str | None = None,
        encoding: str | None = None,
        explicit_start: bool | None = None,
        explicit_end: bool | None = None,
        version: tuple[int, int] | None = None,
        tags: Mapping[str, str] | None = None,
        sort_keys: bool = True,
    ) -> None:
        super().__init__(
            stream,
            default_style,
            default_flow_style,
            canonical,
            indent,
            width,
            allow_unicode,
            line_break,
            encoding,
            explicit_start,
            explicit_end,
            version,
            tags,
            sort_keys,
        )

        self.categories = {RefFrame: "frames"}

        self.add_multi_representer(BaseSoTHasTraits, SotDumper.task_representer)
        self.add_multi_representer(Enum, SotDumper.enum_representer)

    def represent(self, data):
        x = self.represent_data(data)

        full_data = {"sot": data}
        for obj in self.object_keeper:
            for cls, category in self.categories.items():
                if isinstance(obj, cls):
                    d: list = full_data.setdefault(category, [])
                    d.append(obj)

        full_node = self.represent_data(full_data)

        self.serialize(full_node)
        self.represented_objects = {}
        self.object_keeper = []
        self.alias_key = None

    @staticmethod
    def task_representer(dumper, data: BaseSoTHasTraits):
        state = data.__getstate__()
        state.pop("__traits_version__")

        return dumper.represent_mapping(f"!SOT_{data.__class__.__name__}", state)

    @staticmethod
    def enum_representer(dumper, data: Enum):
        return dumper.represent_scalar("!enum", f"{type(data).__name__}.{data.name}")


def dump(data):
    return yaml.dump(data, Dumper=SotDumper)
