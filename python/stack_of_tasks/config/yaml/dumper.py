from __future__ import annotations

from collections.abc import Mapping
from enum import Enum

from typing import Any, Type

import traits.api as ta
import yaml
from numpy import ndarray
from scipy.spatial.transform.rotation import Rotation

from stack_of_tasks.marker import IAMarker
from stack_of_tasks.ref_frame import RefFrame
from stack_of_tasks.utils.traits import ABCSoTHasTraits, BaseSoTHasTraits

SOT_TAG_PREFIX = "!SOT_"


class SotDumper(yaml.CDumper):
    categories = {IAMarker: "marker", RefFrame: "frames"}

    def _represent_sot_cls(self, data: Type[BaseSoTHasTraits]):
        return self.represent_scalar(SOT_TAG_PREFIX + "cls", data.__name__)

    def _represent_sot_object(self, data: BaseSoTHasTraits):
        _, (cls,), state = data.__reduce_ex__(2)

        if "__traits_version__" in state:  # remove traits version info
            state.pop("__traits_version__")

        if isinstance(data, Controller):
            return self.controller_repr(data, state)
        else:
            return self.represent_mapping(SOT_TAG_PREFIX + "object:" + cls.__name__, state)

    def controller_repr(self, controller: Controller, settings: dict[str, Any]):
        # store all traits except robot_model, robot_state, and task_hierarchy as "settings"
        settings.pop("robot_model")
        settings.pop("robot_state")
        settings.pop("task_hierarchy")
        data = self.represent_data({"settings": settings})

        sot_node = self.represent_data(
            {"stack_of_tasks": {k: l for k, l in enumerate(controller.task_hierarchy.levels)}}
        )

        objs = {}
        for obj in self.object_keeper:
            for cls, category in self.categories.items():
                if isinstance(obj, cls):
                    d: list = objs.setdefault(category, [])
                    d.append(obj)

        obs_nodes = self.represent_data(objs)

        data.value.extend(obs_nodes.value)
        data.value.extend(sot_node.value)

        return data

    def _represent_enum(self: SotDumper, data: Enum):
        return self.represent_scalar(
            SOT_TAG_PREFIX + "enum",
            f"{type(data).__name__}.{data.name}",
        )

    def _represent_np_array(self, data: ndarray):
        if len((s := data.shape)) == 2 and s[0] == 4 and s[1] == 4:
            xyz = data[:3, 3].tolist()
            rpy = Rotation.from_matrix(data[:3, :3]).as_euler("ZYX").tolist()

            return self.represent_mapping(
                SOT_TAG_PREFIX + "transform", {"xyz": xyz, "rpy": rpy}, True
            )
        else:
            ln = self.represent_sequence(SOT_TAG_PREFIX + "arr", data.tolist(), True)
            ln.flow_style = True
            return ln


SotDumper.add_multi_representer(ta.MetaHasTraits, SotDumper._represent_sot_cls)
SotDumper.add_multi_representer(BaseSoTHasTraits, SotDumper._represent_sot_object)
SotDumper.add_multi_representer(ABCSoTHasTraits, SotDumper._represent_sot_object)
SotDumper.add_multi_representer(Enum, SotDumper._represent_enum)

SotDumper.add_representer(ndarray, SotDumper._represent_np_array)

SotDumper.add_representer(ta.TraitListObject, SotDumper.represent_list)
SotDumper.add_representer(ta.TraitDictObject, SotDumper.represent_dict)
SotDumper.add_representer(ta.TraitSetObject, SotDumper.represent_set)


def dump(data):
    return yaml.dump(data, Dumper=SotDumper)


from stack_of_tasks.controller import Controller
