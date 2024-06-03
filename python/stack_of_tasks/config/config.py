from collections import namedtuple
from pathlib import Path

from typing import Tuple

from stack_of_tasks.tasks.hierarchy import TaskHierarchy


class InstanceData:
    def __init__(self) -> None:
        self._object_data: dict = {"frames": [], "marker": []}
        self._task_data: dict[int, list[SoTInstancingData]] = {}

        self._instanced_objects = None
        self._instanced_stack = None

    @property
    def stack_of_tasks(self):
        if self._instanced_stack is None:
            s = TaskHierarchy()
            for v in self._task_data.values():
                with s.new_level() as l:
                    l.extend([td.instance for td in v])

            self._instanced_stack = s

        return self._instanced_stack

    @property
    def objects(self):
        return {k: [o.instance for o in v] for k, v in self._object_data.items()}


class Parameter:
    def __init__(self) -> None:
        self.solver = None
        self.actuator = None

        self.params = {}


class Configuration:
    def __init__(self, actuator_cls, solver_cls) -> None:
        self.name = None
        self.parameter: Parameter = Parameter()
        self.parameter.actuator = SoTInstancingData(actuator_cls, {})
        self.parameter.solver = SoTInstancingData(solver_cls, {})

        self.instancing_data: InstanceData = InstanceData()

    @classmethod
    def from_data(cls, data: dict) -> "Configuration":
        c = Configuration(None, None)

        p = c.parameter
        i = c.instancing_data

        name = data.pop("name", "")
        c.name = name

        settings: dict = data.pop("settings")

        i._task_data = data.pop("stack_of_tasks")
        i._object_data = data

        p.solver = settings.pop("solver")
        p.actuator = settings.pop("actuator")

        p.params = settings

        return c

    def to_data(self) -> Tuple[dict, dict]:

        data = {
            "settings": (settings := {}),
            "stack_of_tasks": (sot := {}),
        }

        return data


from .yaml.loader import SoTInstancingData
