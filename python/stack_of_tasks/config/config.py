from collections import namedtuple
from pathlib import Path

from typing import Tuple

from stack_of_tasks.tasks.hierarchy import TaskHierarchy


class InstanceData:
    def __init__(self) -> None:
        self._object_data: dict = None
        self._task_data: dict[int, list[SoTInstancingData]] = None

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
        self.solver_cls = None
        self.solver_parameter = {}

        self.actuator_cls = None
        self.actuator_parameter = {}

        self.params = {}


class Configuration:
    def __init__(self) -> None:
        self.parameter: Parameter = Parameter()
        self.instancing_data: InstanceData = InstanceData()

    @classmethod
    def from_data(cls, data: dict) -> "Configuration":
        c = Configuration()

        p = c.parameter
        i = c.instancing_data

        settings: dict = data.pop("settings")

        i._task_data = data.pop("stack_of_tasks")
        print(i._task_data)
        i._object_data = data

        solver_data = settings.pop("solver")
        actuator_data = settings.pop("actuator")

        p.solver_cls = solver_data["cls"]
        p.solver_parameter = solver_data.get("parameter", {})

        p.actuator_cls = actuator_data["cls"]
        p.actuator_parameter = actuator_data.get("parameter", {})

        p.params = settings

        return c

    def to_data(self) -> Tuple[dict, dict]:

        data = {
            "settings": (settings := {}),
            "stack_of_tasks": (sot := {}),
        }

        return data


from .yaml.loader import SoTInstancingData
