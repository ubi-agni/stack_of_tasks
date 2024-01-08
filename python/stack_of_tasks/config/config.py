from collections import namedtuple
from pathlib import Path

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

                    print(l)

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


class Configuration:
    def __init__(self) -> None:
        self.parameter: Parameter = Parameter()
        self.instancing_data: InstanceData = InstanceData()

    @classmethod
    def from_data(cls, settings: dict, object_data: dict) -> "Configuration":
        c = Configuration()

        p = c.parameter
        i = c.instancing_data

        solver_data = settings["solver"]
        actuator_data = settings["actuator"]

        p.solver_cls = solver_data["cls"]
        p.solver_parameter = solver_data.get("parameter", {})

        p.actuator_cls = actuator_data["cls"]
        p.actuator_parameter = actuator_data.get("parameter", {})

        i._task_data = object_data.pop("stack_of_tasks")
        i._object_data = object_data

        return c


from .yaml.loader import SoTInstancingData
