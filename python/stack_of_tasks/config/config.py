from stack_of_tasks.tasks.hierarchy import TaskHierarchy


class Configuration:
    def __init__(self, *args) -> None:
        if len(args) == 2:  # instantiation from actuator and solver classes
            self.settings = {k: SoTInstancingData(v, {}) for k, v in zip(["solver", "actuator"], args)}
            self._sot = TaskHierarchy()
            self._objects = {"frames": [], "marker": []}
        else:  # instantiation from yaml dictionary
            data = args[0]

            self.settings = data.pop("settings")
            self._sot = None
            self._task_data = data.pop("stack_of_tasks")
            # remaining data are objects
            self._objects = data

    # We postpone the instantiation of stack_of_tasks and objects members
    # into properties to allow for dependency injection

    @property
    def stack_of_tasks(self):
        if self._sot is None:
            self._sot = TaskHierarchy()
            for tasks in self._task_data.values():
                with self._sot.new_level() as level:
                    level.extend([task.instance for task in tasks])

        return self._sot

    @property
    def objects(self):
        return {k: [o.instance for o in v] for k, v in self._objects.items()}


from .yaml.loader import SoTInstancingData
