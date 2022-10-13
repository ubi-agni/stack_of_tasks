from .Task import Task


class TaskHierarchy:
    def __init__(self, controller) -> None:

        self.controller = controller
        self.hierarchy = []

    @property
    def depth(self):
        return len(self.hierarchy) - 1

    def clear(self):
        self.hierarchy = []

    def add_task_at(self, task: Task, prio: int):
        task.controller = self.controller
        self.hierarchy[prio].append(task)

    def add_task_lower(self, task: Task):
        task.controller = self.controller
        self.hierarchy.append([task])

    def add_task_same(self, task: Task):
        if (l := self.depth) < 0:
            self.add_task_lower(task)
        else:
            self.add_task_at(task, l)

    def remove_level(self, level):
        pass

    def remove_task(self, task):
        pass

    def compute(self, data):
        for level in self.hierarchy:
            for task in level:
                task.compute(data)