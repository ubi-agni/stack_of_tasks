from .Task import Task


class TaskHierarchy:
    def __init__(self) -> None:

        self.hierarchy = []

    @property
    def higest_hiracy_level(self):
        if (d := len(self.hierarchy)) == 0:
            return -1
        else:
            return d - 1

    def clear(self):
        self.hierarchy = []

    def add_task_at(self, task: Task, prio: int):
        self.hierarchy[prio].append(task)

    def add_task_lower(self, task: Task):
        self.hierarchy.append([task])

    def add_task_same(self, task: Task):
        if (l := self.higest_hiracy_level) < 0:
            self.add_task_lower(task)
        else:
            self.hierarchy[l].append(task)

    def remove_level(self, level):
        pass

    def remove_task(self, task):
        pass

    def compute(self, data):
        r = []

        for h in self.hierarchy:
            r.append(list(map(lambda x: x.compute(data), h)))
        return r