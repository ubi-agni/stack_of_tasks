from __future__ import annotations

from typing import Any, Dict, List, MutableSequence

from .Task import Task


class TaskItem:
    def __init__(self, task: Task, parent: TaskLevel) -> None:
        self.parent: TaskLevel = parent
        self.task: Task = task
        self.additional_data: Dict[str, Any] = {"name": ""}

    @property
    def row(self) -> int:
        return self.parent.proxy.index(self)


class LevelProxy(MutableSequence[TaskItem]):
    def __init__(self, level: TaskLevel) -> None:
        super().__init__()
        self._proxy: TaskLevel = level
        self._plist = level._list

    def __getitem__(self, idx: int) -> TaskItem:
        return self._plist[idx]

    def __setitem__(self, idx: int, value: TaskItem):
        value.parent = self._proxy
        self._plist[idx] = value

    def __delitem__(self, idx: int):
        del self._plist[idx]

    def insert(self, index: int, value: TaskItem) -> None:
        value.parent = self._proxy
        return self._plist.insert(index, value)

    def __len__(self) -> int:
        return len(self._plist)


class TaskLevel(MutableSequence[Task]):
    def __init__(self, parent: TaskHierarchy) -> None:
        super().__init__()

        self.parent = parent
        self._list: List[TaskItem] = []
        self.proxy = LevelProxy(self)

    @property
    def row(self) -> int:
        return self.parent.index(self)

    def __getitem__(self, idx: int) -> Task:
        return self._list[idx].task

    def __setitem__(self, idx: int, value: Task):
        self._list[idx].task = value

    def __delitem__(self, idx: int):
        del self._list[idx]

    def __len__(self) -> int:
        return len(self._list)

    def insert(self, index: int, value: Task):
        self._list.insert(index, TaskItem(value, self))


class TaskHierarchy(MutableSequence[TaskLevel]):
    def __init__(self) -> None:
        super().__init__()
        self._stack: List[TaskLevel] = []

    def __getitem__(self, idx: int) -> TaskLevel:
        return self._stack[idx]

    def __setitem__(self, idx: int, value: TaskLevel):
        self._stack[idx] = value

    def __delitem__(self, idx: int):
        del self._stack[idx]

    def __len__(self) -> int:
        return len(self._stack)

    def insert(self, index: int, value: TaskLevel):
        self._stack.insert(index, value)

    depth = __len__

    def append_task(self, value: Task | TaskItem):
        l = TaskLevel(self)
        if isinstance(value, Task):
            l.append(value)
        else:
            l.proxy.append(value)
        self.append(l)

    def insert_task(self, idx: int, value: Task | TaskItem):
        l = TaskLevel(self)
        if isinstance(value, Task):
            l.append(value)
        else:
            l.proxy.append(value)
        self.insert(idx, l)

    def compute(self, data):
        for l in self:
            for t in l:
                t.compute(data)
