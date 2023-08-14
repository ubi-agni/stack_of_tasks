from __future__ import annotations

from contextlib import contextmanager
from enum import IntEnum, auto

from typing import Iterator, List

import traits.api as ta
from traits.observation.events import ListChangeEvent

from stack_of_tasks.tasks.Task import Task
from stack_of_tasks.utils.traits import BaseSoTHasTraits


class ChangeType(IntEnum):
    added = auto()
    removed = auto()


class StackChangeEvent:
    def __init__(self, type: ChangeType, index: int, tasks: List[Task]) -> None:
        self.type = type
        self.index = index
        self.tasks = tasks


class HierarchieChanged(StackChangeEvent):
    pass


class LevelChanged(StackChangeEvent):
    def __init__(self, type: ChangeType, index: int, tasks: List[Task], level: int) -> None:
        super().__init__(type, index, tasks)
        self.level = level


class TaskHierarchy(BaseSoTHasTraits):
    levels: List[List[Task]] = ta.List(trait=ta.List(trait=ta.Instance(Task)), items=False)

    def __iter__(self) -> Iterator[List[Task]]:
        """Iterate over all levels in the hierarchy"""
        yield from self.levels

    def __len__(self):
        return len(self.levels)

    def __getitem__(self, index):
        return self.levels[index]

    stack_changed = ta.Event()
    layout_changed = ta.Event()

    @ta.observe("levels:items.items")
    def _stack_changed(self, evt):
        self.stack_changed = evt

    @ta.observe("levels:items:items")
    def _level_changed(self, evt: ListChangeEvent):
        type, tasks = (
            (ChangeType.added, evt.added)
            if len(evt.added) > 0
            else (ChangeType.removed, evt.removed)
        )
        level = self.levels.index(evt.object)
        self.layout_changed = LevelChanged(type, evt.index, tasks, level)

    @ta.observe("levels:items")
    def _levels_changed(self, evt: ListChangeEvent):
        type, tasks = (
            (ChangeType.added, evt.added)
            if len(evt.added) > 0
            else (ChangeType.removed, evt.removed)
        )
        self.layout_changed = HierarchieChanged(type, evt.index, tasks)

    @contextmanager
    def new_level(self) -> Iterator[List[Task]]:
        """Context manager to get a new level witch can be filled with tasks,
        before it is added to the stack.

        Yields:
            set: New Level, an empty set.
        """
        level: List[Task] = list()
        yield level
        self.levels.append(level)

    def remove_task(self, task: Task):
        for l in self.levels:
            if task in l:
                if len(l) > 1:
                    l.remove(task)
                else:
                    self.levels.remove(l)

                break

    def remove_level(self, level: int):
        return self.levels.pop(level)
