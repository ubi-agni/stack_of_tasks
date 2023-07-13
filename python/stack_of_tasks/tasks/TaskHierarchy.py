from __future__ import annotations

from contextlib import contextmanager

from typing import Iterator, List, Set

import traits.api as ta

from stack_of_tasks.tasks.Task import Task
from stack_of_tasks.utils.traits import BaseSoTHasTraits


class TaskHierarchy(BaseSoTHasTraits):
    levels: List[Set[Task]] = ta.List(trait=ta.List(trait=ta.Instance(Task)), items=False)

    def __iter__(self) -> Iterator[Set[Task]]:
        """Iterate over all levels in the hierarchy"""
        yield from self.levels

    def __len__(self):
        return len(self.levels)

    def __getitem__(self, index):
        return self.levels[index]

    stack_changed = ta.Event()

    @ta.observe("levels:items.items")
    def _stack_changed(self, evt):
        self.stack_changed = evt

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

    # TODO more convenience functions
