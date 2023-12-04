from __future__ import annotations

from contextlib import contextmanager

from typing import Iterator, List

import traits.api as ta

from stack_of_tasks.tasks import Task
from stack_of_tasks.utils.traits import BaseSoTHasTraits


class TaskHierarchy(BaseSoTHasTraits):
    levels = ta.List(trait=ta.List(trait=ta.Instance(Task), items=False), items=False)

    def __iter__(self) -> Iterator[List[Task]]:
        """Iterate over all levels in the hierarchy"""
        yield from self.levels

    def __len__(self):
        return len(self.levels)

    def __getitem__(self, index):
        return self.levels[index]

    def all_tasks(self):
        """Iterate over all tasks in the hierarchy"""
        for level in self.levels:
            yield from level

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
            try:
                l.remove(task)
                if len(l) == 0:
                    self.levels.remove(l)
                break

            except ValueError:
                continue
