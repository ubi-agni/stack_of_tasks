#!/usr/bin/env python3
from __future__ import annotations

from enum import Enum
from functools import partial, wraps

from typing import Type

import numpy as np
import traits.api as ta

from stack_of_tasks.config.yaml import dump, load
from stack_of_tasks.ref_frame.frames import Origin
from stack_of_tasks.tasks.Eq_Tasks import PositionTask
from stack_of_tasks.tasks.Task import TaskSoftnessType
from stack_of_tasks.tasks.TaskHierarchy import TaskHierarchy


class DummyJacobian(Origin):
    J = ta.Array(transient=True)


if __name__ == "__main__":
    o = Origin()
    j = DummyJacobian()

    task_0 = PositionTask(o, j, TaskSoftnessType.quadratic)
    task_list = [task_0]

    toml = dump(task_list)
    print(toml)
    result = load(toml)
    print(result)
