#!/usr/bin/env python3
import numpy as np
import traits.api as ta

from stack_of_tasks.ref_frame.frames import Origin, RefFrame
from stack_of_tasks.ref_frame.offset import Offset
from stack_of_tasks.tasks.Eq_Tasks import PositionTask
from stack_of_tasks.tasks.Task import TaskSoftnessType


class DummyJacobian(RefFrame):
    T = ta.Array(value=np.identity(4))
    J = ta.Array(value=np.identity(5))


r1 = Origin()
r2 = Origin()

j1 = DummyJacobian()
j2 = DummyJacobian()

o1 = Offset(j1)

x = PositionTask(refA=o1, refB=j2, softnessType=TaskSoftnessType.linear)
x.observe(print, "_recompute")

# set refA -> trigger recompute -> triggers twice! no no
x.refA = j1

# set refA.J -> trigger recompute
j1.J = np.random.random((4, 4))
