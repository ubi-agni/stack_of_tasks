#!/usr/bin/env python3


from contextlib import contextmanager

from typing import Callable, Iterator, List, Set, Type

import numpy as np
import traits.api as ta

import rospy

from stack_of_tasks.marker.marker_server import MarkerServer
from stack_of_tasks.marker.trait_marker import FullMovementMarker
from stack_of_tasks.ref_frame.frames import Origin, RobotRefFrame
from stack_of_tasks.robot_model.jointstate_publisher import JointStatePublisher
from stack_of_tasks.robot_model.robot_model import RobotModel
from stack_of_tasks.robot_model.robot_state import RobotState
from stack_of_tasks.solver.AbstractSolver import Solver
from stack_of_tasks.solver.OSQPSolver import OSQPSolver
from stack_of_tasks.tasks.Eq_Tasks import (
    DistanceTask,
    OrientationTask,
    ParallelTask,
    PlaneTask,
    PositionTask,
)
from stack_of_tasks.tasks.Task import Task, TaskSoftnessType


class TaskHierarchy(ta.HasTraits):
    levels: List[Set[Task]] = ta.List(trait=ta.Set(trait=ta.Instance(Task)), items=False)

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
    def new_level(self) -> Iterator[Set[Task]]:
        """Context manager to get a new level witch can be filled with tasks, before it is added to the stack.

        Yields:
            set: New Level, an empty set.
        """
        level: Set[Task] = set()
        yield level
        self.levels.append(level)

    # TODO more convenience functions


class Controller(ta.HasTraits):
    robot_model: RobotModel = ta.Instance(RobotModel)
    robot_state: RobotState = ta.Instance(RobotState)

    solver: Solver = ta.Instance(Solver)
    hierarchy: TaskHierarchy = ta.Instance(TaskHierarchy)

    def __init__(self, solver: Type[Solver], **solver_args) -> None:
        # robotmodel
        self.robot_model = RobotModel()

        # robotstate
        self.robot_state = RobotState(self.robot_model)

        super().__init__()

        # collection of tasks
        self.hierarchy = TaskHierarchy()
        self.solver = solver(self.robot_model.N, self.hierarchy, **solver_args)

    @ta.observe("hierarchy.stack_changed")
    def _stack_change(self, evt):
        self.solver.stack_changed()

    def control_loop(self, condition: Callable[[], bool], rate: int, invert_condition=False):
        _condition = lambda: not condition() if invert_condition else condition

        rrate = rospy.Rate(rate)

        warmstart_dq = None
        while _condition():
            warmstart_dq = self.control_step(rate, warmstart_dq)
            rrate.sleep()

    def control_step(self, rate, warmstart):
        lb = np.maximum(
            -0.01, (self.robot_model.mins * 0.95 - self.robot_state.joint_values) / rate
        )
        ub = np.minimum(
            0.01, (self.robot_model.maxs * 0.95 - self.robot_state.joint_values) / rate
        )

        dq = self.solver.solve(lb, ub, warmstart=warmstart)

        if dq is not None:
            self.robot_state.actuate(dq)
        else:
            print("dq is none")

        return dq


def main():
    rospy.init_node("ik")

    ms = MarkerServer()
    m = FullMovementMarker("Movement")
    ms.add_marker(m)

    c = Controller(OSQPSolver, rho=0.1)
    _ = JointStatePublisher(c.robot_state)

    marker_frame = Origin().translate(0.5, 0, 0.5)
    marker_frame.sync_trait("offset", m, "transform")

    robot_frame = RobotRefFrame(c.robot_state, "panda_hand_tcp")

    posTask = PositionTask(marker_frame, robot_frame, TaskSoftnessType.linear)
    oriTask = OrientationTask(marker_frame, robot_frame, TaskSoftnessType.linear, weight=1)

    planeTask = PlaneTask(marker_frame, robot_frame, TaskSoftnessType.linear)
    distTask = DistanceTask(
        marker_frame,
        robot_frame,
        softnessType=TaskSoftnessType.linear,
        weight=0.1,
        distance=0.2,
    )
    axis = np.array([0, 0, 1])
    paraTask = ParallelTask(
        marker_frame, robot_frame, TaskSoftnessType.linear, robot_axis=axis, target_axis=axis
    )

    with c.hierarchy.new_level() as level:
        # level.add(posTask)
        level.add(distTask)
        level.add(planeTask)
        level.add(paraTask)

    c.control_loop(rospy.is_shutdown, 50, True)


if __name__ == "__main__":
    main()
