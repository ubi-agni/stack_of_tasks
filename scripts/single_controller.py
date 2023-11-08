#!/usr/bin/env python3
import numpy as np
from app import Application

import rospy

from stack_of_tasks.marker.trait_marker import FullMovementMarker
from stack_of_tasks.ref_frame.frames import Origin, RobotRefFrame
from stack_of_tasks.solver.OSQPSolver import OSQPSolver
from stack_of_tasks.tasks.Eq_Tasks import (
    DistanceTask,
    OrientationTask,
    ParallelTask,
    PlaneTask,
    PositionTask,
)
from stack_of_tasks.tasks.Task import TaskSoftnessType

np.set_printoptions(precision=3, suppress=True, linewidth=100, floatmode="fixed")


def setup(app: Application):
    marker = FullMovementMarker("marker")
    app.marker_server.add_marker(marker)

    marker_frame = Origin().translate(0.5, 0, 0.5)
    marker_frame.sync_trait("offset", marker, "transform")

    robot_frame = RobotRefFrame(app.controller.robot_state, "panda_hand_tcp")

    posTask = PositionTask(marker_frame, robot_frame, TaskSoftnessType.linear)
    oriTask = OrientationTask(marker_frame, robot_frame, TaskSoftnessType.linear, weight=1)

    planeTask = PlaneTask(marker_frame, robot_frame, TaskSoftnessType.linear)
    distTask = DistanceTask(
        marker_frame,
        robot_frame,
        softness_type=TaskSoftnessType.linear,
        weight=0.1,
        distance=0.2,
    )
    axis = np.array([0, 0, 1])
    axisTask = ParallelTask(
        marker_frame, robot_frame, TaskSoftnessType.linear, robot_axis=axis, target_axis=axis
    )

    with app.task_hierarchy.new_level() as level:
        level.append(posTask)
        level.append(axisTask)


if __name__ == "__main__":
    rospy.init_node("sot")
    app = Application(setup, OSQPSolver, True, rho=0.1)
    app.controller.control_loop(rospy.is_shutdown, 50, True)
