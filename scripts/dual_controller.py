#!/usr/bin/env python3
import numpy as np
from app import Application

import rospy
import tf.transformations as tf

from stack_of_tasks.marker.trait_marker import FullMovementMarker
from stack_of_tasks.ref_frame.frames import Origin, RobotRefFrame
from stack_of_tasks.solver.OSQPSolver import OSQPSolver
from stack_of_tasks.tasks.Eq_Tasks import OrientationTask, PositionTask
from stack_of_tasks.tasks.Task import RelativeType, TaskSoftnessType

np.set_printoptions(precision=3, suppress=True, linewidth=100, floatmode="fixed")
from pprint import pprint


def setup(app: Application):
    marker = FullMovementMarker("marker")
    app.marker_server.add_marker(marker)

    marker_frame = Origin().translate(0, 0, 1.5)
    marker_frame.sync_trait("offset", marker, "transform")

    frame_left = RobotRefFrame(app.controller.robot_state, "left_hand_tcp")
    frame_right = RobotRefFrame(app.controller.robot_state, "right_hand_tcp")

    pos = PositionTask(frame_left, marker_frame, TaskSoftnessType.linear, weight=1.0)

    relative = PositionTask(
        frame_left.translate(z=0.1),
        frame_right,
        TaskSoftnessType.linear,
        weight=1.0,
        relType=RelativeType.A_FIXED,
    )

    orientation = OrientationTask(
        frame_left,
        frame_right.rotate_axis_angle(
            axis=[0, 0.5, 0],
            angle=1.5708 * 2,
        ),
        TaskSoftnessType.linear,
        weight=10,
        relType=RelativeType.RELATIVE,
    )

    with app.task_hierarchy.new_level() as level:
        level.add(pos)
        level.add(relative)

    with app.task_hierarchy.new_level() as level:
        #
        level.add(orientation)


if __name__ == "__main__":
    rospy.init_node("sot")
    app = Application(setup, OSQPSolver, True, rho=0.1)
    app.controller.control_loop(rospy.is_shutdown, 50, True)
