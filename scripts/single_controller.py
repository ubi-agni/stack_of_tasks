#!/usr/bin/env python3
import numpy as np

import rospy

from stack_of_tasks.controller import Controller, MarkerControl
from stack_of_tasks.marker.markers import SixDOFMarker
from stack_of_tasks.ref_frame.frames import JointFrame
from stack_of_tasks.robot_model import JointStatePublisher
from stack_of_tasks.solver.InverseJacobianSolver import InverseJacobianSolver
from stack_of_tasks.solver.OSQPSolver import OSQPSolver
from stack_of_tasks.tasks import PositionTask
from stack_of_tasks.tasks.Task import TaskSoftnessType

np.set_printoptions(precision=3, suppress=True, linewidth=100, floatmode="fixed")


def setup(controller: Controller, mc):
    marker = SixDOFMarker(name="pose", scale=0.1)
    mc.add_marker(marker, marker.name)
    marker.ref_frame.translate(x=0.3, z=0.5)

    frame_hand = JointFrame(controller.robot_state, "panda_hand_tcp")
    pos = PositionTask(
        frame_hand,
        marker.ref_frame,
        TaskSoftnessType.linear,
        PositionTask.RelativeType.B_FIXED,
        weight=0,
    )

    controller.task_hierarchy.append_task(pos)
    controller.solver.stack_changed()


def main():
    rate = rospy.Rate(50)

    targets = {}

    def set_target(name, data):
        targets[name] = data

    mc = MarkerControl()
    mc.marker_data_callback.append(set_target)

    controller = Controller(solver_class=OSQPSolver, rho=0.01)
    _ = JointStatePublisher(controller.robot_state)
    setup(controller, mc)

    while not rospy.is_shutdown():
        controller.hierarchic_control(targets)
        rate.sleep()


if __name__ == "__main__":
    rospy.init_node("ik")

    main()
