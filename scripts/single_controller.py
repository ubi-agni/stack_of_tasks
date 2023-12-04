#!/usr/bin/env python3
import numpy as np
from app import Application

import rospy

from stack_of_tasks.marker.trait_marker import FullMovementMarker
from stack_of_tasks.ref_frame import MarkerFrame
from stack_of_tasks.ref_frame.frames import RobotRefFrame
from stack_of_tasks.tasks import TaskSoftnessType
from stack_of_tasks.robot_model.actuators import VelocityCommandActuator
from stack_of_tasks.solver import CVXOPTSolver, InverseJacobianSolver, OSQPSolver
from stack_of_tasks.tasks.Eq_Tasks import (
    DistanceTask,
    OrientationTask,
    ParallelTask,
    PlaneTask,
    PositionTask,
)

np.set_printoptions(precision=3, suppress=True, linewidth=100, floatmode="fixed")


def setup(app: Application):
    rospy.sleep(0.1)  # wait for joint_states message
    app.controller.robot_state.update()

    eef = RobotRefFrame(app.controller.robot_state, "panda_hand_tcp")

    app.marker_server.add_marker(marker := FullMovementMarker(name="pose", transform=eef.T))
    marker = MarkerFrame(marker)

    posTask = PositionTask(marker, eef, TaskSoftnessType.linear)
    oriTask = OrientationTask(marker, eef, TaskSoftnessType.linear, weight=1)

    planeTask = PlaneTask(marker, eef, TaskSoftnessType.linear)
    distTask = DistanceTask(
        marker,
        eef,
        softness_type=TaskSoftnessType.linear,
        weight=0.1,
        distance=0.2,
    )
    axis = np.array([0, 0, 1])
    axisTask = ParallelTask(
        marker, eef, TaskSoftnessType.linear, robot_axis=axis, target_axis=axis
    )

    with app.task_hierarchy.new_level() as level:
        level.append(posTask)
        level.append(axisTask)


_stopping = False


def stopping():
    return _stopping


def stopController(actuator):
    global _stopping
    _stopping = True
    print("Stopping controller")
    actuator.stop(["joint_velocity_controller"])


if __name__ == "__main__":
    rospy.init_node("sot")

    app = Application(setup, OSQPSolver, False, rho=0.1)
    app.controller.robot_model.vmaxs *= 0.01
    rate = 100
    actuator = VelocityCommandActuator(app.controller, rate)
    app.controller.actuator = actuator
    actuator.switch_controllers(
        start=["joint_velocity_controller"], stop=["position_joint_trajectory_controller"]
    )
    rospy.on_shutdown(lambda: stopController(actuator))
    rospy.sleep(0.1)  # wait for joint states
    app.controller.control_loop(stopping, rate)
