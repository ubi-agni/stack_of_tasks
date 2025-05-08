#!/usr/bin/env python3
import numpy as np
from app import Application, Parameter

import rospy

from stack_of_tasks.marker.trait_marker import FullMovementMarker, IAMarker
from stack_of_tasks.ref_frame import MarkerFrame
from stack_of_tasks.ref_frame.frames import RobotRefFrame
from stack_of_tasks.robot_model.actuators import JointStatePublisherActuator
from stack_of_tasks.solver import OSQPSolver
from stack_of_tasks.tasks import TaskSoftnessType
from stack_of_tasks.tasks.Eq_Tasks import (
    DistanceTask,
    OrientationTask,
    ParallelTask,
    PlaneTask,
    PositionTask,
)

np.set_printoptions(precision=3, suppress=True, linewidth=100, floatmode="fixed")


class DummyConfig(Parameter):
    def __init__(self) -> None:
        super().__init__()

        self.solver_cls = OSQPSolver
        self.solver_parameter.update()
        self.actuator_cls = JointStatePublisherActuator


def setup(app: Application):
    rospy.sleep(0.1)  # wait for joint_states message
    app.controller.robot_state.update()

    eef = RobotRefFrame(link_name="panda_hand_tcp")

    IAMarker._default_frame_id = app.controller.robot_model.root_link
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
    axisTask = ParallelTask(marker, eef, TaskSoftnessType.linear)

    with app.task_hierarchy.new_level() as level:
        level.append(posTask)
        # level.append(axisTask)


if __name__ == "__main__":
    rospy.init_node("sot")

    config = DummyConfig()

    app = Application(config, setup)
    app.controller.control_loop(rospy.is_shutdown, 50)
