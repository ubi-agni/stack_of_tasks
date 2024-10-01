#!/usr/bin/env python3
from __future__ import annotations

import re
from pathlib import Path

from typing import overload

import pybullet as p
import rospkg
from bullet_interface import BulletConnectionMode, ContactPoint, MoveitSyncedBulletScene
from collision_task import AvoidCollision_Eq1D, AvoidCollision_Eq3D, AvoidCollision_Ieq
from VarSizedOSQP import OSQPSolver

import rospy

from stack_of_tasks.config.config import Configuration
from stack_of_tasks.controller import Controller
from stack_of_tasks.marker.marker_server import MarkerServer
from stack_of_tasks.marker.trait_marker import PositionMarker
from stack_of_tasks.ref_frame import MarkerFrame, RobotRefFrame
from stack_of_tasks.robot_model.actuators import JointStatePublisherActuator
from stack_of_tasks.tasks.base import TaskSoftnessType
from stack_of_tasks.tasks.Eq_Tasks import PositionTask


def rewrite_urdf(urdf_string: str):
    """Helper method to replace all files with package-protocol with their absolut paths."""

    search = re.compile(r'"package:\/\/(.*?)\/(.*?)"')
    rpkg = rospkg.RosPack()

    def replace(match: re.Match):
        pkg = match.group(1)
        path = match.group(2)

        return f'"{ str(Path(rpkg.get_path(pkg)) / path) }"'

    return search.sub(replace, urdf_string)


lineIds = []
lineWidth = 3
colorRGB = [1, 0, 0]


from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


def main():
    rospy.init_node("mv")

    config = Configuration(actuator_cls=JointStatePublisherActuator, solver_cls=OSQPSolver)
    controller = Controller(config)

    urdf = rospy.get_param("robot_description")
    upath = Path("robot.urdf")
    upath.write_text(rewrite_urdf(urdf))

    b = MoveitSyncedBulletScene("planning_scene", BulletConnectionMode.GUI)

    rs = controller.robot_state

    robot = b.load_robot(str(upath.absolute()), rs)

    timer = rospy.Rate(30)

    ms = MarkerServer()
    ma = PositionMarker("pos", frame_id="panda_link0")

    ms.add_marker(ma)

    mf = MarkerFrame(ma)
    rf = RobotRefFrame(rs, "panda_link8")

    task = AvoidCollision_Ieq(robot, b, TaskSoftnessType.hard)
    ps = PositionTask(mf, rf, TaskSoftnessType.quadratic)

    with controller.task_hierarchy.new_level() as l:
        l.append(task)
        l.append(ps)

    warmstart = None
    while not rospy.is_shutdown():
        warmstart = controller.control_step(50, warmstart)

        timer.sleep()


if __name__ == "__main__":
    main()
