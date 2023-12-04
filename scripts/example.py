#!/usr/bin/env python3
import numpy

import rospy
import tf.transformations as tfs

from stack_of_tasks.controller import Controller
from stack_of_tasks.marker.marker_server import MarkerServer
from stack_of_tasks.marker.trait_marker import FullMovementMarker
from stack_of_tasks.ref_frame import MarkerFrame, Offset, Origin, RobotRefFrame
from stack_of_tasks.robot_model.actuators import VelocityCommandActuator
from stack_of_tasks.solver import CVXOPTSolver, InverseJacobianSolver
from stack_of_tasks.tasks.Eq_Tasks import OrientationTask, PositionTask
from stack_of_tasks.tasks.Task import TaskSoftnessType

rospy.init_node("example_controller")

rate = 100
c = Controller(InverseJacobianSolver, publish_joints=False)
c.robot_model.vmaxs *= 0.1  # 10% of max speed
c.actuator = VelocityCommandActuator(c, rate=rate)
c.actuator.switch_controllers(
    start=["joint_velocity_controller"], stop=["position_joint_trajectory_controller"]
)
rospy.sleep(0.1)  # wait for joint states
c.robot_state.update()

eef_frame = RobotRefFrame(c.robot_state, "panda_hand_tcp")
if True:  # Use an interactive marker for the goal frame
    marker = FullMovementMarker("pose", transform=eef_frame.T)
    ms = MarkerServer()
    ms.add_marker(marker)
    goal_frame = MarkerFrame(marker)
else:  # Use a fixed goal frame
    goal_frame = Offset(Origin(), tfs.translation_matrix([0, -0.5, 0.5]))


posTask = PositionTask(goal_frame, eef_frame, TaskSoftnessType.linear)
oriTask = OrientationTask(goal_frame, eef_frame, TaskSoftnessType.linear)
with c.task_hierarchy.new_level() as level:
    level.append(posTask)
    level.append(oriTask)


def converged(threshold):
    return numpy.max(numpy.abs(c.actuator.last_residuals)) < threshold


# run control loop until ROS shutdown (Ctrl+C)
c.control_loop(rospy.is_shutdown, rate=rate)
exit(0)

while True:
    c.control_loop(
        lambda: (rospy.is_shutdown() or converged(5e-4)),
        rate=rate,
    )
    if rospy.is_shutdown():
        break
    input("Press enter to start control loop")
