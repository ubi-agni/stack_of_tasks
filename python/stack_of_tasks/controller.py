#!/usr/bin/env python3

import numpy as np

import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from tf import transformations as tf
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from stack_of_tasks.marker.interactive_marker import IAMarker
from stack_of_tasks.ref_frame.frames import JointFrame, World
from stack_of_tasks.ref_frame.offset import OffsetJointFrame, OffsetRefFrame
from stack_of_tasks.robot_model import RobotModel, adjoint
from stack_of_tasks.solver.AbstactSolver import Solver
from stack_of_tasks.solver.HQPSolver import HQPSolver
from stack_of_tasks.solver.InverseJacobianSolver import InverseJacobianSolver
from stack_of_tasks.tasks.TaskHierachy import TaskHierarchy
from stack_of_tasks.utils import Callback

# random.seed(1)


class Controller(object):
    def __init__(
        self,
        solver_class: Solver,
        rate=50,
        publish_joints=True,
        ns_prefix="",
        **solver_options,
    ):
        self.rate = rate

        self.joint_callback = Callback()
        self.control_step_callback = Callback()

        self.robot = RobotModel(param=ns_prefix + "robot_description")
        N = len(self.robot.active_joints)  # number of (active) joints

        self.task_hierarchy = TaskHierarchy(self)
        self.solver = solver_class(N, **solver_options)

        self._fk_cache = {}
        self._joint_position = np.zeros(N)
        self.mins = np.array([j.min for j in self.robot.active_joints])
        self.maxs = np.array([j.max for j in self.robot.active_joints])

        # fetch initial joint states from rosparams
        self.initial_joints = rospy.get_param(
            ns_prefix + "initial_joints", default=0.5 * (self.mins + self.maxs)
        )
        if isinstance(self.initial_joints, dict):
            names = [j.name for j in self.robot.active_joints]
            for name, value in self.initial_joints.items():
                try:
                    self._joint_position[names.index(name)] = value
                except:
                    pass
            self.initial_joints = self._joint_position

        # configure publishing joint states
        if publish_joints:
            joint_msg = JointState()
            joint_msg.name = [j.name for j in self.robot.active_joints]
            joint_pub = rospy.Publisher(
                ns_prefix + "target_joint_states", JointState, queue_size=1, latch=True
            )

            def _send_joint(joint_values):
                joint_msg.position = joint_values
                joint_pub.publish(joint_msg)

            self.joint_callback.append(lambda: _send_joint(self._joint_position))

        self.last_dq = None
        self.reset()  # trigger initialization callbacks

    @property
    def joint_position(self):
        return self._joint_position

    @joint_position.setter
    def joint_position(self, values):
        self._joint_position = values
        self._fk_cache = {}  # invalidate FK cache
        self.joint_callback()

    def fk(self, target: str):
        cached = self._fk_cache.get(target)
        if cached is not None:
            return cached
        result = self.robot.fk(target, self._joint_position)
        self._fk_cache[target] = result
        return result

    def reset(self, randomness=0):
        # center = 0.5 * (self.maxs + self.mins)
        center = self.initial_joints
        width = 0.5 * (self.maxs - self.mins) * randomness
        self.joint_position = center + width * (np.random.random_sample(width.shape) - 0.5)

    def actuate(self, q_delta):
        self.joint_position += q_delta.ravel()

    def check_end(self):
        pass

    def hierarchic_control(self, targets):
        lb = np.maximum(-0.01, (self.mins * 0.95 - self._joint_position) / self.rate)
        ub = np.minimum(0.01, (self.maxs * 0.95 - self._joint_position) / self.rate)

        self.task_hierarchy.compute(targets)

        dq, tcr = self.solver.solve(
            self.task_hierarchy.hierarchy, lb, ub, warmstart=self.last_dq
        )

        self.last_dq = dq

        if dq is not None:
            self.actuate(dq)
            # return tcr[0] < 1e-12 or all(i < 1e-8 for i in tcr)

        self.control_step_callback()
        return False


class MarkerControl:
    def __init__(self) -> None:
        self.ims = InteractiveMarkerServer("controller")
        self.marker = {}
        self.marker_data_callback = Callback()

    def add_marker(self, marker: IAMarker, name):
        self.marker[name] = marker
        marker.data_callbacks.append(self.marker_data_callback)
        marker.init_server(self.ims)

    def delete_marker(self, name):
        self.marker[name].delete()
        # t = self.marker[name].provided_targets()
        # for x in t:
        #    print(x)
        #    del self.targets[x]
        del self.marker[name]


if __name__ == "__main__":

    from stack_of_tasks.marker.interactive_marker import ConeMarker
    from stack_of_tasks.marker.markers import SixDOFMarker
    from stack_of_tasks.tasks.Eq_Tasks import (  # JointPos,
        DistanceTask,
        OrientationTask,
        ParallelTask,
        PlaneTask,
        PositionTask,
    )
    from stack_of_tasks.tasks.Ieq_Tasks import ConeTask
    from stack_of_tasks.tasks.Task import TaskSoftnessType

    # from stack_of_tasks.tasks.Ieq_Tasks import ConeTask

    np.set_printoptions(precision=3, suppress=True, linewidth=100, floatmode="fixed")

    rospy.init_node("ik")
    rate = rospy.Rate(50)

    targets = {}

    def set_target(name, data):
        targets[name] = data

    mc = MarkerControl()
    mc.marker_data_callback.append(set_target)

    # define marker relative to left_hand_tcp with an offset

    c = Controller(solver_class=HQPSolver, rho=0.01)

    marker = SixDOFMarker(name="pose", scale=0.1)
    mc.add_marker(marker, marker.name)
    marker.ref_frame.translate(x=0.3, z=0.5)

    # Task Setup
    frame_left = JointFrame(c, "panda_hand_tcp")
    pos = PositionTask(
        frame_left,
        marker.ref_frame,
        TaskSoftnessType.linear,
        PositionTask.RelativeType.B_FIXED,
        weight=0,
    )

    c.task_hierarchy.add_task_lower(pos)

    # pp = PlotPublisher()
    # pp.add_plot("q", [f"q/{joint.name}" for joint in c.robot.active_joints])
    # pp.add_plot("dq", [f"dq/{joint.name}" for joint in c.robot.active_joints])
    # c.joint_state_callback.append(lambda q: pp.plot("q", q))
    # c.delta_q_callback.append(lambda dq: pp.plot("dq", dq))

    targets["angle"] = 0.1

    targets["desired_joint_pose"] = c.initial_joints
    while not rospy.is_shutdown():

        c.hierarchic_control(targets)

        rate.sleep()
