#!/usr/bin/env python

import random
import threading
from ast import Call

import numpy as np
import rospy
from geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from tf import transformations as tf
from tf2_ros import StaticTransformBroadcaster

from marker.interactive_marker import IAMarker
from robot_model import Joint, RobotModel
from solver.solver import Solver
from stack_of_tasks.tasks.TaskHierachy import TaskHierarchy
from stack_of_tasks.utils import Callback

# random.seed(1)


class Controller(object):
    def __init__(
        self,
        pose=TransformStamped(
            header=Header(frame_id="panda_link8"),
            child_frame_id="target",
            transform=Transform(
                rotation=Quaternion(*tf.quaternion_about_axis(np.pi / 4, [0, 0, 1])),
                translation=Vector3(0, 0, 0.105),
            ),
        ),
        rate=50,
    ):
        self.joint_state_callback = Callback()
        self.T_callback = Callback()
        self.J_callback = Callback()

        self.robot = RobotModel()
        self.robot._add(Joint(pose))  # add a fixed end-effector transform

        self.rate = rate

        self.joint_pub = rospy.Publisher(
            "/target_joint_states", JointState, queue_size=1, latch=True
        )

        # self.static_broadcaster = StaticTransformBroadcaster()
        # self.static_broadcaster.sendTransform(
        #    pose
        # )

        self.joint_msg = JointState()
        self.joint_msg.name = [j.name for j in self.robot.active_joints]
        self.target_link = pose.child_frame_id

        self.N = len(self.robot.active_joints)  # number of (active) joints

        self._T = np.zeros((4, 4))
        self._J = np.zeros(())
        self._joint_position = np.zeros(())

        self.joint_weights = np.ones(self.N)
        self.cartesian_weights = np.ones(6)

        self.mins = np.array([j.min for j in self.robot.active_joints])
        self.maxs = np.array([j.max for j in self.robot.active_joints])

        self.prismatic = np.array([j.jtype == j.prismatic for j in self.robot.active_joints])
        self.reset()

        self.task_hierarchy = TaskHierarchy()

        self.last_dq = None

    @property
    def T(self):
        return self._T

    @T.setter
    def T(self, val):
        self._T = val
        self.T_callback(self._T)

    @property
    def J(self):
        return self._J

    @J.setter
    def J(self, val):
        self._J = val
        self.J_callback(self._J)

    @property
    def joint_position(self):
        return self._joint_position

    @joint_position.setter
    def joint_position(self, val):
        self._joint_position = val
        self.joint_state_callback(self._joint_position)

    def reset(self, randomness=0):
        def _rJoint(m, M):
            j = (m + M) / 2
            dj = (M - m) / 2

            if randomness > 0:
                return j + dj * random.uniform(-randomness, randomness)
            return j

        self.joint_msg.position = list(map(lambda x: _rJoint(*x), zip(self.mins, self.maxs)))
        self.update()

    def update(self):
        self.joint_pub.publish(self.joint_msg)
        self.T, self.J = self.robot.fk(
            self.target_link, dict(zip(self.joint_msg.name, self.joint_msg.position))
        )
        self.joint_position = np.atleast_2d(self.joint_msg.position).T

    def actuate(self, q_delta):
        self.joint_msg.position += q_delta.ravel()
        # clip (prismatic) joints
        self.joint_msg.position[self.prismatic] = np.clip(
            self.joint_msg.position[self.prismatic],
            self.mins[self.prismatic],
            self.maxs[self.prismatic],
        )

        self.update()

    def check_end(self):
        pass

    def hierarchic_control(self, targets):
        lb = np.maximum(-0.01, (self.mins * 0.95 - self.joint_msg.position) / self.rate)
        ub = np.minimum(0.01, (self.maxs * 0.95 - self.joint_msg.position) / self.rate)

        tasks = self.task_hierarchy.compute(targets)

        solver = Solver(self.N, 0.015)
        dq, tcr = solver.solve_sot(tasks, lb, ub, warmstart=self.last_dq)

        self.last_dq = dq

        if dq is not None:
            self.actuate(dq)
            # return tcr[0] < 1e-12 or all(i < 1e-8 for i in tcr)

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


class ControlThread(threading.Thread):
    def __init__(self, func):
        super().__init__(daemon=True)

        self.dorun = False
        self.func = func

    def run(self) -> None:
        self.dorun = True

        while self.dorun:
            x = self.func()
            self.dorun = self.dorun & x

    def stop(self):
        self.dorun = False


if __name__ == "__main__":
    from tasks.Tasks import ConeTask, PositionTask, OrientationTask
    from marker.PositionOrientationMarker import SixDOFMarker

    rospy.init_node("ik")
    rate = rospy.Rate(50)

    def set_target(name, data):
        targets[name] = data

    targets = {}
    c = Controller()
    c.T_callback.append(lambda T: set_target("T", T))
    c.J_callback.append(lambda J: set_target("J", J))
    c.reset()

    mc = MarkerControl()
    mc.marker_data_callback.append(set_target)
    marker = SixDOFMarker(name="pose", scale=0.1)
    mc.add_marker(marker, marker.name)

    # setup tasks
    pos = PositionTask(1.0)
    pos.argmap["T_c"] = "T"
    pos.argmap["T_t"] = marker.name

    # cone = ConeTask((0, 0, 1), (0, 0, 1), 0.1)
    # cone.argmap["T_t"] = "Position"
    # cone.argmap["angle"] = "Cone_angle"

    ori = OrientationTask(1.0)
    ori.argmap["T_c"] = "T"
    ori.argmap["T_t"] = marker.name

    c.task_hierarchy.add_task_lower(pos)
    c.task_hierarchy.add_task_same(ori)

    def loop_function():
        # p = cProfile.Profile()
        # p.enable()
        rval = not c.hierarchic_control(targets)
        # p.disable()
        # p.print_stats()
        rate.sleep()
        return rval

    print("Stop with CRTL-D")
    while True:
        input("Start control with [ENTER]")
        t = ControlThread(loop_function)
        t.start()
        input("Stop control with [ENTER]")
        t.stop()
        t.join()
