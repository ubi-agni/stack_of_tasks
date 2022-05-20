#!/usr/bin/env python

import random
import threading

import numpy
import rospy
from geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from interactive_markers.interactive_marker_server import \
    InteractiveMarkerServer
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from tf import transformations as tf

from markers import *
from robot_model import Joint, RobotModel
from solver import Solver
from task import *

# random.seed(1)


class Controller(object):
    def __init__(
        self,
        pose=TransformStamped(
            header=Header(frame_id="panda_link8"),
            child_frame_id="target",
            transform=Transform(
                rotation=Quaternion(*tf.quaternion_about_axis(numpy.pi / 4, [0, 0, 1])),
                translation=Vector3(0, 0, 0.105),
            ),
        ),
        rate = 50
    ):

        self.robot = RobotModel()
        self.robot._add(Joint(pose))  # add a fixed end-effector transform
        self.rate = rate
        self.joint_state_callback = []

        self.joint_pub = rospy.Publisher(
            "/target_joint_states", JointState, queue_size=1, latch=True
        )

        self.joint_msg = JointState()
        self.joint_msg.name = [j.name for j in self.robot.active_joints]
        self.target_link = pose.child_frame_id
        self.N = len(self.robot.active_joints)  # number of (active) joints

        self.joint_weights = numpy.ones(self.N)
        self.cartesian_weights = numpy.ones(6)

        self.mins = numpy.array([j.min for j in self.robot.active_joints])
        self.maxs = numpy.array([j.max for j in self.robot.active_joints])

        self.prismatic = numpy.array(
            [j.jtype == j.prismatic for j in self.robot.active_joints]
        )
        self.reset()

        self.task_hirachy = TaskHirachy()

        self.last_dq = None
        self.joint_state_callback = []

    def reset(self, randomness=0):
        def _rJoint(m, M):
            j = (m + M) / 2
            dj = (M - m) / 2

            if randomness > 0:
                return j + dj * random.uniform(-randomness, randomness)
            return j

        self.joint_msg.position = list(
            map(lambda x: _rJoint(*x), zip(self.mins, self.maxs))
        )
        self.update()

    def update(self):
        self.joint_pub.publish(self.joint_msg)
        self.T, self.J = self.robot.fk(
            self.target_link, dict(zip(self.joint_msg.name, self.joint_msg.position))
        )

        for x in self.joint_state_callback:
            x(self.joint_msg.position)

    def actuate(self, q_delta):
        self.joint_msg.position += q_delta.ravel()
        # clip (prismatic) joints
        self.joint_msg.position[self.prismatic] = numpy.clip(
            self.joint_msg.position[self.prismatic],
            self.mins[self.prismatic],
            self.maxs[self.prismatic],
        )

        self.update()

    def check_end(self):
        pass

    def hierarchic_control(self, targets):
        lb = np.maximum(-0.01, (self.mins*.95 - self.joint_msg.position) / self.rate)
        ub = np.minimum(0.01, (self.maxs*.95 - self.joint_msg.position) / self.rate)

        targets["J"] = self.J
        targets["T_c"] = self.T
        targets["current_jp"] = np.array(self.joint_msg.position)

        tasks = self.task_hirachy.compute(targets)

        solver = Solver(self.N, 0.015)
        q_delta, tcr = solver.solve_sot(tasks, lb, ub, warmstart=self.last_dq)

        self.last_dq = q_delta

        if q_delta is not None:
            self.actuate(q_delta)
            # return tcr[0] < 1e-12 or all(i < 1e-8 for i in tcr)

        return False


class MarkerControl:
    def __init__(self) -> None:
        
        self.targets = {}
        self.ims = InteractiveMarkerServer("controller")
        self.marker={}

        self.set_data_callback = []

    def add_marker(self, marker:IAMarker, name):
        self.marker[name] = marker
        marker.data_callbacks.append(self.setTarget)
        marker.init_server(self.ims)

    def delete_marker(self, name):
        self.marker[name].delete()
        t = self.marker[name].provided_targets()
        for x in t:
            print(x)
            del self.targets[x]
        del self.marker[name]

    def setTarget(self, name, goal):
        self.targets[name] = goal
        for callback in self.set_data_callback:
            callback(name, goal)


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
    rospy.init_node("ik")
    rate = rospy.Rate(50)

    c = Controller()
    mc = MarkerControl()

    # setup tasks
    pos = PositionTask(1)
    pos.argmap["T_t"] = "Position"

    cone = ConeTask((0, 0, 1), (0, 0, 1), 0.1)
    cone.argmap["T_t"] = "Position"
    cone.argmap["angle"] = "Cone_angle"

    # ori = OrientationTask(3)
    # ori.argmap['T_t'] = 'Position'

    c.task_hirachy.add_task_lower(pos)
    c.task_hirachy.add_task_same(cone)

    #
    def loop_function():
        # p = cProfile.Profile()
        # p.enable()
        rval = not c.hierarchic_control(mc.targets)
        # p.disable()
        # p.print_stats()
        rate.sleep()
        return rval

    #

    print("Stop with CRTL-D")
    while True:
        input("Start control with [ENTER]")
        t = ControlThread(loop_function)
        t.start()
        input("Stop control with [ENTER]")
        t.stop()
        t.join()
