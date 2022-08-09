#!/usr/bin/env python3

import random
import threading

import numpy as np
import rospy
from geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from tf import transformations as tf

from stack_of_tasks.marker.interactive_marker import IAMarker
from stack_of_tasks.robot_model import JointType, RobotModel
from stack_of_tasks.solver.HQPSolver import HQPSolver
from stack_of_tasks.solver.InverseJacobianSolver import InverseJacobianSolver
from stack_of_tasks.tasks.TaskHierachy import TaskHierarchy
from stack_of_tasks.utils import Callback

# random.seed(1)


class Controller(object):
    def __init__(
        self,
        transform=tf.quaternion_matrix([0, 0, 0.382, 0.924]).dot(
            tf.translation_matrix([0, 0, 0.105])
        ),
        rate=50,
    ):
        self.joint_state_callback = Callback()
        self.T_callback = Callback()
        self.J_callback = Callback()

        self.target_offset = transform
        self.rate = rate

        self.target_link = "panda_joint8"

        self.robot = RobotModel()

        self.joint_pub = rospy.Publisher(
            "/target_joint_states", JointState, queue_size=1, latch=True
        )

        self.joint_msg = JointState()
        self.joint_msg.name = [j.name for j in self.robot.active_joints]

        self.N = len(self.robot.active_joints)  # number of (active) joints

        self._T = np.zeros((4, 4))
        self._J = np.zeros(())
        self._joint_position = np.zeros(())

        self.joint_weights = np.ones(self.N)
        self.cartesian_weights = np.ones(6)

        self.mins = np.array([j.min for j in self.robot.active_joints])
        self.maxs = np.array([j.max for j in self.robot.active_joints])

        self.prismatic = np.array(
            [j.jtype is JointType.prismatic for j in self.robot.active_joints]
        )
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

        T_all, J = self.robot.fk(
            self.target_link, dict(zip(self.joint_msg.name, self.joint_msg.position))
        )

        T = T_all[self.robot.active_joints[0].name]
        T = T.dot(self.target_offset)

        self.T = T
        self.J = J
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

        Solver = HQPSolver
        solver = Solver(self.N, rho=0.01)
        dq, tcr = solver.solve(tasks, lb, ub, warmstart=self.last_dq)

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


if __name__ == "__main__":
    from stack_of_tasks.marker.markers import SixDOFMarker
    from stack_of_tasks.tasks.Tasks import ConeTask, OrientationTask, PositionTask

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
    marker = SixDOFMarker(name="pose", scale=0.1, pose=targets["T"])
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

    while not rospy.is_shutdown():
        c.hierarchic_control(targets)
        rate.sleep()
