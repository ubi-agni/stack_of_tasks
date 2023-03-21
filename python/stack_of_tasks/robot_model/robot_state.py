from typing import Dict, Tuple

import numpy
import traits.api as ta

import rospy
from sensor_msgs.msg import JointState

from stack_of_tasks.utils.transform_math import adjoint

from .robot_model import ActiveJoint, Joint, RobotModel


class RobotState(ta.HasTraits):
    robot_model: RobotModel = ta.Instance(RobotModel)

    joint_state_msg: JointState = ta.Instance(JointState)

    joint_values = ta.ListFloat()

    def __init__(
        self, model: RobotModel, ns_prefix: str = "", init_joint_values=None
    ) -> None:
        super().__init__(robot_model=model, joint_state_msg=JointState(), joint_values=[])
        self.joint_state_msg.name = [j.name for j in self.robot_model.active_joints]

        # cache mapping joint -> (T, J)
        self._fk_cache: Dict[Joint, Tuple[numpy.ndarray, numpy.ndarray]] = {}

        if init_joint_values is not None:
            self.init_values = init_joint_values
        elif rospy.has_param(ns_prefix + "initial_joints"):
            self.joint_state_msg.position = numpy.empty((self.robot_model.N))
            init_pos = rospy.get_param(ns_prefix + "initial_joints")

            if isinstance(init_pos, dict):
                self.init_values = [
                    init_pos[joint.name] for joint in self.robot_model.active_joints
                ]
            else:
                self.init_values = init_pos
        else:
            self.init_values = 0.5 * (self.robot_model.mins + self.robot_model.maxs)

        self.joint_values = self.init_values

    @ta.observe("joint_values, joint_values.items")
    def _jv_change(self, evt):
        self.joint_state_msg.position = self.joint_values
        self.clear_cache()

    def reset(self):
        self.joint_values = self.init_values

    def clear_cache(self):
        self._fk_cache.clear()

    def set_random_joints(self, randomness=0):
        width = 0.5 * (self.robot_model.maxs - self.robot_model.mins) * randomness
        self.joint_values = self.init_values + width * (
            numpy.random.random_sample(width.shape) - 0.5
        )

    def _fk(self, joint):
        """Recursively compute forward kinematics and Jacobian (w.r.t. base) for joint"""
        if joint is None:
            return numpy.identity(4), numpy.zeros((6, self.robot_model.N))

        if joint in self._fk_cache:
            return self._fk_cache[joint]

        T, J = self._fk(joint.parent)

        if isinstance(joint, ActiveJoint):
            T = T @ joint.T @ joint.T_motion(self.joint_values[joint.idx])
            J[:, joint.idx] += adjoint(T).dot(joint.twist)
        else:
            T = T @ joint.T

        self._fk_cache[joint] = result = T, J
        return result

    def fk(self, link: str):
        """
        Compute FK and Jacobian for given link frame
        Jacobian uses standard robotics frame:
        - orientation: base frame
        - origin: link frame
        """
        T, J = self._fk(self.robot_model.links[link])
        # shift reference point of J into origin of frame T
        return T, adjoint(T[:3, 3], inverse=True).dot(J)
