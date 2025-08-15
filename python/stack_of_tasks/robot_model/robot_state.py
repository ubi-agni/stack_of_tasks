from typing import Dict, Tuple

import numpy
import traits.api as ta

import rospy

from stack_of_tasks.utils.traits import ABCSoTHasTraits
from stack_of_tasks.utils.transform_math import adjoint

from .robot_model import ActiveJoint, Joint, RobotModel


class RobotState(ABCSoTHasTraits):
    robot_model: RobotModel = ta.Instance(RobotModel)
    joint_values = ta.Array(comparison_mode=ta.ComparisonMode.none)

    def __init__(self, model: RobotModel) -> None:
        super().__init__(robot_model=model)

        # cache mapping joint -> (T, J)
        self._fk_cache: Dict[Joint, Tuple[numpy.ndarray, numpy.ndarray]] = {}

        # buffer for joint values received from sensors
        self.incoming_joint_values = 0.5 * (self.robot_model.mins + self.robot_model.maxs)
        self.joint_values = self.incoming_joint_values

    @ta.observe("joint_values")
    def _jv_change(self, evt):
        self.clear_cache()

    def update(self):
        self.joint_values = self.incoming_joint_values

    def clear_cache(self):
        self._fk_cache.clear()

    def _fk(self, joint):
        """Recursively compute forward kinematics and Jacobian (w.r.t. base) for joint"""
        if joint is None:
            return numpy.identity(4), numpy.zeros((6, self.robot_model.N))

        if joint in self._fk_cache:
            return self._fk_cache[joint]

        T, J = self._fk(joint.parent)

        if isinstance(joint, ActiveJoint):
            T = T @ joint.T @ joint.T_motion(self.joint_values[joint.idx])
            J[:, joint.idx] += adjoint(T) @ joint.twist
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
        return T, adjoint(T[:3, 3], inverse=True) @ J
