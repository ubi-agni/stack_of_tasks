from __future__ import annotations

from typing import Optional

import numpy as np
import pybullet as pb

from stack_of_tasks.robot_model.robot_model import ActiveJoint, MimicJoint
from stack_of_tasks.robot_model.robot_state import RobotState

from .types import BulletJointInfoReturnVal


class BulletRobot:
    """The BulletRobot class keeps a pybullet robot in sync with the joint values given by a SoT-RobotState."""

    def __init__(self, bullet_robot_id: int, robot_state: RobotState) -> None:

        self.bullet_id: int = bullet_robot_id

        self._robot_state = robot_state
        self._robot_model = robot_state.robot_model

        self._robot_state.observe(lambda x: self.set_joint_values(x.new), "joint_values")

        self.njoints = pb.getNumJoints(self.bullet_id)

        self._robot_joints = {}

        self._joint_idxs = []
        self._joint_val_idxs = []

        self._lindex_to_name = {}

        self._mimic_joints = {}

        for i in range(self.njoints):
            joint_info = BulletJointInfoReturnVal(*pb.getJointInfo(self.bullet_id, i))
            self._robot_joints[joint_info.jointName] = joint_info
            self._lindex_to_name[i] = joint_info.linkName

        active_joint_count = 0
        for joint in self._robot_model.joints.values():
            if isinstance(joint, ActiveJoint):
                self._joint_idxs.append(self._robot_joints[joint.name].jointIndex)

                if isinstance(joint, MimicJoint):
                    self._mimic_joints[active_joint_count] = joint
                    self._joint_val_idxs.append(joint.base.idx)

                else:
                    self._joint_val_idxs.append(joint.idx)
                active_joint_count += 1

    def set_joint_values(self, joint_values):
        target = np.ndarray((len(self._joint_idxs), 1))
        target[:, 0] = joint_values[self._joint_val_idxs]

        for k, v in self._mimic_joints.items():
            target[k, 0] = target[k, 0] * v.multiplier + v.offset

        pb.resetJointStatesMultiDof(self.bullet_id, self._joint_idxs, targetValues=target)
