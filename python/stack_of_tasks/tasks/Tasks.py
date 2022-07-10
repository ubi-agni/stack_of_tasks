from __future__ import print_function

import numpy as np
from tf import transformations as tf

from .Task import EQTaskDesc, Task, TaskDesc


class PositionTask(Task):
    name = "Pos"
    args = ["J", "T_c", "T_t"]

    def compute(self, data):
        J, T_c, T_t = self._map_args(data)
        A = J[:3]
        b = self._scale * (T_t[:3, 3] - T_c[:3, 3])
        return EQTaskDesc(A, b, self.name, self._hard)


class OrientationTask(Task):
    name = "Ori"
    args = ["J", "T_c", "T_t"]

    def compute(self, data):
        J, T_c, T_t = self._map_args(data)

        A = J[3:]
        delta = np.identity(4)
        delta[0:3, 0:3] = T_c[0:3, 0:3].T.dot(T_t[0:3, 0:3])
        angle, axis, _ = tf.rotation_from_matrix(delta)
        b = self._scale * (T_c[0:3, 0:3].dot(angle * axis))

        return EQTaskDesc(A, b, self.name)


class DisstanceTask(Task):
    """Keep distance to target position, not considering (approach) direction"""

    name = "Dist"
    args = ["J", "T_c", "T_t", "dist"]

    def compute(self, data):
        J, T_c, T_t, dist = self._map_args(data)

        delta = T_c[0:3, 3] - T_t[0:3, 3]
        A = np.array([delta.T.dot(J[:3])])
        b = np.array([-self._scale * (np.linalg.norm(delta) - dist)])
        return EQTaskDesc(A, b, self.name)


class ConstantSpeed(Task):
    name = "ConstantSpeed"
    args = ["J", "T_c", "T_t", "normal", "speed"]

    def compute(self, data):
        J, T_c, T_t, normal, speed = self._map_args(data)

        axis = T_c[0:3, 0:3].dot(normal)
        A = J[:3]

        r = T_t[0:3, 3] - T_c[0:3, 3]
        b = np.cross(axis, r)
        b = speed * b / np.linalg.norm(b)

        return EQTaskDesc(A, b, self.name)


class ParallelTask(Task):
    name = "Parallel"

    def __init__(self, tgt_axis, robot_axis, scale=1.0, hard_task=False) -> None:
        super().__init__(scale, hard_task)
        self.ta = tgt_axis
        self.ra = robot_axis

    def compute(self, J, T_c, T_t):
        """Align axis in eef frame to be parallel to reference axis in base frame"""
        axis = T_c[0:3, 0:3].dot(self.ra)  # transform axis from eef frame to base frame
        ref = T_t[0:3, 0:3].dot(self.ta)
        A = (self.skew(ref).dot(self.skew(axis))).dot(J[3:])
        b = self._scale * np.cross(ref, axis)
        return EQTaskDesc(A, b, self.name)


class PlaneTask(Task):
    name = "Plane"
    args = ["J", "T_c", "T_t"]

    def compute(self, data):
        """Move eef within plane given by normal vector and distance to origin"""
        J, T_c, T_t = self._map_args(data)

        normal = T_t[0:3, 2]
        dist = normal.dot(T_t[0:3, 3])

        A = np.array([normal.T.dot(J[:3])])
        b = np.array([-self._scale * (normal.dot(T_c[0:3, 3]) - dist)])

        return EQTaskDesc(A, b, self.name)


class LineTask(Task):
    name = "Line"

    def compute(self, J, T_c, T_t):
        normal = T_t[0:3, 2]
        sw = self.skew(normal)

        d = T_c[0:3, 3] - T_t[0:3, 3]

        A = sw.dot(J[:3])
        b = -self._scale * sw.dot(d)
        return EQTaskDesc(A, b, self.name)


class ConeTask(Task):
    name = "Cone"
    args = ["J", "T_c", "T_t", "angle", "target_axis", "robot_axis"]

    def compute(self, data):
        """Align axis in eef frame to lie in cone spanned by reference axis and opening angle acos(threshold)"""
        J, T_c, T_t, threshold, ta, ra = self._map_args(data)

        threshold = np.cos(threshold)
        axis = T_c[0:3, 0:3].dot(ra)  # transform axis from eef frame to base frame
        reference = T_t[0:3, 0:3].dot(ta)

        A = np.array([reference.T.dot(self.skew(axis)).dot(J[3:])])
        b = np.array([self._scale * (reference.T.dot(axis) - threshold)])

        return TaskDesc(A, b, np.array([-10e20]), self.name)


class JointPos(Task):
    name = "Joint Position"
    args = ["current_jp", "desired_joint_pose"]

    def compute(self, data):
        (current_jp, desired_joint_pose) = self._map_args(data)

        A = np.identity(current_jp.size)
        b = -self._scale * (desired_joint_pose - current_jp)
        return EQTaskDesc(A, b, self.name)


class PreventJointBounds(Task):
    name = "Prevent Joints Bound"
    args = ["current_jp", "mins", "maxs"]

    def compute(self, data):
        (current_jp, mins, maxs) = self._map_args(data)

        A = np.identity(current_jp.size)
        b = -self._scale * (
            np.log(current_jp - mins) / (maxs - current_jp)
            - np.log(np.max(maxs - current_jp, 0)) / (current_jp - mins)
        )

        return EQTaskDesc(A, b, self.name)
