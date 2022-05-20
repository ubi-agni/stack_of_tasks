import typing

import numpy as np
from tf import transformations as tf


class TaskDesc:
    def __init__(self, A, upper, lower, name, is_hard=False) -> None:

        self.name = name
        self.is_hard = is_hard
        self.A = A
        self.upper = upper
        self.lower = lower
        self.size = self.lower.size

    def unpack(self):
        """returns A, upper, lower bounds"""
        return self.A, self.upper, self.lower


class EQTaskDesc(TaskDesc):
    def __init__(self, A, bound, name=None, is_hard=False) -> None:
        super().__init__(A, bound, bound, name, is_hard)


class Task:
    name: str
    args: typing.List

    def __init__(self, scale=1.0, hard_task=False) -> None:
        self._scale = scale
        self._hard = hard_task

        self.argmap = {a: a for a in self.args}

    def _map_args(self, data: dict):
        return tuple(map(data.get, self.argmap.values()))

    def compute(self, data):
        pass

    @staticmethod
    def skew(w):
        return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])


class TaskHirachy:
    def __init__(self) -> None:

        self.hirachy = []

    @property
    def higest_hiracy_level(self):
        if (d := len(self.hirachy)) == 0:
            return -1
        else:
            return d - 1

    def clear(self):
        self.hirachy = []

    def add_task_at(self, task: Task, prio: int):
        assert prio > -1
        assert prio <= self.higest_hiracy_level
        self.hirachy[prio].append(task)

    def add_task_lower(self, task: Task):
        self.hirachy.append([task])

    def add_task_same(self, task: Task):
        if (l := self.higest_hiracy_level) < 0:
            self.add_task_lower(task)
        else:
            self.hirachy[l].append(task)
    
    def remove_level(self, level):
        pass

    def remove_task(self, task):
        pass

    def compute(self, data):
        r = []

        for h in self.hirachy:
            r.append(list(map(lambda x: x.compute(data), h)))
        return r


### task instances


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

    def __init__(self, scale=1, hard_task=False) -> None:
        super().__init__(scale, hard_task)

        self.argmap = {"J": "J", "T_c": "T_c", "T_t": "T_t", "normal": "normal"}

    def compute(self, J, T_c, T_t, normal, speed):
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
    args = ["J", "T_c", "T_t", "angle"]

    def __init__(self, tgt_axis, robot_axis, scale=1.0) -> None:
        super().__init__(scale)
        self.ta = tgt_axis
        self.ra = robot_axis

    def compute(self, data):
        """Align axis in eef frame to lie in cone spanned by reference axis and opening angle acos(threshold)"""
        J, T_c, T_t, threshold = self._map_args(data)

        threshold = np.cos(threshold)
        axis = T_c[0:3, 0:3].dot(self.ra)  # transform axis from eef frame to base frame
        reference = T_t[0:3, 0:3].dot(self.ta)

        A = np.array([reference.T.dot(self.skew(axis)).dot(J[3:])])
        b = np.array([self._scale * (reference.T.dot(axis) - threshold)])

        return TaskDesc(A, b, np.array([-10e20]), self.name)


class JointPos(Task):
    name = "Joints"
    args = ["current_jp"]

    def __init__(self, desired_joint_pose, scale=1, hard_task=False) -> None:
        super().__init__(scale, hard_task)
        self.desired_joint_pose = desired_joint_pose

    def compute(self, data):
        (current_jp,) = self._map_args(data)

        A = np.identity(current_jp.size)
        b = -self._scale * (self.desired_joint_pose - current_jp)
        print(b)
        return EQTaskDesc(A, b, self.name)

class PreventJointBounds(Task):
    name = "JointsBound"
    args = ["current_jp"]

    def __init__(self, mins, maxs, scale=1, hard_task=False) -> None:
        super().__init__(scale, hard_task)
        self.mins = mins
        self.maxs = maxs

    def compute(self, data):
        (current_jp,) = self._map_args(data)

        A = np.identity(current_jp.size)
        b = -self._scale * (np.log( current_jp - self.mins )/(self.maxs - current_jp) - np.log(np.max(self.maxs - current_jp, 0)) / (current_jp - self.mins))

        return EQTaskDesc(A, b, self.name)

__available_task_classes__ = [PlaneTask, PositionTask, OrientationTask, JointPos, PreventJointBounds]
