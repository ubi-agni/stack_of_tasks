from constantly import Names
import numpy as np
from sympy import Eq
from tf import transformations as tf

class TaskDesc:
    def __init__(self, A, upper, lower, name=None) -> None:
        self.name = name
        self.A = A
        self.upper = upper
        self.lower = lower

    def unpack(self):
        return self. A, self.upper, self.lower

    def unpack_only_upper(self):
        A = np.r_[-self.A,self.A]
        b = np.r_[-self.lower, self.upper]
        return A, b, np.ones(b.size) * -1e20

class EQTaskDesc(TaskDesc):
    def __init__(self, A, bound, name=None) -> None:
        super().__init__(A, bound, bound, name)

class Task:
    def __init__(self, scale=1.0) -> None:
        self._scale = scale
    
    @staticmethod
    def skew(w):
        return np.array([[0, -w[2], w[1]],
                            [w[2], 0, -w[0]],
                            [-w[1], w[0], 0]])

class CombineTasks(Task):

    def compute(self, tasks):
        assert len(tasks) > 0
        names = [tasks[0].name]
        A, ub, lb = tasks.pop(0).unpack()
        
        for t in tasks:
            names.append(t.name)
            ai, ui, li = t.unpack()
            A = np.r_[A, ai]
            ub = np.r_[ub, ui]
            lb = np.r_[lb, li]
        return TaskDesc(A, ub, lb, ",".join(names))

class PositionTask(Task):
    name = 'Pos'
    def __init__(self, scale=1.0) -> None:
        super().__init__()

    def compute(self, J, T_t, T_c):
        A = J[:3]
        b = self._scale *  (T_t[:3,3]-T_c[:3,3])
        return EQTaskDesc(A, b, self.name)

class OrientationTask(Task):
    name = 'Ori'
    def __init__(self) -> None:
        super().__init__()
        self._scale = .1

    def compute(self, J, T_c, T_t):
        A = J[3:]
        delta = np.identity(4)
        delta[0:3, 0:3] = T_c[0:3, 0:3].T.dot(T_t[0:3, 0:3])
        angle, axis, _ = tf.rotation_from_matrix(delta)
        b =  self._scale*(T_c[0:3, 0:3].dot(angle * axis))

        return EQTaskDesc(A, b, self.name)



class DisstanceTask(Task):
    """Keep distance to target position, not considering (approach) direction"""
    name = "Dist"
    def compute(self, J, T_t, T_c, dist=0):
        delta = T_c[0:3, 3] - T_t[0:3, 3]
        A = delta.T.dot(J[:3])
        b = -self.scale * (np.linalg.norm(delta) - dist)
        return EQTaskDesc(A, b, self.name)

class ParallelTask(Task):
    name = "Parallel"
    def __init__(self, tgt_axis, robot_axis) -> None:
        super().__init__()
        self.ta = tgt_axis
        self.ra = robot_axis

    def compute(self, J, T_c,T_t):
        """Align axis in eef frame to be parallel to reference axis in base frame"""
        axis = T_c[0:3, 0:3].dot(self.ra)  # transform axis from eef frame to base frame
        ref = T_t[0:3, 0:3].dot(self.ta)
        A = (self.skew(ref).dot(self.skew(axis))).dot(J[3:])
        b = self._scale * np.cross(ref, axis)
        return EQTaskDesc(A, b, self.name)

class PlaneTask(Task):
    name = "Plane"
    def compute(self, J, T_t, T_c):
        """Move eef within plane given by normal vector and distance to origin"""
        normal = T_t[0:3, 2]
        dist = normal.dot(T_t[0:3, 3])

        A = np.array([ normal.T.dot(J[:3]) ])
        b = np.array([ -self._scale * (normal.dot(T_c[0:3, 3]) - dist)])

        return EQTaskDesc(A, b, self.name)

class ConeTask(Task):
    name = "Cone"
    def __init__(self, tgt_axis, robot_axis) -> None:
        super().__init__()
        self.ta = tgt_axis
        self.ra = robot_axis

    def compute(self,J, T_c, T_t, threshold, scale=1.0):
        """Align axis in eef frame to lie in cone spanned by reference axis and opening angle acos(threshold)"""
        axis = T_c[0:3, 0:3].dot(self.ra) # transform axis from eef frame to base frame
        reference = T_t[0:3, 0:3].dot(self.ta)

        A = np.array([reference.T.dot(self.skew(axis)).dot(J[3:])])
        b = np.array([self._scale * (reference.T.dot(axis) - threshold) ])

        return TaskDesc(A, b, -10e20, self.name)
    


