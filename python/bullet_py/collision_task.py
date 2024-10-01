from __future__ import annotations

from typing import Tuple

import numpy as np

run = True
from bullet_interface import BulletRobot, MoveitSyncedBulletScene
from scipy.linalg import norm

import rospy

from stack_of_tasks.robot_model.robot_state import RobotState
from stack_of_tasks.tasks.base import A, Bound, EqTask, IeqTask, Task, TaskSoftnessType, ta
from stack_of_tasks.utils.transform_math import adjoint, skew


def alignment_rotation(a):
    a = a / np.linalg.norm(a)
    if np.allclose(a[0], -1):
        return np.diag([-1, -1, 1])

    m = np.zeros((3, 3))
    m[0, 1:3] = a[1:3]
    m[1:3, 0] = -a[1:3]

    return np.eye(3) + m + m @ m / (1 + a[0])


class AvoidCollisionBase(Task):
    """A base class for all collision-avoidance tasks."""

    task_size: int = ta.Int(0)  # The task-size needs to be variable.

    dist_threshhold: float = ta.Float(0.1)

    def __init__(
        self,
        bullet_robot: BulletRobot,
        bullet_scene: MoveitSyncedBulletScene,
        softness_type: TaskSoftnessType,
        dist_threshhold: float = 0.1,
        weight: float = 1,
        **traits,
    ) -> None:
        super().__init__(softness_type, weight, dist_threshhold=dist_threshhold, **traits)

        self.robot = bullet_robot
        self.scene = bullet_scene

        self.robot._robot_state.observe(self._trigger_recompute, "joint_values")

    def _collect_twists(self):
        """This method calculates all twits of motions that repell the robot from near objects."""

        points = self.scene.collision(self.robot, self.dist_threshhold)
        # all minimal points between each link and the world.

        link_twists = {}

        for point in points:
            link_name = self.robot._lindex_to_name[point.linkIndexA]

            repeller = np.array(point.positionOnA) - np.array(point.positionOnB)
            # calculate the repelling direction and magnitude.

            repeller = repeller * (self.dist_threshhold / np.linalg.norm(repeller) - 1)
            # scale the repeller to correct length

            link_twists.setdefault(link_name, []).append(repeller)

        for k in link_twists.keys():
            link_twists[k] = np.mean(link_twists[k], 0)

        return link_twists


class AvoidCollision_Eq3D(AvoidCollisionBase, EqTask):
    def compute(self):
        twists = self._collect_twists()
        self.task_size = len(twists) * 3

        if self.task_size == 0:
            return np.zeros((0, 8)), np.zeros((0,)), np.zeros((0,))

        js = []
        b = []

        for k, v in twists.items():
            _, J = self.robot._robot_state.fk(k)

            js.append(J[:3])
            b.append(v)

        return np.vstack(js), np.concatenate(b)


class AvoidCollision_Eq1D(AvoidCollisionBase, EqTask):
    def compute(self):
        twists = self._collect_twists()
        self.task_size = len(twists)

        if self.task_size == 0:
            return np.zeros((0, 8)), np.zeros((0,)), np.zeros((0,))

        js = []
        b = []

        for k, v in twists.items():
            _, J = self.robot._robot_state.fk(k)
            R_align = alignment_rotation(v)

            J = adjoint(R_align) @ J

            js.append(J[0])
            b.append([norm(v)])

        return np.vstack(js), np.concatenate(b)


class AvoidCollision_Ieq(IeqTask, AvoidCollisionBase):
    def compute(self):
        twists = self._collect_twists()
        self.task_size = len(twists)

        if self.task_size == 0:
            return np.zeros((0, 8)), np.zeros((0,)), np.zeros((0,))

        js = []
        ls = []
        us = []

        for k, v in twists.items():
            _, J = self.robot._robot_state.fk(k)

            R_align = alignment_rotation(v)
            J = adjoint(R_align) @ J

            d = np.linalg.norm(v)

            ls.append([0])
            js.append(J[0])

            us.append([1e4])

        return np.vstack(js), np.concatenate(ls), np.concatenate(us)
