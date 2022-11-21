from typing import Union

import numpy as np
from numpy.typing import NDArray

from tf import transformations as tf
from geometry_msgs.msg import Point, Pose, Quaternion


class Callback(list):
    def __call__(self, *args) -> None:
        for listener in self:
            listener(*args)


class OffsetTransform:
    def __init__(self, frame: str, offset: Union[NDArray, Pose] = np.eye(4)) -> None:
        self.frame = frame
        if isinstance(offset, Pose):
            self.offset = pose_to_matrix(offset)
        else:
            self.offset = offset


def create_pose(T: NDArray, frame_id=None) -> Pose:
    if T.shape != (4, 4):  # if not 4x4 matrix: assume position vector
        Tnew = np.identity(4)
        Tnew[0:3, 3] = T
        T = Tnew

    p = Pose(
        position=Point(*T[0:3, 3]),
        orientation=Quaternion(
            *tf.quaternion_from_matrix(T),
        ),
    )
    return p


def pose_to_matrix(pose: Pose) -> NDArray:
    q = pose.orientation
    p = pose.position
    T = tf.quaternion_matrix(np.array([q.x, q.y, q.z, q.w]))
    T[0:3, 3] = np.array([p.x, p.y, p.z])
    return T


def skew(w):
    return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])


def prettyprintMatrix(inmatrix):
    matrix = []
    maxL = 0
    for row in inmatrix:
        col = []
        for x in row:
            s = f"{x:.2e}"
            maxL = max(len(s), maxL)
            col.append(s)
        matrix.append(col)

    s = ""
    for r in matrix[:-1]:
        for x in r[:-1]:
            s += f"{x :^{maxL}}│"
        s += f"{r[-1] :^{maxL}}"
        s += "\n"
        s += ("─" * maxL + "┼") * (len(r) - 1) + "─" * maxL
        s += "\n"

    for r in matrix[-1][:-1]:
        s += f"{r :^{maxL}}│"
    s += f"{matrix[-1][-1] :^{maxL}}"

    return s
