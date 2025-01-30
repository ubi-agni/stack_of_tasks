from numpy.typing import NDArray

import tf.transformations as tf
from geometry_msgs.msg import Point, Pose, Quaternion


def pose_to_matrix(pose: Pose) -> NDArray:
    q = pose.orientation
    p = pose.position
    T = tf.quaternion_matrix([q.x, q.y, q.z, q.w])
    T[0:3, 3] = p.x, p.y, p.z
    return T


def matrix_to_pose(T: NDArray = None, p: NDArray = None, R: NDArray = None) -> Pose:

    if (T is None and p is None and R is None) or (T is not None and (p is not None or R is not None)):
        raise AttributeError()

    pose = Pose()
    if T is not None:
        pose.position = Point(*T[0:3, 3])
        pose.orientation = Quaternion(*tf.quaternion_from_matrix(T))
    else:
        if p is not None:
            pose.position = Point(*p)
        if R is not None:
            pose.orientation = Quaternion(*tf.quaternion_from_matrix(R))

    return pose
