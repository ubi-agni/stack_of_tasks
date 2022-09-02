from socket import if_nameindex

import numpy
from numpy.typing import NDArray

from tf import transformations as tf
from geometry_msgs.msg import Point, Pose, Quaternion


def create_pose(T: NDArray) -> Pose:
    if T.shape != (4, 4):  # if not 4x4 matrix: assume position vector
        Tnew = numpy.identity(4)
        Tnew[0:3, 3] = T
        T = Tnew

    return Pose(
        position=Point(*T[0:3, 3]),
        orientation=Quaternion(*tf.quaternion_from_matrix(T)),
    )


def pose_to_matrix(pose: Pose) -> NDArray:
    q = pose.orientation
    p = pose.position
    T = tf.quaternion_matrix(numpy.array([q.x, q.y, q.z, q.w]))
    T[0:3, 3] = numpy.array([p.x, p.y, p.z])
    return T
