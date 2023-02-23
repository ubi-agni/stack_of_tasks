#!/usr/bin/env python3

import math

import numpy as np

import rospy

from stack_of_tasks.marker.marker_server import MarkerServer
from stack_of_tasks.marker.trait_marker import ConeMarker, OrientationMarker, PositionMarker


def main():
    T = np.identity(4)
    T[0, 3] = 0

    server = MarkerServer()

    p = PositionMarker("pos")
    p.transform = T
    server.add_marker(p)

    o = OrientationMarker("ori")
    T[0, 3] = 1
    o.transform = T
    server.add_marker(o)

    c = ConeMarker("cone")
    T[0, 3] = 2
    c.transform = T
    server.add_marker(c)

    i = 0.0

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        s = (math.sin(i) + 1) / 2

        i += 0.1
        t = p.transform.copy()
        t[0, 3] = s * 0.4 - 0.2

        p.transform = t

        c.angle = 0.1 + s / 2
        rate.sleep()


if __name__ == "__main__":
    rospy.init_node("ik")
    main()
    rospy.spin()
