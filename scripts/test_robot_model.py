#!/usr/bin/env python3

import pstats
from cProfile import Profile
from random import randint

import numpy

import rospy

from stack_of_tasks.robot_model import RobotModel


def _main():

    rospy.init_node("ik")
    numpy.set_printoptions(precision=2, linewidth=200)
    rm = RobotModel(param="robot_description", ns_prefix="")

    j = numpy.zeros((rm.N,))
    rm.joint_values = j

    def dump_stats(profile, title):
        print(title)
        stats = pstats.Stats(profile)
        stats.strip_dirs().sort_stats(pstats.SortKey.CUMULATIVE).print_stats(10)

    joints = list(rm.joints.values())[:7]

    numJoints = len(joints) - 1

    random_joints = [
        [
            joints[randint(1, numJoints)].name,
            joints[randint(1, numJoints)].name,
            joints[randint(1, numJoints)].name,
        ]
        for _ in range(1000)
    ]

    with Profile() as p:
        for j1, j2, j3 in random_joints:
            rm.fk(j1)
            rm.fk(j2)
            rm.fk(j3)

            rm.clear_cache()

    dump_stats(p, "Recursive, caching FK 3 random joints over 1000 runs")


if __name__ == "__main__":
    _main()
