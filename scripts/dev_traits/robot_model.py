#!/usr/bin/env python3

import rospy

from stack_of_tasks.ref_frame.frames import RobotRefFrame
from stack_of_tasks.robot_model.jointstate_publisher import JointStatePublisher
from stack_of_tasks.robot_model.robot_model import RobotModel
from stack_of_tasks.robot_model.robot_state import RobotState


def main():
    rospy.init_node("ik")

    m = RobotModel()

    s = RobotState(m)
    f = RobotRefFrame(s, "panda_hand_tcp")
    _ = JointStatePublisher(s)

    # f.observe(print, "T")

    def jiggle(rs: RobotState, link: int, rateHz: int, dur=1):
        rate = rospy.Rate(50)

        start_val = cval = rs.joint_values[link]

        m = rs.robot_model.mins[link]
        M = rs.robot_model.maxs[link]

        radPerStep = (M - m) / dur / rateHz

        def move_step(val):
            nonlocal cval, radPerStep
            if cval <= val:
                cval += radPerStep
            else:
                cval -= radPerStep

        def move(dest):
            nonlocal cval, radPerStep
            while abs(cval - dest) > radPerStep:
                move_step(dest)
                rs.joint_values[link] = cval
                rate.sleep()

            rs.joint_values[link] = cval = dest
            rate.sleep()

        move(m)
        move(M)
        move(start_val)

    for i in range(8):
        jiggle(s, i, 50, 1)


if __name__ == "__main__":
    main()
