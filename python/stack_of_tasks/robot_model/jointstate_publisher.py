import traits.api as ta

import rospy
from sensor_msgs.msg import JointState

from stack_of_tasks.utils.traits import BaseSoTHasTraits

from .robot_state import RobotState


class JointStatePublisher(BaseSoTHasTraits):
    robot_state = ta.Instance(RobotState)

    def __init__(self, robot_state: RobotState, ns_prefix: str = "") -> None:
        self.ns_prefix = ns_prefix

        self._pub = rospy.Publisher(
            ns_prefix + "target_joint_states", JointState, queue_size=1, latch=True
        )

        super().__init__(robot_state=robot_state)

    @ta.on_trait_change("robot_state.joint_values")
    def publish_joints(self):
        self._pub.publish(self.robot_state.joint_state_msg)
