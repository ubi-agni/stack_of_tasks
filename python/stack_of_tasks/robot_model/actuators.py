import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

from .robot_state import RobotState


class DummyActuator:
    """Directly update current joint values with deltas"""

    def __init__(self, robot_state: RobotState) -> None:
        self._robot_state = robot_state

    def actuate(self, dq):
        self._robot_state.incoming_joint_values = self._robot_state.joint_values + dq


class JointStateSubscriber:
    """Update RobotState's incoming_joint_values from /joint_states topic"""

    def __init__(self, robot_state: RobotState, ns_prefix: str = "") -> None:
        self._robot_state = robot_state
        self.ns_prefix = ns_prefix
        self._sub = rospy.Subscriber(
            ns_prefix + "joint_states", JointState, self._joint_states_cb
        )

    def _joint_states_cb(self, msg):
        incoming = dict(zip(msg.name, msg.position))
        self._robot_state.incoming_joint_values = [
            incoming.get(j.name, 0) for j in self._robot_state.robot_model.active_joints
        ]


class JointStatePublisherActuator(JointStateSubscriber):
    """Publish updated joint values to target_joint_states topic"""

    def __init__(self, robot_state: RobotState, ns_prefix: str = "") -> None:
        super().__init__(robot_state, ns_prefix)
        self._pub = rospy.Publisher(
            ns_prefix + "target_joint_states", JointState, queue_size=1, latch=True
        )
        self.msg = JointState()
        self.msg.name = [j.name for j in robot_state.robot_model.active_joints]

    def actuate(self, dq):
        self.msg.position = self._robot_state.joint_values + dq
        self._pub.publish(self.msg)


class VelocityCommandActuator(JointStateSubscriber):
    """Publish joint state deltas to velocity controller"""

    def __init__(
        self, robot_state: RobotState, rate: float, ns: str = "/joint_velocity_controller"
    ) -> None:
        super().__init__(robot_state)
        self._pub = rospy.Publisher(
            ns + "/command", Float64MultiArray, queue_size=1, latch=True
        )
        # retrieve (sub)set of controlled joints
        controlled = rospy.get_param(ns + "/joints")
        active = robot_state.robot_model.active_joints
        self.joint_mask = [j.name in controlled for j in active]

        self.msg = Float64MultiArray()
        dim = MultiArrayDimension(label="dq", size=len(controlled), stride=len(controlled))
        self.msg.layout.dim.append(dim)
        self.rate = rate

    def actuate(self, dq):
        self.msg.data = dq[self.joint_mask] * self.rate
        self._pub.publish(self.msg)
