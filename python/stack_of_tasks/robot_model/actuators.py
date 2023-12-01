import numpy

import rospy
from controller_manager_msgs.srv import ListControllers, LoadController, SwitchController
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

from .robot_state import RobotState


def initial_joint_values(state: RobotState, ns="", param="initial_joints"):
    values = rospy.get_param(ns + "/" + param, {})

    if isinstance(values, dict):
        return [
            values.get(joint.name, 0.5 * (joint.min + joint.max))
            for joint in state.robot_model.active_joints
        ]
    elif isinstance(values, list):
        return values
    else:
        raise TypeError(f"Invalid type for {param}: {type(values)}")


class DummyActuator:
    """Directly update current joint values with deltas"""

    def __init__(self, robot_state: RobotState, ns_prefix="") -> None:
        self._robot_state = robot_state
        self.initial_values = initial_joint_values(robot_state, ns_prefix)
        robot_state.incoming_joint_values = self.initial_values
        robot_state.update()

    def set_random_joints(self, randomness=0):
        model = self._robot_state.robot_model
        width = 0.5 * (model.maxs - model.mins) * randomness
        self._robot_state.incoming_joint_values = self.initial_values + width * (
            numpy.random.random_sample(width.shape) - 0.5
        )
        self._robot_state.update()

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


class JointStatePublisher:
    """Publish updated joint values to target_joint_states topic"""

    def __init__(self, robot_state: RobotState, ns_prefix: str = "") -> None:
        self._pub = rospy.Publisher(
            ns_prefix + "target_joint_states", JointState, queue_size=1, latch=True
        )
        self.msg = JointState()
        self.msg.name = [j.name for j in robot_state.robot_model.active_joints]
        self.publish(robot_state.joint_values)

    def publish(self, pos, vel=None):
        self.msg.position = pos
        self.msg.velocity = numpy.zeros_like(pos) if vel is None else vel
        self._pub.publish(self.msg)


class DummyPublisherActuator(DummyActuator, JointStatePublisher):
    """Directly update joint values and publish them, but don't subscribe to /joint_states"""

    def __init__(self, robot_state: RobotState, ns_prefix: str = "") -> None:
        DummyActuator.__init__(self, robot_state, ns_prefix)
        JointStatePublisher.__init__(self, robot_state, ns_prefix)

    def actuate(self, dq):
        DummyActuator.actuate(self, dq)
        self.publish(self._robot_state.joint_values, dq)


class JointStatePublisherActuator(DummyActuator, JointStateSubscriber, JointStatePublisher):
    """Publish updated joint values to target_joint_states topic"""

    def __init__(self, robot_state: RobotState, ns_prefix: str = "") -> None:
        DummyActuator.__init__(self, robot_state, ns_prefix)
        JointStateSubscriber.__init__(self, robot_state, ns_prefix)
        JointStatePublisher.__init__(self, robot_state, ns_prefix)

    def actuate(self, dq):
        self.publish(self._robot_state.joint_values + dq, dq)


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

    @staticmethod
    def switch_controllers(start, stop=None, ns="/controller_manager"):
        def call(ns, cls, **kwargs):
            rospy.wait_for_service(ns, timeout=1)
            service = rospy.ServiceProxy(ns, cls)
            return service(**kwargs)

        loaded = call(ns + "/list_controllers", ListControllers)
        loaded = [c.name for c in loaded.controller]

        for name in start:
            if name not in loaded:
                call(ns + "/load_controller", LoadController, name=name)

        if not call(
            ns + "/switch_controller",
            SwitchController,
            start_controllers=start,
            stop_controllers=stop,
            strictness=1,
            start_asap=False,
            timeout=0.0,
        ).ok:
            raise RuntimeError("Failed to switch controller")

    def stop(self, stop=None):
        self.actuate(numpy.zeros(self._robot_state.robot_model.N))
        self.switch_controllers([], stop)
