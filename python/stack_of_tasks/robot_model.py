#!/usr/bin/env python3

"""
This module contains different Joint and RobotModel representation with an forward kinematic.
"""

from enum import Enum
from typing import Dict
from xml.dom.minidom import Element, parseString

import numpy
import rospy
from tf import transformations as tf


def hat(w):
    """
    Generates skew-symmetric matrix.
    Usefull for cross product as matrix operation.

    Args:
        w (NDArray): Left side vector of cross product

    Returns:
        NDArray: Matrix
    """
    return numpy.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])


def adjoint(T, inverse=False):
    """
    Generates the adjoint matrix for given Transform T.

    Args:
        T (NDArray): Transform matrix.
        inverse (bool, optional): Inverse adjoint. Defaults to False.

    Returns:
        NDArray: Adjoint matrix
    """
    if T.shape == (4, 4):
        R = T[0:3, 0:3]
        p = T[0:3, 3]
    elif T.shape == (3, 3):
        R = T
        p = numpy.zeros(3)
    else:
        R = numpy.identity(3)
        p = T
    r = numpy.zeros((6, 6))
    if inverse:
        r[:3, :3] = r[3:, 3:] = R.T
        r[:3, 3:] = R.T.dot(hat(-p))
    else:
        r[:3, :3] = r[3:, 3:] = R
        r[:3, 3:] = hat(p).dot(R)

    return r


class JointType(Enum):
    """
    Enum of available joint types.
    """

    fixed = "fixed"
    revolute = "revolute"
    continuous = "continuous"
    prismatic = "prismatic"
    floating = "floating"


class Joint:
    """
    Joint base class.
    """

    def __init__(self, jtype, name, T):
        self.jtype = jtype
        self.name = name
        self.T = T

        self.parent_joint = None


class ActiveJoint(Joint):
    """
    Representation of Active Joints
    """

    def __init__(self, jtype, name, T, axis, joint_min, joint_max):
        super().__init__(jtype, name, T)

        self.axis = axis
        self.min = joint_min
        self.max = joint_max
        self.joint_twist = None

        if self.jtype is JointType.revolute:
            self.joint_twist = numpy.block([numpy.zeros(3), self.axis])

        elif self.jtype is JointType.prismatic:
            self.joint_twist = numpy.block([self.axis, numpy.zeros(3)])

    def T_motion(self, joint_angle):
        """Returns transformmatrix for this joints motion, given a joint angle

        Args:
            joint_angle (float): Current joint angle

        Returns:
            Any: Transform matrix
        """
        if self.jtype is JointType.revolute:
            return tf.quaternion_matrix(
                tf.quaternion_about_axis(angle=joint_angle, axis=self.axis)
            )
        elif self.jtype is JointType.prismatic:
            return tf.translation_matrix(joint_angle * self.axis)


class MimicJoint(ActiveJoint):
    """
    Active joint, which mimics the motion of another joint.
    """

    def __init__(
        self,
        jtype,
        name,
        T,
        axis,
        joint_min,
        joint_max,
        mimic_joint_name,
        mimic_offset,
        mimic_mult,
    ):
        super().__init__(jtype, name, T, axis, joint_min, joint_max)

        self.mimic_joint_name = mimic_joint_name
        self.mimic_joint = None
        self.mimic_offset = mimic_offset
        self.mimic_mult = mimic_mult

        self.joint_twist = self.mimic_mult * self.joint_twist

    def T_motion(self, joint_angle):

        return super().T_motion(self.mimic_offset + self.mimic_mult * joint_angle)


class JointFactory:
    """
    Joint factory class.
    """

    @staticmethod
    def get_xml_attr(tag: Element, attr: str, attr_type=str, default=None, child: str = None):
        """
        Retrieves the value of the attribute 'attr' of tag or the tags 'child', if given.
        Casts the value to 'type'. If the attribute is missing, the function returns 'default' or None.

        Args:
            tag (Element): XML Element
            attr (str): Attribute name
            attr_type (Any, optional): Casting type. Defaults to str.
            default (Any, optional): Default value. Defaults to None.
            child (str, optional): Child name. Defaults to None.

        Returns:
            Nay: Attributes value.
        """
        if child:
            tag = tag.getElementsByTagName(child)[0]

        if tag.hasAttribute(attr):
            return attr_type(tag.getAttribute(attr))
        else:
            return default

    @staticmethod
    def get_vector_from_xml(tag: Element, attr, child=None, default=None):
        v = JointFactory.get_xml_attr(tag, attr, child=child, default=default if default else [])
        return numpy.array(v.split(" "), numpy.float64)

    @staticmethod
    def from_xml(tag: Element):
        gxa = JointFactory.get_xml_attr
        pv = JointFactory.get_vector_from_xml

        jtype = JointType(tag.getAttribute("type"))
        name = tag.getAttribute("name")

        parent = gxa(tag, "link", child="parent")
        child = gxa(tag, "link", child="child")

        T = tf.euler_matrix(*pv(tag, "rpy", child="origin"), axes="sxyz")
        T[0:3, 3] = pv(tag, "xyz", child="origin")

        joint = None

        if jtype in [JointType.revolute, JointType.prismatic]:
            axis = pv(tag, "xyz", child="axis")

            jmin = gxa(tag, "lower", attr_type=float, child="limit")
            jmax = gxa(tag, "upper", attr_type=float, child="limit")
            if jmin is None or jmax is None:
                raise Exception(f"Joint {name} has not limits")

            if mimic := tag.getElementsByTagName("mimic"):
                mimic = mimic[0]
                m_joint = gxa(mimic, "joint")
                multiplier = gxa(mimic, "multiplier", float, 0.0)
                offset = gxa(mimic, "offset", float, 0.0)

                joint = MimicJoint(jtype, name, T, axis, jmin, jmax, m_joint, offset, multiplier)

            else:
                joint = ActiveJoint(jtype, name, T, axis, jmin, jmax)

        else:
            joint = Joint(jtype, name, T)

        return joint, parent, child


class RobotModel:
    def __init__(self, param="robot_description"):
        self.joints = {}  # map joint name to joint instance
        self.active_joints = []  # active joints

        description = rospy.get_param(param)
        doc = parseString(description)  # nosec B318
        # ignore https://bandit.readthedocs.io/en/latest/blacklists/blacklist_calls.html?highlight=B318#b313-b320-xml

        robot = doc.getElementsByTagName("robot")[0]

        links = {}
        for tag in robot.getElementsByTagName("joint"):
            joint, parent_name, child_name = JointFactory.from_xml(
                tag
            )  # get joint and its parent and child links from xml

            self.joints[joint.name] = joint

            # store parent and child information
            if parent_name in links:
                links[parent_name]["c"].append(joint)
            else:
                links[parent_name] = {"c": [joint], "p": None}

            if child_name in links:
                links[child_name]["p"] = joint
            else:
                links[child_name] = {"c": [], "p": joint}

        # set parent joints for all joints. link information no longer needed
        for link in links.values():
            for child_name in link["c"]:
                child_name.parent_joint = link["p"]

        # store list of active joints
        self.active_joints = list(filter(lambda x: type(x) == ActiveJoint, self.joints.values()))

        # set relation in mimic joints
        for joint in filter(lambda x: type(x) == MimicJoint, self.joints.values()):
            joint.mimic_joint = self.joints[joint.mimic_joint_name]

    def fk(self, target_joint_name: str, joint_values: Dict[str, float]):
        """Calculates forward kinematric for all joints up to ``target_joint_name``


        Args:
            target_joint_name (str): Name of target joint
            joint_values (Dict[str, float]): current joint values

        Returns:
            (NDArray, NDArray): Return Transformation T and Jacobi matrix
        """

        T = numpy.identity(4)
        T_all = {}
        J = numpy.zeros((6, len(self.active_joints)))

        idx = len(self.active_joints) - 2
        joint: Joint = self.joints[target_joint_name]

        while joint is not None:

            T_offset = joint.T  # fixed transform from parent to joint frame

            if isinstance(joint, ActiveJoint):

                value = joint_values[
                    joint.mimic_joint_name if type(joint) == MimicJoint else joint.name
                ]

                # transform twist from current joint frame (joint.axis) into eef frame (via T^-1)
                twist = adjoint(T, inverse=True).dot(joint.joint_twist)
                T_motion = joint.T_motion(value)
                # post-multiply joint's motion transform (rotation / translation along joint axis)
                T_offset = T_offset.dot(T_motion)

                # update the Jacobian
                J[:, idx] += twist  # add twist contribution
                idx -= 1

            # pre-multiply joint transform with T (because traversing from eef to root)
            T = T_offset.dot(T)  # T' = joint.T * T_motion * T
            T_all[joint.name] = T
            # climb upwards to parent joint
            joint = joint.parent_joint

        # As we climbed the kinematic tree from end-effector to base frame, we have
        # represented all Jacobian twists w.r.t. the end-effector frame.
        # Now transform all twists into the orientation of the base frame
        R = T[0:3, 0:3]
        Ad_R = numpy.zeros((6, 6))
        Ad_R[:3, :3] = Ad_R[3:, 3:] = R
        return T_all, Ad_R.dot(J)


def main():
    import random

    from sensor_msgs.msg import JointState

    from utils import prettyprintMatrix

    rospy.init_node("test_node")
    pub = rospy.Publisher("/target_joint_states", JointState, queue_size=10)

    robot = RobotModel()

    while not rospy.is_shutdown():
        joints = {j.name: random.uniform(j.min, j.max) for j in robot.active_joints}
        pub.publish(JointState(name=joints.keys(), position=joints.values()))

        T_all, J = robot.fk("panda_joint8", joints)
        print("J:\n", prettyprintMatrix(J), "\n")

        for k, v in T_all.items():
            print(f"{k}:")
            print(prettyprintMatrix(v), "\n")
        rospy.rostime.wallsleep(1)


# code executed when directly running this script
if __name__ == "__main__":
    main()
