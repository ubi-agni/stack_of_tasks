#!/usr/bin/env python3

"""
This module contains different Joint and RobotModel representation with an forward kinematic.
"""

from enum import Enum
from typing import List
from xml.dom import Node, minidom

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


def getElementsByTag(elem, name):
    for node in elem.childNodes:
        if node.nodeType == Node.ELEMENT_NODE and node.tagName == name:
            yield node


def get_xml_attr(tag: minidom.Element, attr: str, default=None, child: str = None):
    """
    Retrieves the value of the attribute 'attr' of tag or the tags 'child', if given.
    Casts the value to 'type'. If the attribute is missing, the function returns 'default' or None.

    Args:
        tag (Element): XML Element
        attr (str): Attribute name
        default (Any, optional): Default value. Defaults to None.
        child (str, optional): Child name. Defaults to None.

    Returns:
        Nay: Attributes value.
    """
    if child:
        try:
            tag = next(getElementsByTag(tag, child))
        except StopIteration:
            return default

    if tag.hasAttribute(attr):
        attr_type = str if default is None else type(default)
        return attr_type(tag.getAttribute(attr))
    else:
        return default


def get_vector_from_xml(tag: minidom.Element, attr, child=None, default=None):
    v = get_xml_attr(tag, attr, child=child, default=default if default else "")
    return numpy.array(v.split(), numpy.float64)


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
        self.parent = None
        self.T = T


class ActiveJoint(Joint):
    """
    Representation of Active Joints
    """

    def __init__(self, jtype, name, T, axis, joint_min, joint_max):
        super().__init__(jtype, name, T)

        self.idx = None  # index of associated Jacobian column
        self.axis = axis
        self.min = joint_min
        self.max = joint_max
        self.twist = None

        if self.jtype is JointType.revolute:
            self.twist = numpy.block([numpy.zeros(3), self.axis])

        elif self.jtype is JointType.prismatic:
            self.twist = numpy.block([self.axis, numpy.zeros(3)])

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
        base,
        multiplier,
        offset,
    ):
        super().__init__(jtype, name, T, axis, joint_min, joint_max)

        self.base = base
        self.multiplier = multiplier
        self.offset = offset
        self.twist *= self.multiplier

    def T_motion(self, joint_angle):
        return super().T_motion(joint_angle * self.multiplier + self.offset)


class RobotModel:
    def __init__(self, param="robot_description"):
        self.joints = {}  # map joint name to joint instance
        self.links = {}  # map link name to driving joint instance
        self.active_joints = []  # active joints

        description = rospy.get_param(param)
        doc = minidom.parseString(description)
        robot = next(getElementsByTag(doc, "robot"))

        unlinked = dict()
        for tag in getElementsByTag(robot, "joint"):  # process all joint tags
            pending = self._add_joint(*self._create_joint(tag))  # create and add joint
            unlinked.update(pending)
        for joint, parent in unlinked.items():  # find parent for all not yet linked joints
            joint.parent = self.links.get(parent, None)

        # store list of active joints
        self.active_joints = [j for j in self.joints.values() if type(j) == ActiveJoint]
        for idx, joint in enumerate(self.active_joints):
            joint.idx = idx

        # set indexes of mimic joints
        for joint in self.joints.values():
            if isinstance(joint, MimicJoint):
                joint.idx = joint.base.idx

    def _mimic_base(self, name: str, multiplier: float, offset: float):
        "recursively resolve MimicJoint to its base"
        while True:
            base = self.joints[name]
            if not isinstance(base, MimicJoint):
                return base, multiplier, offset
            multiplier *= base.multiplier
            offset += multiplier + base.offset
            name = base.name

    def _create_joint(self, tag: minidom.Element):
        gxa = get_xml_attr
        pv = get_vector_from_xml

        jtype = JointType(tag.getAttribute("type"))
        name = tag.getAttribute("name")

        parent = gxa(tag, "link", child="parent")
        child = gxa(tag, "link", child="child")

        T = tf.euler_matrix(*pv(tag, "rpy", child="origin", default="0 0 0"), axes="sxyz")
        T[0:3, 3] = pv(tag, "xyz", child="origin", default="0 0 0")

        if jtype in [JointType.revolute, JointType.prismatic]:
            axis = pv(tag, "xyz", child="axis")
            jmin = gxa(tag, "lower", default=-numpy.inf, child="limit")
            jmax = gxa(tag, "upper", default=numpy.inf, child="limit")
            if jmin is None or jmax is None:
                raise Exception(f"Joint {name} has not limits")

            if mimic := list(getElementsByTag(tag, "mimic")):
                mimic = mimic[0]
                base, multiplier, offset = self._mimic_base(
                    gxa(mimic, "joint"),
                    gxa(mimic, "multiplier", 1.0),
                    gxa(mimic, "offset", 0.0),
                )
                joint = MimicJoint(jtype, name, T, axis, jmin, jmax, base, multiplier, offset)
            else:
                joint = ActiveJoint(jtype, name, T, axis, jmin, jmax)
        else:
            joint = Joint(jtype, name, T)

        return joint, parent, child

    def _add_joint(self, joint: Joint, parent: str, child: str):
        self.joints[joint.name] = joint
        self.links[child] = joint
        joint.parent = self.links.get(parent, None)
        return dict([(joint, parent)]) if joint.parent is None else dict()

    def fk(self, target_joint_name: str, joint_values: List[float]):
        """Calculates forward kinematric for all joints up to ``target_joint_name``


        Args:
            target_joint_name (str): Name of target joint
            joint_values (Dict[str, float]): current joint values

        Returns:
            (NDArray, NDArray): Return Transformation T and Jacobi matrix
        """

        T = numpy.identity(4)
        J = numpy.zeros((6, len(self.active_joints)))
        joint = self.links.get(target_joint_name, None) or self.joints[target_joint_name]

        while joint is not None:
            T_offset = joint.T  # fixed transform from parent to joint frame

            if isinstance(joint, ActiveJoint):

                # transform twist from current joint frame (joint.axis) into eef frame (via T^-1)
                twist = adjoint(T, inverse=True).dot(joint.twist)
                T_motion = joint.T_motion(joint_values[joint.idx])
                # post-multiply joint's motion transform (rotation / translation along joint axis)
                T_offset = T_offset.dot(T_motion)

                # update the Jacobian
                J[:, joint.idx] += twist  # add twist contribution

            # pre-multiply joint transform with T (because traversing from eef to root)
            T = T_offset.dot(T)  # T' = joint.T * T_motion * T
            # climb upwards to parent joint
            joint = joint.parent

        # As we climbed the kinematic tree from end-effector to base frame, we have
        # represented all Jacobian twists w.r.t. the end-effector frame.
        # Now transform all twists into the orientation of the base frame
        R = T[0:3, 0:3]
        Ad_R = numpy.zeros((6, 6))
        Ad_R[:3, :3] = Ad_R[3:, 3:] = R
        return T, Ad_R.dot(J)
