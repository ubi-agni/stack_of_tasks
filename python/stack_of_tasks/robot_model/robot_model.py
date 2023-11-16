"""
This module contains different Joint and RobotModel representation with an forward kinematic.
"""


from enum import Enum
from xml.dom import Node, minidom

from typing import Dict, List

import numpy

import rospy
from tf import transformations as tf


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

    def __init__(self, jtype, name, T, axis, joint_min, joint_max, velocity_max):
        super().__init__(jtype, name, T)

        self.idx = None  # index of associated Jacobian column
        self.axis = axis
        self.min = joint_min
        self.max = joint_max
        self.vmax = velocity_max
        self.twist = None

        if self.jtype is JointType.revolute:
            self.twist = numpy.block([numpy.zeros(3), self.axis])
            self._motion = lambda q: tf.rotation_matrix(angle=q, direction=self.axis)

        elif self.jtype is JointType.prismatic:
            self.twist = numpy.block([self.axis, numpy.zeros(3)])
            self._motion = lambda q: tf.translation_matrix(q * self.axis)

    def T_motion(self, joint_angle):
        return self._motion(joint_angle)


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
        velocity_max,
        base,
        multiplier,
        offset,
    ):
        super().__init__(jtype, name, T, axis, joint_min, joint_max, velocity_max)

        self.base = base
        self.multiplier = multiplier
        self.offset = offset
        self.twist *= self.multiplier

    def T_motion(self, joint_angle):
        return super().T_motion(joint_angle * self.multiplier + self.offset)


class RobotModelException(BaseException):
    """Raised when processing a robot model fails."""


class RobotModel:
    def __init__(self, param="robot_description"):
        self.joints: Dict[str, Joint] = {}  # map joint name to joint instance
        self.links = {}  # map link name to driving joint instance
        self.active_joints = []  # active joints

        # load model
        description = rospy.get_param(param)
        doc = minidom.parseString(description)
        robot = next(getElementsByTag(doc, "robot"))

        unlinked = dict()
        for tag in getElementsByTag(robot, "joint"):  # process all joint tags
            pending = self._add_joint(*self._create_joint(tag))  # create and add joint
            unlinked.update(pending)
        for joint, parent in unlinked.items():  # find parent for all not yet linked joints
            joint.parent = self.links.get(parent, None)

        # store list of active joints. Attention: Mimic joints are no true active joints!
        self.active_joints = [j for j in self.joints.values() if type(j) is ActiveJoint]

        for idx, joint in enumerate(self.active_joints):
            joint.idx = idx

        # set indexes of mimic joints
        for joint in self.joints.values():
            if isinstance(joint, MimicJoint):
                joint.idx = joint.base.idx

        # infos derived from model

        self.N = len(self.active_joints)
        self.mins = numpy.array([j.min for j in self.active_joints])
        self.maxs = numpy.array([j.max for j in self.active_joints])
        self.vmaxs = numpy.array([j.vmax for j in self.active_joints])

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
            vmax = gxa(tag, "velocity", default=10.0, child="limit")

            if jmin is None or jmax is None:
                raise RobotModelException(f"Joint {name} has not limits")

            if mimic := list(getElementsByTag(tag, "mimic")):
                mimic = mimic[0]
                base, multiplier, offset = self._mimic_base(
                    gxa(mimic, "joint"),
                    gxa(mimic, "multiplier", 1.0),
                    gxa(mimic, "offset", 0.0),
                )
                joint = MimicJoint(
                    jtype, name, T, axis, jmin, jmax, vmax, base, multiplier, offset
                )
            else:
                joint = ActiveJoint(jtype, name, T, axis, jmin, jmax, vmax)
        else:
            joint = Joint(jtype, name, T)

        return joint, parent, child

    def _add_joint(self, joint: Joint, parent: str, child: str):
        self.joints[joint.name] = joint
        self.links[child] = joint

        joint.parent = self.links.get(parent, None)

        return dict([(joint, parent)]) if joint.parent is None else dict()

    @property
    def link_names(self):
        return list(self.links.keys())

    def isEEF(self, link: str):
        joint = self.links[link]
        return not any([joint is j.parent for j in self.joints.values()])

    def getEEFs(self) -> List[str]:
        return {link for link in self.links if self.isEEF(link)}
