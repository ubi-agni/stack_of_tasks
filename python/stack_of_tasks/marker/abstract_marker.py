from __future__ import annotations

from abc import abstractmethod

import numpy as np
import traits.api as ta
from traits.observation.events import TraitChangeEvent

from tf import transformations as tf
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Vector3
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
)

from stack_of_tasks.ui.utils.class_register import Register
from stack_of_tasks.utils.tf_mappings import matrix_to_pose
from stack_of_tasks.utils.traits import ABCSoTHasTraits, Guard

MarkerRegister = Register("MarkerRegister")


@MarkerRegister.register_base
class IAMarker(ABCSoTHasTraits):
    name = ta.Str(value="")

    transform = ta.Array(
        shape=(4, 4), value=np.identity(4), comparison_mode=ta.ComparisonMode.none
    )

    scale = ta.Range(low=0.0, value=0.25, exclude_low=True)

    sync = ta.Event()

    def __init__(self, name, **kwargs) -> None:
        super().__init__(name=name, **kwargs)
        self.lg = Guard()
        self.markers = []

        self.marker = self._create_interactive_marker(
            name=self.name,
            scale=self.scale,
            pose=matrix_to_pose(self.transform),
        )

        self.markers.append(self.marker)

    @ta.observe("name", post_init=True)
    def _name_changed(self, evt: TraitChangeEvent):
        self.marker.name = evt.new

    @ta.observe("transform", post_init=True)
    def _transform_changed(self, evt: TraitChangeEvent):
        if "transform" not in self.lg:
            self.marker.pose = matrix_to_pose(self.transform)
            self.sync = self.name

    @ta.observe("scale", post_init=True)
    def _scale_changed(self, evt: TraitChangeEvent):
        if "scale" not in self.lg:
            self.marker.scale = self.scale
            self.sync = self.name

    @abstractmethod
    def feedback(self, fb: InteractiveMarkerFeedback):
        pass

    @staticmethod
    def _create_interactive_marker(name: str, scale: float, pose):
        im = InteractiveMarker(name=name, scale=scale)

        if isinstance(pose, PoseStamped):
            im.header.frame_id = pose.header.frame_id
            im.pose = pose.pose
        else:
            im.header.frame_id = "world"
            im.pose = pose if isinstance(pose, Pose) else matrix_to_pose(pose)

        return im

    @staticmethod
    def _add_display_marker(im: InteractiveMarker, name: str, *markers):
        control = InteractiveMarkerControl(
            name=name,
            interaction_mode=InteractiveMarkerControl.NONE,
            always_visible=True,
        )

        control.markers.extend(markers)
        im.controls.append(control)
        return control

    @staticmethod
    def _add_movement_marker(im: InteractiveMarker, name: str, mode: int, *markers):
        control = InteractiveMarkerControl(name=name, interaction_mode=mode)
        control.markers.extend(markers)
        im.controls.append(control)

    @staticmethod
    def _add_movement_control(
        im: InteractiveMarker,
        name: str,
        mode: int,
        directions="xyz",
        or_mode=None,
        *markers,
    ):
        for d in directions:
            control = InteractiveMarkerControl(
                name=f"{name}_{d}",
                interaction_mode=mode,
            )

            if or_mode:
                control.orientation_mode = or_mode

            # todo unnormalized quaternion
            control.orientation.x = 0.7071068 if d == "x" else 0
            control.orientation.z = 0.7071068 if d == "y" else 0
            control.orientation.y = 0.7071068 if d == "z" else 0
            control.orientation.w = 0.7071068

            control.markers.extend(markers)

            im.controls.append(control)

    @staticmethod
    def sphere(radius=0.02, color=ColorRGBA(1, 0, 1, 0.5), **kwargs):
        """Create a sphere marker"""
        return Marker(
            type=Marker.SPHERE, scale=Vector3(radius, radius, radius), color=color, **kwargs
        )

    @staticmethod
    def cylinder(radius=0.02, length=0.1, color=ColorRGBA(1, 0, 0, 1), **kwargs):
        """Create a cylinder marker"""
        return Marker(
            type=Marker.CYLINDER,
            scale=Vector3(radius, radius, length),
            color=color,
            **kwargs,
        )

    @staticmethod
    def box(size=Vector3(0.1, 0.1, 0.1), color=ColorRGBA(1, 1, 1, 0.5), **kwargs):
        """Create a box marker"""
        return Marker(type=Marker.CUBE, scale=size, color=color, **kwargs)

    @staticmethod
    def plane(size=1.0, color=ColorRGBA(1, 1, 1, 0.5), **kwargs):
        """Create a plane (a flat box)"""
        return IAMarker.box(size=Vector3(size, size, 1e-3), color=color, **kwargs)

    @staticmethod
    def cone(halfOpenAngle, scale=0.1, color=ColorRGBA(1, 0, 1, 1), **kwargs):
        twopi = np.pi * 2
        height = scale * np.cos(halfOpenAngle)
        radius = scale * np.sin(halfOpenAngle)
        points = []
        numTriangles = 50

        for i in range(numTriangles):
            # outside
            points.append(Point(0, 0, 0))
            theta = twopi * i / numTriangles
            points.append(Point(radius * np.sin(theta), radius * np.cos(theta), height))
            theta = twopi * (i + 1) / numTriangles
            points.append(Point(radius * np.sin(theta), radius * np.cos(theta), height))

            # inside
            inner = radius * 0.95
            theta = twopi * i / numTriangles
            points.append(Point(inner * np.sin(theta), inner * np.cos(theta), height))
            points.append(Point(0, 0, radius - inner))
            theta = twopi * (i + 1) / numTriangles
            points.append(Point(inner * np.sin(theta), inner * np.cos(theta), height))

        return Marker(
            type=Marker.TRIANGLE_LIST,
            points=points,
            color=color,
            scale=Vector3(1, 1, 1),
            **kwargs,
        )

    @staticmethod
    def arrow(length=0.1, width=None, color=ColorRGBA(1, 0, 0, 1), **kwargs):
        """Create an arrow marker"""
        width = width or 0.1 * length
        scale = Vector3(length, width, width)
        return Marker(type=Marker.ARROW, scale=scale, color=color, **kwargs)

    @staticmethod
    def frame(T, scale=0.1, radius=None, frame_id="world", ns="frame"):
        """Create a frame composed from three cylinders"""
        markers = []
        p = T[0:3, 3]

        defaults = dict(header=Header(frame_id=frame_id), ns=ns)
        if radius is None:
            radius = scale / 10

        xaxis = tf.quaternion_about_axis(np.pi / 2.0, [0, 1, 0])
        yaxis = tf.quaternion_about_axis(np.pi / 2.0, [-1, 0, 0])
        offset = np.array([0, 0, scale / 2.0])

        m = IAMarker.cylinder(radius, scale, color=ColorRGBA(1, 0, 0, 1), id=0, **defaults)
        q = tf.quaternion_multiply(tf.quaternion_from_matrix(T), xaxis)
        m.pose.orientation = Quaternion(*q)
        m.pose.position = Point(*(p + tf.quaternion_matrix(q)[:3, :3].dot(offset)))
        markers.append(m)

        m = IAMarker.cylinder(radius, scale, color=ColorRGBA(0, 1, 0, 1), id=1, **defaults)
        q = tf.quaternion_multiply(tf.quaternion_from_matrix(T), yaxis)
        m.pose.orientation = Quaternion(*q)
        m.pose.position = Point(*(p + tf.quaternion_matrix(q)[:3, :3].dot(offset)))
        markers.append(m)

        m = IAMarker.cylinder(radius, scale, color=ColorRGBA(0, 0, 1, 1), id=2, **defaults)
        m.pose.orientation = Quaternion(*tf.quaternion_from_matrix(T))
        m.pose.position = Point(*(p + T[:3, :3].dot(offset)))
        markers.append(m)
        return markers
