from abc import ABC, abstractmethod

import numpy

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from tf import transformations as tf
from geometry_msgs.msg import Point, Quaternion, Vector3
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker

from .utils import create_pose


class IAMarker(ABC):
    def __init__(
        self,
        server: InteractiveMarkerServer,
        name: str,
        pose,
        scale: float,
        frame: str = "world",
        callback=None,
        **kwargs,
    ) -> None:

        self.server = None
        self.name = name
        self.data_callbacks = [callback] if callback else []

        self._setup_server = lambda: self._setup_marker(name, pose, scale, **kwargs)
        if self.server:
            self.init_server(server)

    def init_server(self, server: InteractiveMarkerServer):
        if self.server is None:
            self.server = server
            self._setup_server()
            del self._setup_server
            self.server.applyChanges()
        else:
            pass  # Warning, server already set

    @abstractmethod
    def _setup_marker(self, name, pose, scale, **kwargs):
        pass

    @abstractmethod
    def provided_targets(self):
        pass

    @abstractmethod
    def delete(self):
        pass

    def _data_callback(self, name, data):
        for c in self.data_callbacks:
            c(name, data)

    def _update_marker(self, marker):
        self.server.insert(marker)

    def _get_marker(self, name) -> InteractiveMarker:
        return self.server.marker_contexts.get(name).int_marker

    @staticmethod
    def _create_interactive_marker(
        server, name, scale, pose, frame_id="world", callback=None
    ):
        im = InteractiveMarker(name=name)
        im.header.frame_id = frame_id
        im.pose = create_pose(pose)
        im.scale = scale

        server.insert(im, callback)
        return im

    @staticmethod
    def _add_display_marker(im: InteractiveMarker, name, marker=None, markers=None):

        control = InteractiveMarkerControl()
        control.name = name
        control.interaction_mode = InteractiveMarkerControl.NONE
        control.always_visible = True

        if marker:
            control.markers.append(marker)
        if markers:
            control.markers.extend(markers)
        im.controls.append(control)

    @staticmethod
    def _add_movement_marker(im: InteractiveMarker, name, mode, marker=None, markers=None):
        control = InteractiveMarkerControl()
        control.name = name
        control.interaction_mode = mode
        if marker:
            control.markers.append(marker)
        if markers:
            control.markers.extend(markers)
        im.controls.append(control)

    @staticmethod
    def _add_movement_control(
        im, name, mode, directoins="xyz", or_mode=None, marker=None, markers=None
    ):
        for d in directoins:
            control = InteractiveMarkerControl()
            control.name = f"{name}_{d}"
            control.interaction_mode = mode
            if or_mode:
                control.orientation_mode = or_mode
            control.orientation.x = 0.7071068 if d == "x" else 0
            control.orientation.z = 0.7071068 if d == "y" else 0
            control.orientation.y = 0.7071068 if d == "z" else 0
            control.orientation.w = 0.7071068

            if marker:
                control.markers.append(marker)
            if markers:
                control.markers.extend(markers)

            im.controls.append(control)

    @staticmethod
    def sphere(radius=0.02, color=ColorRGBA(1, 0, 1, 0.5), **kwargs):
        """Create a sphere marker"""
        return Marker(
            type=Marker.SPHERE,
            scale=Vector3(radius, radius, radius),
            color=color,
            **kwargs,
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
        twopi = numpy.pi * 2
        height = scale * numpy.cos(halfOpenAngle)
        radius = scale * numpy.sin(halfOpenAngle)
        points = []
        numTriangles = 50

        for i in range(numTriangles):
            # outside
            points.append(Point(0, 0, 0))
            theta = twopi * i / numTriangles
            points.append(Point(radius * numpy.sin(theta), radius * numpy.cos(theta), height))
            theta = twopi * (i + 1) / numTriangles
            points.append(Point(radius * numpy.sin(theta), radius * numpy.cos(theta), height))

            # inside
            inner = radius * 0.95
            theta = twopi * i / numTriangles
            points.append(Point(inner * numpy.sin(theta), inner * numpy.cos(theta), height))
            points.append(Point(0, 0, radius - inner))
            theta = twopi * (i + 1) / numTriangles
            points.append(Point(inner * numpy.sin(theta), inner * numpy.cos(theta), height))

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

        xaxis = tf.quaternion_about_axis(numpy.pi / 2.0, [0, 1, 0])
        yaxis = tf.quaternion_about_axis(numpy.pi / 2.0, [-1, 0, 0])
        offset = numpy.array([0, 0, scale / 2.0])

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
