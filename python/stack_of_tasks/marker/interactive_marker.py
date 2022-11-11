import warnings
from abc import ABC, abstractmethod

import numpy

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from tf import transformations as tf
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Vector3
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker

from stack_of_tasks.ref_frame.frames import RefFrame, Transform, World
from stack_of_tasks.ref_frame.offset import OffsetRefFrame
from stack_of_tasks.utils import create_pose, pose_to_matrix


class IAMarker(ABC):
    def __init__(
        self,
        server: InteractiveMarkerServer,
        name: str,
        callback=None,
        **kwargs,
    ) -> None:

        self.server = None
        self.name = name
        self.data_callbacks = [callback] if callback is not None else []

        self._setup_server = lambda: self._setup_marker(name, **kwargs)
        if self.server:
            self.init_server(server)

    def init_server(self, server: InteractiveMarkerServer):
        if self.server is None:
            self.server = server
            self._setup_server()
            del self._setup_server
            self.server.applyChanges()
        else:
            warnings.warn(f"Server of marker {self.name} was already set!", RuntimeWarning)

    @abstractmethod
    def _setup_marker(self, name, /, **kwargs):
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
    def _create_interactive_marker(server, name, scale, pose, callback=None):
        im = InteractiveMarker(name=name)

        if isinstance(pose, PoseStamped):
            im.header.frame_id = pose.header.frame_id
            im.pose = pose.pose
        else:
            im.header.frame_id = "world"
            im.pose = pose if isinstance(pose, Pose) else create_pose(pose)
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
            type=Marker.SPHERE, scale=Vector3(radius, radius, radius), color=color, **kwargs
        )

    @staticmethod
    def cylinder(radius=0.02, len=0.1, color=ColorRGBA(1, 0, 0, 1), **kwargs):
        """Create a cylinder marker"""
        return Marker(
            type=Marker.CYLINDER, scale=Vector3(radius, radius, len), color=color, **kwargs
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
    def arrow(len=0.1, width=None, color=ColorRGBA(1, 0, 0, 1), **kwargs):
        """Create an arrow marker"""
        width = width or 0.1 * len
        scale = Vector3(len, width, width)
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


class ConeMarker(IAMarker):
    def __init__(
        self,
        server: InteractiveMarkerServer = None,
        name: str = "Cone",
        frame: RefFrame = World(),
        offset: Transform = None,
        scale: float = 1,
        angle=0.4,
        mode=InteractiveMarkerControl.ROTATE_3D,
    ) -> None:
        self._angle = angle
        self._scale = scale
        super().__init__(server, name=name, frame=frame, offset=offset, mode=mode)

    def _setup_marker(self, name, frame, offset, mode):
        self.ref_frame = OffsetRefFrame(frame, offset)
        self.ref_frame.callback.append(self._set_marker_pose)

        self.loc_marker = self._create_interactive_marker(
            self.server,
            f"{self.name}_Pos",
            pose=self.ref_frame.offset,
            scale=0.1,
            callback=self._callback_pose,
        )

        self._add_display_marker(self.loc_marker, "", self.cone(self._angle, self._scale))
        self._add_movement_control(self.loc_marker, "", InteractiveMarkerControl.MOVE_AXIS)
        self._add_movement_control(self.loc_marker, "", InteractiveMarkerControl.ROTATE_AXIS)

        handle_marker = self._create_interactive_marker(
            self.server,
            f"{self.name}_Handle",
            scale=0.05,
            callback=self._callback_angle,
            pose=tf.translation_matrix([0, 0, 0]),
        )

        self._add_movement_marker(
            handle_marker, "", InteractiveMarkerControl.MOVE_PLANE, marker=self.sphere()
        )
        self._add_movement_control(
            handle_marker, "", InteractiveMarkerControl.MOVE_AXIS, directoins="z"
        )
        self._add_movement_control(
            handle_marker, "", InteractiveMarkerControl.MOVE_AXIS, directoins="y"
        )
        self.server.applyChanges()

        self._calc_handle_pose(self.ref_frame.offset)
        self._data_callback(f"{self.name}_angle", self._angle)
        self.server.applyChanges()

    def provided_targets(self):
        return ["{self.name}_Pos", "{self.name}_Handle"]

    def delete(self):
        self.server.erase(f"{self.name}_Pos")
        self.server.erase(f"{self.name}_Handle")
        self.server.applyChanges()

    def _calc_handle_pose(self, T_root):
        handle_pose = tf.rotation_matrix(self._angle, [1, 0, 0]).dot(
            tf.translation_matrix([0, 0, self._scale])
        )
        self.server.setPose(f"{self.name}_Handle", create_pose(T_root.dot(handle_pose)))

    def _set_marker_pose(self, transform):
        self.loc_marker.pose = create_pose(transform)
        self._calc_handle_pose(transform)
        self.server.applyChanges()

    def _callback_pose(self, feedback):
        self.ref_frame.offset = pose_to_matrix(feedback.pose)

        self._calc_handle_pose(pose_to_matrix(feedback.pose))
        self.server.applyChanges()

    def _callback_angle(self, feedback):
        T_marker = pose_to_matrix(feedback.pose)
        coneM = self._get_marker(f"{self.name}_Pos")

        # vector from cone's origin to marker
        v = T_marker[0:3, 3] - self.ref_frame.offset[0:3, 3]
        self._scale = numpy.linalg.norm(v)
        self._angle = numpy.arccos(
            numpy.maximum(0, self.ref_frame.offset[0:3, 2].dot(v) / self._scale)
        )

        coneM.controls[0].markers[0].points = self.cone(self._angle, self._scale).points
        self._update_marker(coneM)
        self._data_callback(f"{self.name}_angle", self._angle)
        self.server.applyChanges()
