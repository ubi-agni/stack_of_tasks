from __future__ import print_function

from abc import ABC, abstractmethod

import numpy
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from interactive_markers.interactive_marker_server import (
    InteractiveMarkerFeedback, InteractiveMarkerServer)
from std_msgs.msg import ColorRGBA, Header
from tf import transformations as tf
from visualization_msgs.msg import (InteractiveMarker,
                                    InteractiveMarkerControl, Marker)


class IAMarker(ABC):
    def __init__(self, server: InteractiveMarkerServer=None, name: str="Marker", pose=None, scale:int=1, callback=None, **kwargs) -> None:
        self.server = server
        self._name = name
        
        if callback:
            self.data_callbacks = [callback]
        else:
            self.data_callbacks = []

        if self.server:
            self._setup_marker(name, pose, scale, **kwargs)
            self.server.applyChanges()
        else:
            ## hold back creation
            self._setup_later = lambda: self._setup_marker(name, pose, scale, **kwargs)


    def init_server(self, server: InteractiveMarkerServer):
        if self.server is None:
            self.server = server
            self._setup_later()
            del self._setup_later
            self.server.applyChanges()
        else:
            pass #Warning, server already set

    @abstractmethod
    def _setup_marker(self, name, pose, scale, **kwargs):
        pass

    @abstractmethod
    def provided_targets():
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

    def _create_interactive_marker(
        self, name, scale=1, pose=tf.translation_matrix([0, 0, 0]), callback=None
    ):
        im = InteractiveMarker(name=name)
        im.header.frame_id = "world"
        im.pose = createPose(pose)
        im.scale = scale

        self.server.insert(im, callback)
        return im

    @staticmethod
    def _add_display_marker(im, name, marker=None, markers=None):
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
    def _add_movement_marker(im, name, mode, marker=None, markers=None):
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
            control.orientation.x = 1 if d == "x" else 0
            control.orientation.z = 1 if d == "y" else 0
            control.orientation.y = 1 if d == "z" else 0
            control.orientation.w = 1

            if marker:
                control.markers.append(marker)
            if markers:
                control.markers.extend(markers)

            im.controls.append(control)

class SingelControlMarker(IAMarker, ABC):
    def __init__(self, server: InteractiveMarkerServer = None, name: str = "Marker", pose=None, scale: int = 1, callback=None, additional_marker=None) -> None:
        super().__init__(server, name, pose, scale, callback, additional_marker=additional_marker)

    @abstractmethod
    def _setup_marker(self, name, pose, scale, additional_marker=None):
        self.marker = self._create_interactive_marker(
            name, scale=scale, pose=pose, callback=self._callback
        )

        if additional_marker:
            if isinstance(additional_marker, list):
                for x in additional_marker:
                    self._add_display_marker(self.marker, "", x)
            else:
                self._add_display_marker(self.marker, "", additional_marker)
        
        self._data_callback(self._name, pose)


    def delete(self):
        self.server.erase(self._name)
        self.server.applyChanges()

    def provided_targets(self):
        return [self._name]

    def _callback(self, fb: InteractiveMarkerFeedback):
        self._data_callback(fb.marker_name, poseMsgToTM(fb.pose))
        self.server.applyChanges()

class PositionMarker(SingelControlMarker):
    def _setup_marker(self, name, pose, scale, additional_marker=None):
        super()._setup_marker(name, pose, scale, additional_marker=additional_marker)
        self._add_movement_marker(self.marker, "", InteractiveMarkerControl.MOVE_3D, sphere())
        self._add_movement_control(self.marker, "", InteractiveMarkerControl.MOVE_AXIS)

class OrientationMarker(SingelControlMarker):

    def _setup_marker(self, name, pose, scale, additional_marker=None):
        super()._setup_marker(name, pose, scale, additional_marker=additional_marker)
        self._add_movement_marker(self.marker, "", InteractiveMarkerControl.ROTATE_3D, sphere())
        self._add_movement_control(self.marker, "", InteractiveMarkerControl.ROTATE_AXIS)


class SixDOFMarker(SingelControlMarker):
    def _setup_marker(self, name, pose, scale, additional_marker=None):
        super()._setup_marker(name, pose, scale, additional_marker=additional_marker)
        self._add_movement_marker(self.marker, "", InteractiveMarkerControl.MOVE_ROTATE_3D, sphere())
        self._add_movement_control(self.marker, "", InteractiveMarkerControl.MOVE_AXIS)
        self._add_movement_control(self.marker, "", InteractiveMarkerControl.ROTATE_AXIS)

class ConeMarker(IAMarker):
    def __init__(
        self,
        server: InteractiveMarkerServer,
        pose,
        name: str = "Cone",
        scale: float = 1,
        angle=0.4,
        mode=InteractiveMarkerControl.ROTATE_3D,
    ) -> None:
        super().__init__(server, pose=pose, name=name, scale=scale, angle=angle, mode=mode)

    def _setup_marker(self, name, pose, scale, angle, mode):
        self._angle = angle
        self._scale = scale

        loc_marker = self._create_interactive_marker(
            f"{self._name}_Pos", pose=pose, scale=0.1, callback=self._callback_pose
        )
        self._add_display_marker(loc_marker, "", cone(self._angle, self._scale))
        self._add_movement_control(loc_marker, "", InteractiveMarkerControl.MOVE_AXIS)
        self._add_movement_control(loc_marker, "", InteractiveMarkerControl.ROTATE_AXIS)

        handle_marker = self._create_interactive_marker(
            f"{self._name}_Handle", callback=self._callback_angel, scale=0.05
        )
        self._add_movement_marker(
            handle_marker, "", InteractiveMarkerControl.MOVE_PLANE, marker=sphere()
        )
        self._add_movement_control(
            handle_marker, "", InteractiveMarkerControl.MOVE_AXIS, directoins="z"
        )
        self._add_movement_control(
            handle_marker, "", InteractiveMarkerControl.MOVE_AXIS, directoins="y"
        )

        self._data_callback(f"{self._name}_angle", self._angle)
        self._data_callback(f"{self._name}_pose", pose)

    def provided_targets(self):
        return ["{self._name}_Pos","{self._name}_Handle"]

    def delete(self):
        self.server.erase(f"{self._name}_Pos")
        self.server.erase(f"{self._name}_Handle")
        self.server.applyChanges()

    def _calc_handle_pose(self, T_root):
        handle_pose = tf.rotation_matrix(self._angle, [1, 0, 0]).dot(
            tf.translation_matrix([0, 0, self._scale])
        )
        self.server.setPose(f"{self._name}_Handle", createPose(T_root.dot(handle_pose)))

    def _callback_pose(self, feedback):
        T = poseMsgToTM(feedback.pose)
        self._calc_handle_pose(T)
        self._data_callback(f"{self._name}_pose", T)
        self.server.applyChanges()

    def _callback_angel(self, feedback):
        T_marker = poseMsgToTM(feedback.pose)
        coneM = self._get_marker(f"{self._name}_Pos")
        T_cone = poseMsgToTM(coneM.pose)

        v = T_marker[0:3, 3] - T_cone[0:3, 3]  # vector from cone's origin to marker
        self._scale = numpy.linalg.norm(v)
        self._angle = numpy.arccos(
            numpy.maximum(0, T_cone[0:3, 2].dot(v) / self._scale)
        )

        self._calc_handle_pose(T_cone)
        coneM.controls[0].markers[0].points = cone(self._angle, self._scale).points
        self._update_marker(coneM)
        self._data_callback(f"{self._name}_angle", self._angle)
        self.server.applyChanges()


def sphere(radius=0.02, color=ColorRGBA(1, 0, 1, 0.5), **kwargs):
    """Create a sphere marker"""
    scale = Vector3(radius, radius, radius)
    return Marker(type=Marker.SPHERE, scale=scale, color=color, **kwargs)


def cylinder(radius=0.02, len=0.1, color=ColorRGBA(1, 0, 0, 1), **kwargs):
    """Create a cylinder marker"""
    scale = Vector3(radius, radius, len)
    return Marker(type=Marker.CYLINDER, scale=scale, color=color, **kwargs)


def box(size=Vector3(0.1, 0.1, 0.1), color=ColorRGBA(1, 1, 1, 0.5), **kwargs):
    """Create a box marker"""
    return Marker(type=Marker.CUBE, scale=size, color=color, **kwargs)


def plane(size=1.0, color=ColorRGBA(1, 1, 1, 0.5), **kwargs):
    """Create a plane (a flat box)"""
    return box(size=Vector3(size, size, 1e-3), color=color, **kwargs)


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
        points.append(
            Point(radius * numpy.sin(theta), radius * numpy.cos(theta), height)
        )
        theta = twopi * (i + 1) / numTriangles
        points.append(
            Point(radius * numpy.sin(theta), radius * numpy.cos(theta), height)
        )

        # inside
        rinside = radius * 0.95
        theta = twopi * i / numTriangles
        points.append(
            Point(rinside * numpy.sin(theta), rinside * numpy.cos(theta), height)
        )
        points.append(Point(0, 0, radius - rinside))
        theta = twopi * (i + 1) / numTriangles
        points.append(
            Point(rinside * numpy.sin(theta), rinside * numpy.cos(theta), height)
        )

    return Marker(
        type=Marker.TRIANGLE_LIST,
        points=points,
        color=color,
        scale=Vector3(1, 1, 1),
        **kwargs,
    )


def arrow(len=0.1, width=None, color=ColorRGBA(1, 0, 0, 1), **kwargs):
    """Create an arrow marker"""
    width = width or 0.1 * len
    scale = Vector3(len, width, width)
    return Marker(type=Marker.ARROW, scale=scale, color=color, **kwargs)


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

    m = cylinder(radius, scale, color=ColorRGBA(1, 0, 0, 1), id=0, **defaults)
    q = tf.quaternion_multiply(tf.quaternion_from_matrix(T), xaxis)
    m.pose.orientation = Quaternion(*q)
    m.pose.position = Point(*(p + tf.quaternion_matrix(q)[:3, :3].dot(offset)))
    markers.append(m)

    m = cylinder(radius, scale, color=ColorRGBA(0, 1, 0, 1), id=1, **defaults)
    q = tf.quaternion_multiply(tf.quaternion_from_matrix(T), yaxis)
    m.pose.orientation = Quaternion(*q)
    m.pose.position = Point(*(p + tf.quaternion_matrix(q)[:3, :3].dot(offset)))
    markers.append(m)

    m = cylinder(radius, scale, color=ColorRGBA(0, 0, 1, 1), id=2, **defaults)
    m.pose.orientation = Quaternion(*tf.quaternion_from_matrix(T))
    m.pose.position = Point(*(p + T[:3, :3].dot(offset)))
    markers.append(m)
    return markers


def add3DControls(im, markers, mode=InteractiveMarkerControl.MOVE_ROTATE_3D, **kwargs):
    # Create a control to move a (sphere) marker around with the mouse
    control = InteractiveMarkerControl(interaction_mode=mode, markers=markers, **kwargs)
    im.controls.append(control)


def addArrowControls(im, dirs="xyz"):
    # Create arrow controls to move the marker
    for dir in dirs:
        control = InteractiveMarkerControl()
        control.name = "move_" + dir
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation.x = 1 if dir == "x" else 0
        control.orientation.z = 1 if dir == "y" else 0
        control.orientation.y = 1 if dir == "z" else 0
        control.orientation.w = 1
        im.controls.append(control)


def addOrientationControls(im, dirs="xyz"):
    # Create controls to rotate the marker
    for dir in dirs:
        control = InteractiveMarkerControl()
        control.name = "rotate_" + dir
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation.x = 1 if dir == "x" else 0
        control.orientation.z = 1 if dir == "y" else 0
        control.orientation.y = 1 if dir == "z" else 0
        control.orientation.w = 1
        im.controls.append(control)


def createPose(T):
    if T.shape != (4, 4):  # if not 4x4 matrix: assume position vector
        Tnew = numpy.identity(4)
        Tnew[0:3, 3] = T
        T = Tnew
    return Pose(
        position=Point(*T[0:3, 3]),
        orientation=Quaternion(*tf.quaternion_from_matrix(T)),
    )


def addMarker(im_server, im, feedback_callback):
    # call feedback callback once to initialize target
    feedback_callback(InteractiveMarkerFeedback(marker_name=im.name, pose=im.pose))
    im_server.insert(im, feedback_callback)


def poseMsgToTM(pose):
    q = pose.orientation
    p = pose.position
    T = tf.quaternion_matrix(numpy.array([q.x, q.y, q.z, q.w]))
    T[0:3, 3] = numpy.array([p.x, p.y, p.z])
    return T


def processFeedback(pose_callback):
    def process_marker_feedback(feedback):
        pose_callback(feedback.marker_name, poseMsgToTM(feedback.pose))

    return process_marker_feedback


def iMarker(
    T, markers=[], name="pose", mode=InteractiveMarkerControl.MOVE_ROTATE_3D, **kwargs
):
    im = InteractiveMarker(name=name, pose=createPose(T), **kwargs)
    im.header.frame_id = "world"
    if markers:
        add3DControls(im, markers, mode=mode)
    return im


def iPositionMarker(T, markers=[sphere()], name="pos", **kwargs):
    im = iMarker(T, markers, name, scale=0.2, description="Pos")
    addArrowControls(im)
    return im


def iPoseMarker(T, markers=[sphere()], name="pose"):
    im = iMarker(T, markers, name, scale=0.2, description="Pose 6D")
    addArrowControls(im)
    addOrientationControls(im)
    return im


def iPlaneMarker(pos, markers, name="plane", **kwargs):
    im = iMarker(pos, name=name, scale=0.2, description="Plane", **kwargs)
    if markers:
        add3DControls(im, markers)
    else:
        addArrowControls(im, dirs="z")
        addOrientationControls(im, dirs="xy")
    return im
