from abc import ABC, abstractmethod

import numpy as np

import tf.transformations as tf
from interactive_markers.interactive_marker_server import (
    InteractiveMarkerFeedback,
    InteractiveMarkerServer,
)
from visualization_msgs.msg import InteractiveMarkerControl

from stack_of_tasks.ref_frame.frames import RefFrame, Transform, World
from stack_of_tasks.ref_frame.offset import OffsetRefFrame
from stack_of_tasks.utils.tf_mappings import matrix_to_pose, pose_to_matrix

from .interactive_marker import IAMarker


class ControlMarker(IAMarker, ABC):
    def __init__(
        self,
        server: InteractiveMarkerServer = None,
        name: str = "Marker",
        callback=None,
        frame: RefFrame = World(),
        offset: Transform = None,
        scale: int = 1,
        additional_marker=None,
    ) -> None:
        super().__init__(
            server,
            name,
            callback,
            frame=frame,
            offset=offset,
            scale=scale,
            additional_marker=additional_marker,
        )

    @abstractmethod
    def _setup_marker(
        self,
        name: str,
        frame: RefFrame,
        offset: Transform,
        scale: float,
        additional_marker=None,
    ):

        self.ref_frame = OffsetRefFrame(frame, offset)

        self.marker = self._create_interactive_marker(
            self.server,
            name,
            scale=scale,
            pose=self.ref_frame.offset,
            callback=self._callback,
        )

        self.ref_frame.callback.append(self._set_marker_pose)

        if additional_marker:
            if isinstance(additional_marker, list):
                for x in additional_marker:
                    self._add_display_marker(self.marker, "", x)
            else:
                self._add_display_marker(self.marker, "", additional_marker)

        # self._data_callback(
        #    self.name, OffsetTransform(self.marker.header.frame_id, self.marker.pose)
        # )

    def _set_marker_pose(self, transform):
        self.marker.pose = matrix_to_pose(transform)
        self.server.applyChanges()

    def delete(self):
        self.server.erase(self.name)
        self.server.applyChanges()

    def provided_targets(self):
        return []

    def _callback(self, fb: InteractiveMarkerFeedback):
        self.ref_frame.offset = pose_to_matrix(fb.pose)
        self.server.applyChanges()


class PositionMarker(ControlMarker):
    def _setup_marker(
        self,
        name: str,
        frame: RefFrame,
        offset: Transform,
        scale: float,
        additional_marker=None,
    ):

        super()._setup_marker(
            name=name,
            frame=frame,
            offset=offset,
            scale=scale,
            additional_marker=additional_marker,
        )
        self._add_movement_marker(
            self.marker, "", InteractiveMarkerControl.MOVE_3D, self.sphere()
        )
        self._add_movement_control(self.marker, "", InteractiveMarkerControl.MOVE_AXIS)


class OrientationMarker(ControlMarker):
    def _setup_marker(
        self,
        name: str,
        frame: RefFrame,
        offset: Transform,
        scale: float,
        additional_marker=None,
    ):

        super()._setup_marker(
            name=name,
            frame=frame,
            offset=offset,
            scale=scale,
            additional_marker=additional_marker,
        )
        self._add_movement_marker(
            self.marker, "", InteractiveMarkerControl.ROTATE_3D, self.sphere()
        )
        self._add_movement_control(self.marker, "", InteractiveMarkerControl.ROTATE_AXIS)

        for c in self.marker.controls:
            print("controls: ", c.interaction_mode)


class SixDOFMarker(ControlMarker):
    def _setup_marker(
        self,
        name: str,
        frame: RefFrame,
        offset: Transform,
        scale: float,
        additional_marker=None,
    ):

        super()._setup_marker(
            name=name,
            frame=frame,
            offset=offset,
            scale=scale,
            additional_marker=additional_marker,
        )
        self._add_movement_marker(
            self.marker, "", InteractiveMarkerControl.MOVE_ROTATE_3D, self.sphere()
        )
        self._add_movement_control(self.marker, "", InteractiveMarkerControl.MOVE_AXIS)
        self._add_movement_control(self.marker, "", InteractiveMarkerControl.ROTATE_AXIS)


class ConeMarker(IAMarker):
    def __init__(
        self,
        server: InteractiveMarkerServer = None,
        pose=tf.translation_matrix([0, 0, 0]),
        name: str = "Cone",
        scale: float = 1,
        angle=0.4,
        mode=InteractiveMarkerControl.ROTATE_3D,
    ) -> None:
        super().__init__(
            server=server, name=name, pose=pose, scale=scale, angle=angle, mode=mode
        )

        self._angle: int
        self._scale: int

    def _setup_marker(self, name, pose, scale, angle, mode):
        self._angle = angle
        self._scale = scale

        loc_marker = self._create_interactive_marker(
            self.server,
            f"{self.name}_Pos",
            pose=pose,
            scale=0.1,
            callback=self._callback_pose,
        )

        self._add_display_marker(loc_marker, "", IAMarker.cone(self._angle, self._scale))
        self._add_movement_control(loc_marker, "", InteractiveMarkerControl.MOVE_AXIS)
        self._add_movement_control(loc_marker, "", InteractiveMarkerControl.ROTATE_AXIS)

        handle_marker = self._create_interactive_marker(
            self.server,
            f"{self.name}_Handle",
            scale=0.05,
            callback=self._callback_angel,
            pose=tf.translation_matrix([0, 0, 0]),
        )
        self._add_movement_marker(
            handle_marker, "", InteractiveMarkerControl.MOVE_PLANE, marker=self.sphere()
        )
        self._add_movement_control(
            handle_marker, "", InteractiveMarkerControl.MOVE_AXIS, directions="z"
        )
        self._add_movement_control(
            handle_marker, "", InteractiveMarkerControl.MOVE_AXIS, directions="y"
        )
        self.server.applyChanges()
        self._calc_handle_pose(pose)

        self._data_callback(f"{self.name}_pose", pose)
        self._data_callback(f"{self.name}_angle", self._angle)

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
        self.server.setPose(f"{self.name}_Handle", matrix_to_pose(T_root.dot(handle_pose)))
        self.server.applyChanges()

    def _callback_pose(self, feedback):
        T = pose_to_matrix(feedback.pose)
        self._calc_handle_pose(T)
        self._data_callback(f"{self.name}_pose", T)
        self.server.applyChanges()

    def _callback_angel(self, feedback):
        T_marker = pose_to_matrix(feedback.pose)
        coneM = self._get_marker(f"{self.name}_Pos")
        T_cone = pose_to_matrix(coneM.pose)

        # vector from cone's origin to marker
        v = T_marker[0:3, 3] - T_cone[0:3, 3]
        self._scale = np.linalg.norm(v)
        self._angle = np.arccos(np.maximum(0, T_cone[0:3, 2].dot(v) / self._scale))

        self._calc_handle_pose(T_cone)
        coneM.controls[0].markers[0].points = IAMarker.cone(self._angle, self._scale).points
        self._update_marker(coneM)
        self._data_callback(f"{self.name}_angle", self._angle)
        self.server.applyChanges()
