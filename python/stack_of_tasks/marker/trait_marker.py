from __future__ import annotations

import numpy
from traits.api import Range, observe
from traits.observation.events import TraitChangeEvent

from tf import transformations as tf
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import InteractiveMarkerControl, InteractiveMarkerFeedback

from stack_of_tasks.utils.tf_mappings import matrix_to_pose, pose_to_matrix

from .abstract_marker import IAMarker


class PositionMarker(IAMarker):
    def __init__(self, name, **kwargs) -> None:
        super().__init__(name, **kwargs)

        self._add_movement_marker(self.marker, "", InteractiveMarkerControl.MOVE_3D, self.sphere())

        self._add_movement_control(self.marker, "_move", InteractiveMarkerControl.MOVE_AXIS)


class OrientationMarker(IAMarker):
    def __init__(self, name, **kwargs) -> None:
        super().__init__(name, **kwargs)

        self._add_display_marker(self.marker, "_disp", self.sphere())
        self._add_movement_control(self.marker, "_rot", InteractiveMarkerControl.ROTATE_AXIS)


class FullMovementMarker(IAMarker):
    def __init__(self, name, **kwargs) -> None:
        super().__init__(name, **kwargs)

        self._add_display_marker(self.marker, "", self.sphere())
        self._add_movement_control(self.marker, "_rot", InteractiveMarkerControl.ROTATE_AXIS)
        self._add_movement_control(self.marker, "_move", InteractiveMarkerControl.MOVE_AXIS)


class PlaneMarker(IAMarker):
    def __init__(self, name, **kwargs) -> None:
        super().__init__(name, **kwargs)

        self._add_display_marker(self.marker, "", self.sphere())
        self._add_movement_marker(
            self.marker, "plane", InteractiveMarkerControl.MOVE_ROTATE_3D, self.plane()
        )


class ConeMarker(IAMarker):
    angle = Range(0.0, 1.5, value=0.2)

    def __init__(self, name, **kwargs) -> None:
        super().__init__(name, **kwargs)

        self._cone = self._add_display_marker(self.marker, "_cone", self.cone(self.angle, self.scale))
        # self._add_movement_control(self.marker, "_move", InteractiveMarkerControl.MOVE_AXIS)
        # self._add_movement_control(self.marker, "_rot", InteractiveMarkerControl.ROTATE_AXIS)

        self.handle_marker = self._create_interactive_marker(
            f"{self.name}_Handle",
            scale=0.05,
            frame_id=self.frame_id,
            pose=tf.translation_matrix([0, 0, 0]),
        )
        self._set_handle_pose()

        self._add_movement_marker(
            self.handle_marker,
            "",
            InteractiveMarkerControl.MOVE_PLANE,
            self.sphere(color=ColorRGBA(0, 1, 1, 1)),
        )

        self.markers.append(self.handle_marker)

    def feedback(self, fb: InteractiveMarkerFeedback):
        if fb.marker_name == self.marker.name:
            super().feedback(fb)
            self._set_handle_pose()
        else:
            with self.lg("angle", "scale"):
                T_marker = pose_to_matrix(fb.pose)

                # vector from cone's origin to marker
                v = T_marker[0:3, 3] - self.transform[0:3, 3]
                self.scale = numpy.linalg.norm(v)

                self.angle = numpy.arccos(numpy.maximum(0, self.transform[0:3, 2].dot(v) / self.scale))

                self._set_handle_pose()
                self._cone.markers[0].points = self.cone(self.angle, self.scale).points
                self.sync = self.marker.name

    def _set_handle_pose(self):
        handle_pose = tf.rotation_matrix(self.angle, [1, 0, 0]) @ tf.translation_matrix(
            [0, 0, self.scale]
        )

        self.handle_marker.pose = matrix_to_pose(self.transform @ (handle_pose))

    @observe("angle")
    def _angle_changed(self, evt: TraitChangeEvent):
        if "angle" not in self.lg:
            self._set_handle_pose()
            self._cone.markers[0].points = self.cone(self.angle, self.scale).points
            self.sync = self.marker.name
