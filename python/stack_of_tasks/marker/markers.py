from abc import ABC, abstractmethod
from typing import Union

from interactive_markers.interactive_marker_server import (
    InteractiveMarkerFeedback,
    InteractiveMarkerServer,
)
from tf import transformations as tf
from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import InteractiveMarkerControl

from stack_of_tasks.utils import OffsetTransform

from .interactive_marker import IAMarker


class ControlMarker(IAMarker, ABC):
    def __init__(
        self,
        server: InteractiveMarkerServer = None,
        name: str = "Marker",
        pose=tf.translation_matrix([0, 0, 0]),
        scale: int = 1,
        callback=None,
        additional_marker=None,
    ) -> None:
        super().__init__(
            server, name, pose, scale, callback, additional_marker=additional_marker
        )

    @abstractmethod
    def _setup_marker(
        self, name: str, pose: Union[Pose, PoseStamped], scale: float, additional_marker=None
    ):
        self.marker = self._create_interactive_marker(
            self.server, name, scale=scale, pose=pose, callback=self._callback
        )

        if additional_marker:
            if isinstance(additional_marker, list):
                for x in additional_marker:
                    self._add_display_marker(self.marker, "", x)
            else:
                self._add_display_marker(self.marker, "", additional_marker)

        self._data_callback(
            self.name, OffsetTransform(self.marker.header.frame_id, self.marker.pose)
        )

    def delete(self):
        self.server.erase(self.name)
        self.server.applyChanges()

    def provided_targets(self):
        return [self.name]

    def _callback(self, fb: InteractiveMarkerFeedback):
        self._data_callback(fb.marker_name, OffsetTransform(fb.header.frame_id, fb.pose))
        self.server.applyChanges()


class PositionMarker(ControlMarker):
    def _setup_marker(self, name, pose, scale, additional_marker=None):
        super()._setup_marker(name, pose, scale, additional_marker=additional_marker)
        self._add_movement_marker(
            self.marker, "", InteractiveMarkerControl.MOVE_3D, self.sphere()
        )
        self._add_movement_control(self.marker, "", InteractiveMarkerControl.MOVE_AXIS)


class OrientationMarker(ControlMarker):
    def _setup_marker(self, name, pose, scale, additional_marker=None):
        super()._setup_marker(name, pose, scale, additional_marker=additional_marker)
        self._add_movement_marker(
            self.marker, "", InteractiveMarkerControl.ROTATE_3D, self.sphere()
        )
        self._add_movement_control(self.marker, "", InteractiveMarkerControl.ROTATE_AXIS)

        for c in self.marker.controls:
            print("controls: ", c.interaction_mode)


class SixDOFMarker(ControlMarker):
    def _setup_marker(self, name, pose, scale, additional_marker=None):
        super()._setup_marker(name, pose, scale, additional_marker=additional_marker)
        self._add_movement_marker(
            self.marker, "", InteractiveMarkerControl.MOVE_ROTATE_3D, self.sphere()
        )
        self._add_movement_control(self.marker, "", InteractiveMarkerControl.MOVE_AXIS)
        self._add_movement_control(self.marker, "", InteractiveMarkerControl.ROTATE_AXIS)
