from abc import ABC, abstractmethod
from typing import Union

from interactive_markers.interactive_marker_server import (
    InteractiveMarkerFeedback,
    InteractiveMarkerServer,
)
from visualization_msgs.msg import InteractiveMarkerControl

from stack_of_tasks.ref_frame.frames import RefFrame, Transform, World
from stack_of_tasks.ref_frame.offset import OffsetRefFrame
from stack_of_tasks.utils import OffsetTransform as OT_Old
from stack_of_tasks.utils import create_pose, pose_to_matrix

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
        self.marker.pose = create_pose(transform)
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
