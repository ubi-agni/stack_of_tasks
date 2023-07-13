from __future__ import annotations

from traits.api import Dict, HasTraits, observe
from traits.observation.events import TraitChangeEvent

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarkerFeedback

from stack_of_tasks.utils.traits import BaseSoTHasTraits

from .abstract_marker import IAMarker


class MarkerServer(BaseSoTHasTraits):
    marker = Dict(str, IAMarker)
    _control = Dict(str, IAMarker)

    def __init__(self, namespace="controller") -> None:
        super().__init__()
        self._ims = InteractiveMarkerServer(namespace)

    def add_marker(self, marker: IAMarker):
        self.marker[marker.name] = marker

        for m in marker.markers:
            self._control[m.name] = marker
            self._ims.insert(m, self._server_cb)
        self._ims.applyChanges()

    @observe("marker.items:sync")
    def _sync(self, evt: TraitChangeEvent):
        # self._ims.setPose()
        for marker in self.marker[evt.new].markers:
            self._ims.setPose(marker.name, marker.pose)
            self._ims.insert(marker, self._server_cb)
        self._ims.applyChanges()
        # print(evt)

    def _server_cb(self, fb: InteractiveMarkerFeedback):
        self._control[fb.marker_name].feedback(fb)

    @observe("marker.items:name")
    def _marker_name_changed(self, evt: TraitChangeEvent):
        self.marker[evt.new] = (marker := self.marker.pop(evt.old))
        self._ims.erase(evt.old)
        self._ims.insert(marker.marker, self._server_cb)
        self._ims.applyChanges()
