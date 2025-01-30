from __future__ import annotations

import threading
import types
import weakref
from threading import Lock

from traits.api import Dict, observe
from traits.observation.events import TraitChangeEvent

import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from rospy import Rate
from rospy.timer import TimerEvent, time
from visualization_msgs.msg import (
    InteractiveMarkerFeedback,
    InteractiveMarkerInit,
    InteractiveMarkerUpdate,
)

from stack_of_tasks.marker.abstract_marker import IAMarker
from stack_of_tasks.utils.traits import BaseSoTHasTraits


class WeakRefTimer(threading.Thread):
    """
    Convenience class for calling a callback at a specified rate

    This class is a fixed version of the Timer class in rospy.
    It stores the callback in a weakref to prevent reference cycles.
    """

    def __init__(self, period, callback, oneshot=False, reset=False):
        """
        Constructor.
        @param period: desired period between callbacks
        @type  period: rospy.Duration
        @param callback: callback to be called
        @type  callback: function taking rospy.TimerEvent
        @param oneshot: if True, fire only once, otherwise fire continuously until shutdown is called [default: False]
        @type  oneshot: bool
        @param reset: if True, timer is reset when rostime moved backward. [default: False]
        @type  reset: bool
        """
        super(WeakRefTimer, self).__init__()
        self._period = period

        if isinstance(callback, types.MethodType):
            self._callback = weakref.WeakMethod(callback)
        else:
            self._callback = weakref.ref(callback)

        self._oneshot = oneshot
        self._reset = reset
        self._shutdown = threading.Event()
        self.daemon = True
        self.start()

    def __del__(self):
        self.shutdown()

    def shutdown(self):
        self._shutdown.set()

    def run(self):
        r = Rate(1.0 / self._period.to_sec(), reset=self._reset)
        current_expected = rospy.rostime.get_rostime() + self._period
        last_expected, last_real, last_duration = None, None, None

        while not rospy.core.is_shutdown() and not self._shutdown.is_set():
            try:
                r.sleep()
            except rospy.exceptions.ROSInterruptException as e:
                if rospy.core.is_shutdown():
                    break
                raise

            if self._shutdown.is_set():
                break

            current_real = rospy.rostime.get_rostime()
            start = time.time()

            try:
                self._callback()(
                    TimerEvent(
                        last_expected,
                        last_real,
                        current_expected,
                        current_real,
                        last_duration,
                    )
                )
            except TypeError:
                break

            if self._oneshot:
                break
            last_duration = time.time() - start
            last_expected, last_real = current_expected, current_real
            current_expected += self._period


class FixedInteractiveMarkerServer(InteractiveMarkerServer):
    ## @brief Create an InteractiveMarkerServer and associated ROS connections
    ## @param topic_ns      The interface will use the topics topic_ns/update and
    ##                      topic_ns/feedback for communication.
    ## @param server_id     If you run multiple servers on the same topic from
    ##                      within the same node, you will need to assign different names to them.
    ##                      Otherwise, leave this empty.
    def __init__(self, topic_ns, server_id="", q_size=100):
        self.topic_ns = topic_ns
        self.seq_num = 0
        self.mutex = Lock()

        self.server_id = rospy.get_name() + server_id

        # contains the current state of all markers
        # string : MarkerContext
        self.marker_contexts = dict()

        # updates that have to be sent on the next publish
        # string : UpdateContext
        self.pending_updates = dict()

        self.init_pub = rospy.Publisher(
            topic_ns + "/update_full", InteractiveMarkerInit, latch=True, queue_size=100
        )
        self.update_pub = rospy.Publisher(topic_ns + "/update", InteractiveMarkerUpdate, queue_size=100)

        self.sc = rospy.Subscriber(
            topic_ns + "/feedback",
            InteractiveMarkerFeedback,
            self.processFeedback,
            queue_size=q_size,
        )

        WeakRefTimer(rospy.Duration(0.5), self.keepAlive, reset=True)


class MarkerServer(BaseSoTHasTraits):
    marker: dict = Dict(str, IAMarker)
    _control: dict = Dict(str, IAMarker)

    def __init__(self, namespace="controller") -> None:
        super().__init__()
        self._ims = FixedInteractiveMarkerServer(namespace)

    def __del__(self):
        self._ims.sc.unregister()  # the subscriber needs to be unregistered explicitly here.

    def add_marker(self, marker: IAMarker):
        self.marker[marker.name] = marker

        for m in marker.markers:
            self._control[m.name] = marker
            self._ims.insert(m, self._server_cb)
        self._ims.applyChanges()

    def remove(self, marker: IAMarker):
        for control in marker.markers:
            self._ims.erase(control.name)
            self._control.pop(control.name)

        self._ims.applyChanges()
        self.marker.pop(marker.name)

    @observe("marker.items:sync")
    def _sync(self, evt: TraitChangeEvent):
        for marker in self.marker[evt.new].markers:
            self._ims.setPose(marker.name, marker.pose)
            self._ims.insert(marker, self._server_cb)
        self._ims.applyChanges()

    def _server_cb(self, fb: InteractiveMarkerFeedback):
        self._control[fb.marker_name].feedback(fb)

    @observe("marker.items:name")
    def _marker_name_changed(self, evt: TraitChangeEvent):
        self.marker[evt.new] = (marker := self.marker.pop(evt.old))
        self._ims.erase(evt.old)
        self._ims.insert(marker.marker, self._server_cb)
        self._ims.applyChanges()
