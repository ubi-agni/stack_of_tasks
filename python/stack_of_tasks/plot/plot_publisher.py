#!/usr/bin/env python3


from time import time
from typing import List
from rospy import Publisher, Duration
from rospy.timer import Timer, TimerEvent
from plotjuggler_msgs.msg import Dictionary, DataPoints, DataPoint
from random import getrandbits


class PlotPublisher:
    def __init__(self) -> None:
        self.dict_publisher = Publisher(f"sot/names", Dictionary, queue_size=1)
        self.data_publisher = Publisher(f"sot", DataPoints, queue_size=10)
        self.plots = {}

        self.republishtimer = Timer(Duration(1.0 / 10), self._resend_dict)

    def _resend_dict(self, evt: TimerEvent):
        for d in self.plots.values():
            self.dict_publisher.publish(d)

    def _create_data_msg(self, index, time, value):
        d = DataPoint()
        d.stamp = time
        d.name_index = index
        d.value = value
        return d

    def add_plot(self, name: str, data_names: List[str]):
        dictionary = Dictionary()
        dictionary.dictionary_uuid = getrandbits(32)
        for n in data_names:
            dictionary.names.append(n)

        self.plots[name] = dictionary
        self.dict_publisher.publish(dictionary)

    def plot(self, name: str, values: List[float], timestamp=None):
        d = DataPoints()
        d.dictionary_uuid = self.plots[name].dictionary_uuid

        if timestamp is None:
            timestamp = time()

        for i, v in enumerate(values):
            d.samples.append(self._create_data_msg(i, timestamp, v))

        self.data_publisher.publish(d)


if __name__ == "__main__":
    import rospy
    from math import sin, cos

    rospy.init_node("plot_publisher")

    p = PlotPublisher()
    p.add_plot("plot", ["a/a", "a/b", "a/c"])
    rospy.sleep(0.1)
    t = 0.0
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        p.plot("plot", [cos(t), sin(t), 2 * cos(t)], t)
        t += 0.01
        rate.sleep()
