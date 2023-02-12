from random import getrandbits
from time import time

from typing import List

from plotjuggler_msgs.msg import DataPoint, DataPoints, Dictionary

from rospy import Duration, Publisher
from rospy.timer import Timer, TimerEvent


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
