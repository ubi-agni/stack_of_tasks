from plotjuggler_msgs.msg import StatisticsNames, StatisticsValues

from rospy import Publisher

from .plot_base import PlotBase


class PlotPublisher(PlotBase):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._version = 0

        self.name_publisher = Publisher(
            f"{self._prefix}/names", StatisticsNames, latch=True, queue_size=1
        )
        self.data_publisher = Publisher(f"{self._prefix}", StatisticsValues, latch=True, queue_size=10)

    def update_names(self, _=None):
        self._version += 1
        msg = StatisticsNames()
        msg.names = self._names()
        msg.names_version = self._version

        self.name_publisher.publish(msg)

    def update_data(self, _=None):
        if self._version == 0:
            self.update_names()

        msg = StatisticsValues()
        msg.values = self._values()
        msg.names_version = self._version

        self.data_publisher.publish(msg)
