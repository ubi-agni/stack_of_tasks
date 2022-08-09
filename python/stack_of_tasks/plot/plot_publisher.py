
from rospy import Publisher, Time
from stack_of_tasks.msg import Plot

class PlotPublisher:

    def __init__(self, prefix="plot") -> None:
        self.prefix = prefix
        self.pps = {}

    

    def plot(self, values, topic):
            pb:Publisher = self.pps.setdefault(
                topic, Publisher(f"/{self.prefix}/{topic}", Plot, queue_size=1)
            )
            p = Plot()
            p.stamp = Time.now()
            p.data = values
            pb.publish(p)
            
