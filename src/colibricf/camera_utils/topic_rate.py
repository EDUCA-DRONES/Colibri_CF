import rospy
import time
from sensor_msgs.msg import Image

class TopicRater:
    def __init__(self, topic_name, sample_size=30):
        self.topic_name = topic_name
        self.sample_size = sample_size
        self.times = []

    def callback(self, msg):
        now = time.time()
        self.times.append(now)
        if len(self.times) > self.sample_size:
            self.times.pop(0)

    def get_rate(self):
        if len(self.times) < 2:
            return 0.0
        intervals = [t2 - t1 for t1, t2 in zip(self.times[:-1], self.times[1:])]
        avg_interval = sum(intervals) / len(intervals)
        return 1.0 / avg_interval
