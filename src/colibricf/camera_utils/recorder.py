import rospy
import cv2
import os
import threading
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from .topic_rate import TopicRater

class Recorder:
    def __init__(self, topic: str):
        self.topic = topic
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.recording: bool = False
        self.rater = TopicRater(self.topic)
        self.FPS = None
        self.out = None
        self.thread = None
        self.sync_fps()

    def sync_fps(self) -> None:
        '''
        Sync frame rate for records acording the setted topic.
        '''

        rospy.loginfo('Syncing FPS.')
        self.rater.topic_name = self.topic
        rospy.Subscriber(self.topic, Image, self.rater.callback)

        rate = rospy.Rate(1)
        for _ in range(5): # Margin for fps aproach
            fps = int(self.rater.get_rate())
            self.FPS = fps if fps >= 5 else 5
            rate.sleep()
        rospy.loginfo(f"Fps synced to {self.FPS}.")

    def _record(self):
        def _rec_callback(msg):
            if self.recording:
                frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

                if self.out is None:
                    h, w, _ = frame.shape
                    timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
                    filename = os.path.join('./', 'clover-' + timestamp + '.mp4')
                    self.out = cv2.VideoWriter(filename, cv2.VideoWriter_fourcc(*"mp4v"), self.FPS, (w, h))
                with self.lock:
                    self.out.write(frame)

        rospy.Subscriber(self.topic, Image, _rec_callback, queue_size=1)
        rate = rospy.Rate(10)

        while not rospy.is_shutdown() and self.recording:
            rate.sleep()
        self._cleanup()

    def record(self):
        if not self.recording:
            self.recording = True
            self.thread = threading.Thread(target=self._record, daemon=True)
            self.thread.start()

    def stop(self):
        if self.recording:
            self.recording = False
            if self.thread:
                self.thread.join(timeout=2)
            self._cleanup()

    def _cleanup(self):
        with self.lock:
            if self.out:
                self.out.release()
                self.out = None

