import rospy
import piexif
import cv2
import os
from datetime import datetime
from clover import srv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from fractions import Fraction
from .camera_utils.recorder import Recorder
from .files.filemanager import FileManager, Extension

class Camera():
    def __init__(self, topic: str = 'main_camera/image_raw_throttled') -> None:
        self.bridge = CvBridge()
        self.topic = topic
        self.recorder = Recorder(self.topic)
        self.filemanager = FileManager()

    def set_topic(self, topic: str) -> None:
        '''
        Set the topic that camera is looking at.
        '''

        self.topic = topic
        self.recorder.topic = topic
        self.recorder.sync_fps()

    def retrieve_cv_frame(self):
        '''
        Retrieve a single frame.
        '''

        return self.bridge.imgmsg_to_cv2(rospy.wait_for_message(self.topic, Image), 'bgr8')

    def save_image(self, extension: Extension = Extension.IMG_JPEG) -> None:
        '''
        Save image to a jpeg file.
        '''

        filename = self.filemanager.filename(extension)

        frame = self.retrieve_cv_frame()
        cv2.imwrite(filename, frame)

        self.filemanager.add_metadata(filename)

    def publish_image(self, frame, node_name: str) -> None:
        '''
        Publish an image to a node.
        '''

        image_pub = rospy.Publisher(f'~camera/{node}', Image, queue_size=1)
        image_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))

    def record(self):
        '''
        Start recording.
        '''

        self.recorder.record()

    def stop(self):
        '''
        Stop recording.
        '''

        self.recorder.stop()
