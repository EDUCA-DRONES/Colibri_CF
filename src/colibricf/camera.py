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

class Camera():
    def __init__(self, topic: str = 'main_camera/image_raw_throttled') -> None:
        self.bridge = CvBridge()
        self.topic = topic
        self.recorder = Recorder(self.topic)

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

    def save_image(self, path:str) -> None:
        '''
        Save image to a jpeg file.
        '''

        timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        filename = os.path.join(path, 'clover-' + timestamp + '.jpg')

        frame = self.retrieve_cv_frame()
        cv2.imwrite(filename, frame)

        telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)()

        lat = telemetry.lat
        lon = telemetry.lon
        alt = telemetry.alt

        def to_dms(value: float):
            abs_degrees = abs(value)
            degress = int(abs_degrees)
            minutes = int((abs_degrees - degress) * 60)
            seconds = int(((abs_degrees - degress) * 60 - minutes) * 60 * 10000)

            return ((degress, 1), (minutes, 1), (seconds, 10000))

        exif_dict = {
            "GPS": {
                piexif.GPSIFD.GPSLatitudeRef: b"N" if lat >= 0 else b"S",
                piexif.GPSIFD.GPSLatitude: to_dms(lat),
                piexif.GPSIFD.GPSLongitudeRef: b"E" if lon >= 0 else b"W",
                piexif.GPSIFD.GPSLongitude: to_dms(lon),
                piexif.GPSIFD.GPSAltitude: (int(alt * 1000), 1000),
                piexif.GPSIFD.GPSAltitudeRef: 0,
            }
        }

        exif_bytes = piexif.dump(exif_dict)
        piexif.insert(exif_bytes, filename)

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
