import rospy
import threading
import piexif
import cv2
import os
from datetime import datetime
from clover import srv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from clover import long_callback
from fractions import Fraction

class Camera():
    def __init__(self) -> None:
        self.bridge = CvBridge()
        self.recording: bool = False
        self.lock = threading.Lock()
        self.out = None
        self.thread = None

    def retrieve_cv_frame(self) -> None:
        '''
        Retrieve a single frame.
        '''

        return self.bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw_throttled', Image), 'bgr8')

    def save_image(self, path:str) -> None:
        '''
        Save image to a jpeg file.
        '''

        timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        filename = os.path.join(path, 'clover-', timestamp + '.jpg')

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

    def _record(self):
        def _rec_callback(msg):
            if self.recording:
                frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

                if self.out is None:
                    h, w, _ = frame.shape
                    timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
                    filename = os.path.join('./', 'clover-', timestamp + '.mp4')
                    self.out = cv2.VideoWriter(filename, cv2.VideoWriter_fourcc(*"mp4v"), 30, (w, h))
                with self.lock:
                    self.out.write(frame)

        rospy.Subscriber('main_camera/image_raw_throttled', Image, _rec_callback, queue_size=1)
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

