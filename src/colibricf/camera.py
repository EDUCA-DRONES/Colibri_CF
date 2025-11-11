import rospy
import piexif
import cv2
import os
from datetime import datetime
from clover import srv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from fractions import Fraction

class Camera():
    def __init__(self):
        self.bridge = CvBridge()

    def retrieve_cv_frame(self):
        '''
        Retrieve a single frame.
        '''

        return self.bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw_throttled', Image), 'bgr8')

    def save_image(self, path:str):
        timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        filename = os.path.join(path, timestamp + '.jpg')

        frame = self.retrieve_cv_frame()
        cv2.imwrite(filename, frame)

        telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)()

        lat = telemetry.lat
        lon = telemetry.lon
        alt = telemetry.alt

        def to_deg(value):
            frac = Fraction(value).limit_denominator()
            return ((abs(frac.numerator), abs(frac.denominator)),)

        exif_dict = {
            "GPS": {
                piexif.GPSIFD.GPSLatitudeRef: "N" if lat >= 0 else "S",
                piexif.GPSIFD.GPSLatitude: to_deg(lat),
                piexif.GPSIFD.GPSLongitudeRef: "E" if lon >= 0 else "W",
                piexif.GPSIFD.GPSLongitude: to_deg(lon),
                piexif.GPSIFD.GPSAltitude: (int(alt), 1),
                piexif.GPSIFD.GPSAltitudeRef: 0,
            }
        }

        exif_bytes = piexif.dump(exif_dict)
        piexif.insert(exif_bytes, filename)
