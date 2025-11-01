import rospy
from pyzbar import pyzbar
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from clover import long_callback

class Camera():
    def __init__(self):
        rospy.init_node('cv')
        self.bridge = CvBridge()

    def retrieve_cv_frame(self):
        '''
        Retrieve a single frame.
        '''

        return self.bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')

    @long_callback
    def _qrcode_callback(self, msg):
        '''
        Read a qrcode
        '''

        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        barcodes = pyzbar.decode(img)
        for barcode in barcodes:
            b_data = barcode.data.decode('utf-8')
            b_type = barcode.type
            (x, y, w, h) = barcode.rect
            xc = x + w/2
            yc = y + h/2
            print('Found {} with data {} with center at x={}, y={}'.format(b_type, b_data, xc, yc))

    def get_qrcode_sub(self):
        '''
        Return a qrcode reader.
        '''

        image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, self._qrcode_callback, queue_size=1)
        return image_sub

