import cv2
import rospy
from . import utils
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from clover import long_callback
import os


def detectFace(frame):
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    cascade_path = os.path.join(BASE_DIR, 'haar_cascade', 'haar_cascade.xml')

    haar_cascade_face = cv2.CascadeClassifier(cascade_path)
    matrix = haar_cascade_face.detectMultiScale(frame, scaleFactor=1.4, minNeighbors=4)
    return matrix

@long_callback
def _draw_face_callback(data):
    '''
    Draw a face in image and publish
    '''

    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(data, 'bgr8')
    frame = cv2.flip(frame, 1)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    face_rect = detectFace(gray)

    for (x, y, w, h) in face_rect:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), thickness=2)
        cv2.putText(frame, f'face ({x},{y})', (x , y - 20), cv2.FONT_HERSHEY_PLAIN, 1.1, (0, 255, 0), 2)

    resized = utils.resize(frame)

    image_pub = rospy.Publisher('~face_detect', Image, queue_size=1)
    image_pub.publish(bridge.cv2_to_imgmsg(resized, 'bgr8'))
