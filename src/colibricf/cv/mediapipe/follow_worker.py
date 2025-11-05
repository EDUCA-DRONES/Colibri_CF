import rospy
import cv2
import mediapipe as mp
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from clover import long_callback

bridge = CvBridge()
rospy.init_node('follow')
face_mesh = mp.solutions.face_mesh.FaceMesh(refine_landmarks=True)
pub = rospy.Publisher('~face_position', Point, queue_size=1)

@long_callback
def _image_callback(msg):
    frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = face_mesh.process(rgb)

    if results.multi_face_landmarks:
        h, w, _ = frame.shape
        face = results.multi_face_landmarks[0]
        nose = face.landmark[1]

        pt = Point(
            x = (nose.x - 0.5),
            y = (nose.y - 0.5),
            z = nose.z
        )
        pub.publish(pt)

rospy.Subscriber('main_camera/image_raw_throttled', Image, _image_callback, queue_size=1)
rospy.spin()
