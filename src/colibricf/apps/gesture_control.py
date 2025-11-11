import cv2 as cv
import mediapipe as mp
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from clover import long_callback
from .handlers.gesture_control import handle_move, handle_fingercount

bridge = CvBridge()
image_pub = rospy.Publisher('~gesture_control/debug', Image, queue_size=1)

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
mp_styles = mp.solutions.drawing_styles
hands = mp_hands.Hands(max_num_hands=2, model_complexity=0, min_detection_confidence=0.5, min_tracking_confidence=0.5)


@long_callback
def _gc_callback(data):
    frame = bridge.imgmsg_to_cv2(data, 'bgr8')
    blank = np.zeros((500, 500, 3), dtype="uint8")

    frame = cv.flip(frame, 1)
    frame.flags.writeable = False
    frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
    results = hands.process(frame)
    frame.flags.writeable = True
    frame = cv.cvtColor(frame, cv.COLOR_RGB2BGR)

    h,w,_ = frame.shape

    points = []
    if results.multi_hand_landmarks:
        for hand_landmark in results.multi_hand_landmarks:
            mp_drawing.draw_landmarks(blank, hand_landmark, mp_hands.HAND_CONNECTIONS, mp_styles.get_default_hand_landmarks_style(), mp_styles.get_default_hand_connections_style())

            for index, cord in enumerate(hand_landmark.landmark):
                cx, cy = int(cord.x * w), int(cord.y * h)
                points.append((cx, cy))

            count = 0
            if hand_landmark:
                count = handle_fingercount(points)

            cv.putText(blank, str(count), (20, 70), cv.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255), 2)

            handle_move(count)

    image_pub.publish(bridge.cv2_to_imgmsg(frame, 'bgr8'))
