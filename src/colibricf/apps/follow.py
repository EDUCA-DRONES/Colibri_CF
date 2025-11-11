
import time
from math import trunc
import cv2 as cv
import mediapipe as mp
from ..cv.utils import draw_landmarks_on_image
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from clover import long_callback
from .handlers.follow import follow_handle_move

import rospy
import os

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
model_path= os.path.join(BASE_DIR, '..', 'cv', 'models', 'pose_landmarker_full.task')
BaseOptions = mp.tasks.BaseOptions
PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarkerOptions
VisionRunningMode = mp.tasks.vision.RunningMode
detection_results = None
bridge = CvBridge()

def result_callback(result, output_image, timestamp_ms):
    detection_results = result

options = PoseLandmarkerOptions(
    base_options=BaseOptions(model_asset_path=model_path),
    running_mode=VisionRunningMode.LIVE_STREAM,
    num_poses=1,
    min_pose_detection_confidence=0.65, result_callback=result_callback)

landmarker = mp.tasks.vision.PoseLandmarker.create_from_options(options)
image_pub = rospy.Publisher('~follow/debug', Image, queue_size=1)

def set_target_size(result, h):
    try:
        pose_landmarks = result.pose_landmarks[0]
        y0 = pose_landmarks[0].y
        y1 = pose_landmarks[23].y
    except (IndexError, TypeError):
        return None
    return (y1 - y0) * h

@long_callback
def _follow_callback(data):
    frame = bridge.imgmsg_to_cv2(data,'bgr8')

    frame = cv.flip(frame, 1)

    frame_timestamp_ms = int(time.time() * 1000)
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame)
    landmarker.detect_async(mp_image, frame_timestamp_ms)

    if detection_results is not None:
        h, w, _ = frame.shape
        center = trunc(w / 2)
        target_size = set_target_size(detection_results, h)

        if target_size is not None:
            landmark0 = detection_results.pose_landmarks[0][0]
            centralize_in_target(center, landmark0, w)
            follow_handle_move(target_size)

        annotated_frame= draw_landmarks_on_image(frame, detection_results)
        frame = cv.cvtColor(annotated_frame, cv.COLOR_RGB2BGR)

    image_pub.publish(bridge.cv2_to_imgmsg(frame, 'bgr8'))
