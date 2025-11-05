import time
from math import trunc, radians
import cv2 as cv
import numpy as np
import mediapipe as mp
from mediapipe.tasks.python import vision
from utils import draw_landmarks_on_image
from cv_bridge import CvBridge
from clover import long_callback
from ..drone import Drone

import rospy
import os

class Follow:
    def __init__(self):
        BASE_DIR = os.path.dirname(os.path.abspath(__file__))
        self.model_path= os.path.join(BASE_DIR, 'models', 'pose_landmarker_full.task')
        BaseOptions = mp.tasks.BaseOptions
        PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarkerOptions
        VisionRunningMode = mp.tasks.vision.RunningMode
        self.detection_results = None
        self.drone = Drone()
        self.bridge = CvBridge()

        def result_callback(result, output_image, timestamp_ms):
            self.detection_results = result

        options = PoseLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=self.model_path),
            running_mode=VisionRunningMode.LIVE_STREAM,
            num_poses=1,
            min_pose_detection_confidence=0.6,
            result_callback=result_callback)

        self.landmarker = mp.tasks.vision.PoseLandmarker.create_from_options(options)

    def set_target_size(self, result, h):
        try:
            pose_landmarks = result.pose_landmarks[0]
            y0 = pose_landmarks[0].y
            y1 = pose_landmarks[23].y
        except (IndexError, TypeError):
            # Nenhum marco detectado
            return None
        return (y1 - y0) * h

    def centralize_in_target(self, center, landmark0, w):
        pos = landmark0.x * w
        if center - 100 <= pos <= center + 100:
            self.drone.set_yaw(yaw=radians(0), frame_id='body')
        elif pos > center + 100:
            self.drone.set_yaw(yaw=radians(5), frame_id='body')
        elif pos < center - 100:
            self.drone.set_yaw(yaw=radians(-5), frame_id='body')

    def handle_move(self, target_size):
        if 250 <= target_size <= 340:
            pass
            # self.drone.navigate_wait(x=0, y=0, z=0.0, frame_id='body', speed=0.5, auto_arm=True)
        elif target_size < 250:
            self.drone.navigate_wait(x=0.1, y=0.0, z=0.0, frame_id='body', speed=0.5, auto_arm=True)
        elif target_size > 340:
            self.drone.navigate_wait(x=-0.1, y=0, z=0, frame_id='body', speed=0.5, auto_arm=True)

    @long_callback
    def _follow_callback(self, data):
        frame = self.bridge.imgmsg_to_cv2(data,'bgr8')

        frame = cv.flip(frame, 1)

        frame_timestamp_ms = int(time.time() * 1000)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame)
        self.landmarker.detect_async(mp_image, frame_timestamp_ms)

        if self.detection_results is None:
            return

        h, w, _ = frame.shape
        center = trunc(w / 2)
        target_size = self.set_target_size(self.detection_results, h)

        if target_size is not None:
            landmark0 = self.detection_results.pose_landmarks[0][0]
            self.centralize_in_target(center, landmark0, w)
            self.handle_move(target_size)

        annotated_image = draw_landmarks_on_image(frame, self.detection_results)
        annotated_image_bgr = cv.cvtColor(annotated_image, cv.COLOR_RGB2BGR)

        image_pub = rospy.Publisher('~follow/debug', Image)
        image_pub.publish(bridge.cv2_to_imgmsg(annotated_image_bgr, 'bgr8'))

    def run(self):
        rospy.Subscriber('main_camera/image_raw_throttled', Image, self._follow_callback, queue_size=1)
        rospy.spin()
