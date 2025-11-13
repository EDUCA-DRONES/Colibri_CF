
    def _rec_callback(self, msg):
        if self.recording:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            with self.lock:
                self.out.write(frame)
