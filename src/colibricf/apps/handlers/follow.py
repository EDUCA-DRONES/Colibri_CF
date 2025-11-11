from ...drone import Drone

drone = Drone()

def follow_handle_move(target_size: float):
    if 250 <= target_size <= 340:
        pass
        # drone.navigate_wait(x=0, y=0, z=0.0, frame_id='body', speed=0.5, auto_arm=True)
    elif target_size < 250:
        drone.navigate_wait(x=0.1, y=0.0, z=0.0, frame_id='body', speed=0.5, auto_arm=True)
    elif target_size > 340:
        drone.navigate_wait(x=-0.1, y=0, z=0, frame_id='body', speed=0.5, auto_arm=True)

def centralize_in_target(center: float, landmark0, w):
    pos = landmark0.x * w
    if center - 100 <= pos <= center + 100:
        drone.set_yaw(yaw=radians(0), frame_id='body')
    elif pos > center + 100:
        drone.set_yaw(yaw=radians(5), frame_id='body')
    elif pos < center - 100:
        drone.set_yaw(yaw=radians(-5), frame_id='body')

