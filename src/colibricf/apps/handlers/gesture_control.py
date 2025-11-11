from ...drone import Drone

drone = Drone()

def handle_move(count):
    if count == 1:
        drone.navigate_wait(x=0, y=0, z=0.1, frame_id='body', auto_arm=True)
    elif count == 2:
        drone.navigate_wait(x=0, y=0, z=-0.1, frame_id='body')
    elif count == 3:
        drone.navigate_wait(x=0.1, y=0, z=0, frame_id='body')
    elif count == 4:
        drone.navigate_wait(x=-0.1, y=0, z=0, frame_id='body')
    else:
        drone.navigate_wait(x=0, y=0, z=0, frame_id='body')
    return

def handle_fingercount(points):
    # RIGHT HAND PALM
    THUMB = 4
    fingers = [8, 12, 16, 20]
    count = 0

    if points[THUMB][0] < points[THUMB - 2][0]:
        count += 1

    for x in fingers:
        if points[x][1] < points[x - 2][1]:
            count += 1

    return count
