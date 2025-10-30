# Information: https://clover.coex.tech/programming
# Use this example to create new missions

import rospy
from colibricf.drone import Drone
drone = Drone()

TAKEOFF_ALTITUDE = 1.4

try:
    print('Arming')
    drone.arm()
    rospy.sleep(2)

    print('Take off and hover 1.4 m above the ground')
    drone.navigate_wait(x=0, y=0, z=TAKEOFF_ALTITUDE, frame_id='body', auto_arm=True)

except KeyboardInterrupt:
    print("Aborting")

except Exception as e:
    print(f"Error: {e}")

finally:
    print('Landing')
    drone.land_wait()
