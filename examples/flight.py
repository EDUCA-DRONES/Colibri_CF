# Information: https://clover.coex.tech/programming
# Use this example to create new missions

import rospy
from colibricf.task import Task

class Mission(Task):
    '''
    Example of implementation.
    '''

    TAKEOFF_ALTITUDE = 1.4

    def mission(self):
        # Implement your mission here
        self.drone.arm()
        rospy.sleep(2)

        self.drone.navigate_wait(x=0, y=0, z=self.TAKEOFF_ALTITUDE, frame_id='body', auto_arm=True)

Mission().run()

