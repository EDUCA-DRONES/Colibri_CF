import rospy
from colibricf.task import Task

class Follow(Task):
    '''
    Follows a person in from of the drone.
    '''

    TAKEOFF_ALTITUDE = 2

    def mission(self):
        self.drone.arm()
        rospy.sleep(3)

        self.drone.navigate_wait(z=self.TAKEOFF_ALTITUDE, auto_arm=True)

        self.drone.gesture_control()

Follow().run()
