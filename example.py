# Information: https://clover.coex.tech/programming
# Use this example to create new missions

from colibricf import servo
import rospy
from colibricf.task import Task
from colibricf.drone import Waypoint

class Mission(Task):
    '''
    Example of implementation. 
    '''

    TAKEOFF_ALTITUDE = 1.4

    def mission(self):
        # Implement your mission here
        
        print('note: arming')
        self.drone.arm()
        rospy.sleep(2)

        print('Take off and hover 1.4 m above the ground')
        self.drone.navigate_wait(x=0, y=0, z=self.TAKEOFF_ALTITUDE, frame_id='body', auto_arm=True)

Mission().run()

class WaypointMission(Task):
    '''
    Example of implementation. 
    '''

    def __init__(self, gpio:(int | None) = None):
        super().__init__(gpio=gpio)
        self.waypoints = [
            Waypoint(0, 2, 0),
            Waypoint(2, 0, 0),
            Waypoint(0, -2, 0),
            Waypoint(-2, 0, 0),
        ]

    TAKEOFF_ALTITUDE = 1.4

    def mission(self):
        
        print('note: arming')
        self.drone.arm()
        rospy.sleep(2)

        print('Take off and hover 1.4 m above the ground')
        self.drone.navigate_wait(x=0, y=0, z=self.TAKEOFF_ALTITUDE, frame_id='body', auto_arm=True)

        self.drone.waypoint_navigate(self.waypoints)

WaypointMission().run()
