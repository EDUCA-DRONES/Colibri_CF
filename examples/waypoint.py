# Information: https://clover.coex.tech/programming
# Use this example to create new missions

import rospy
from colibricf.task import Task
from colibricf.drone import Waypoint

class WaypointMission(Task):
    '''
    Example of implementation.
    '''

    TAKEOFF_ALTITUDE = 1.4

    def mission(self):
        waypoints = [
            Waypoint(0, 2, 0),
            Waypoint(2, 0, 0),
            Waypoint(0, -2, 0),
            Waypoint(-2, 0, 0),
        ]

        self.drone.arm()
        rospy.sleep(2)

        self.drone.navigate_wait(x=0, y=0, z=self.TAKEOFF_ALTITUDE, frame_id='body', auto_arm=True)

        self.drone.waypoint_navigate(waypoints)

WaypointMission().run()
