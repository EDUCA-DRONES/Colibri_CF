import rospy
import enum
import math
import numpy as np
import dynamic_reconfigure.client
from typing import List
from clover import srv
from pymavlink import mavutil
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from mavros import mavlink
from mavros_msgs.srv import CommandBool, SetMode, ParamGet, ParamSet, CommandLong
from mavros_msgs.msg import State, ParamValue, Mavlink

class DroneMode(enum.Enum):
    '''
    Drone flight modes.
    '''

    STABILIZED = "STABILIZED"
    GUIDED = "GUIDED"
    ACRO = "ACRO"
    RATTITUDE = "RATTITUDE"
    OFFBOARD = "OFFBOARD"
    MISSION = "AUTO.MISSION"
    RTL = "RTL"

class Waypoint():
    x:float = 0
    y:float = 0
    z:float = 0

    def __init__(self, x:float, y:float, z:float):
        self.x = x
        self.y = y
        self.z = z

class GlobalWaypoint():
    lat:float
    lon: float
    alt: float

    def __init__(self, lat:float, lon:float, alt:float):
        self.lat = lat
        self.lon = lon
        self.alt = alt


class Drone:
    def __init__(self, node_name="flight"):
        rospy.init_node(node_name)

        self.tolerance = 0.1

        self.get_telemetry = rospy.ServiceProxy(
            'get_telemetry', srv.GetTelemetry)
        self._navigate = rospy.ServiceProxy('navigate', srv.Navigate)
        self.navigate_global = rospy.ServiceProxy(
            'navigate_global', srv.NavigateGlobal)
        self.set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
        self.set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
        self.set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
        self.set_yaw = rospy.ServiceProxy('set_yaw', srv.SetYaw)
        self.set_yaw_rates = rospy.ServiceProxy('set_yaw_rate', srv.SetYawRate)
        self.set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
        self._land = rospy.ServiceProxy('land', Trigger)
        self._set_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.send_command = rospy.ServiceProxy(
            'mavros/cmd/command', CommandLong)
        self.release = rospy.ServiceProxy('simple_offboard/release', Trigger)

    def navigate_wait(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, yaw: float = float('nan'), speed: float = 0.5, frame_id: str = 'body', auto_arm: bool = True) -> None:
        '''
        Navigate to a position with wait for completion.
        '''

        telemetry = self.get_telemetry(frame_id='body')

        self._navigate(x=x, y=y, z=z, yaw=yaw, speed=speed,
                       frame_id=frame_id, auto_arm=auto_arm)

        while not rospy.is_shutdown():
            telemetry = self.get_telemetry(frame_id='navigate_target')
            if math.sqrt((telemetry.x)**2 + (telemetry.y)**2 + (telemetry.z)**2) < self.tolerance:
                break
            rospy.sleep(0.1)

    def navigate_global_wait(self, lat: float, lon: float, z: float = 0.0, yaw: float = float('nan'), speed: float = 0.5, frame_id: str = 'body', auto_arm: bool = True) -> None:
        '''
        Navigate to a global position with wait for completion.
        '''

        telemetry = self.get_telemetry(frame_id='body')

        self.navigate_global(lat=lat, lon=lon, z=z, yaw=yaw,
                             frame_id=frame_id, speed=speed, auto_arm=auto_arm)

        while not rospy.is_shutdown():
            telemetry = self.get_telemetry(frame_id='navigate_target')
            if math.sqrt((telemetry.x)**2 + (telemetry.y)**2 + (telemetry.z)**2) < self.tolerance:
                break
            rospy.sleep(0.1)

    def land_wait(self) -> None:
        '''
        Land the drone and wait until it is landed.
        '''

        self._land()
        while self.get_telemetry().armed:
            rospy.sleep(0.2)

    def wait_arrival(self, tolerance: float = 0.1) -> None:
        '''
        Wait until the drone arrives at the target position.
        '''

        while not rospy.is_shutdown():
            telemetry = self.get_telemetry(frame_id='navigate_target')
            if math.sqrt(telemetry.x ** 2 + telemetry.y ** 2 + telemetry.z ** 2) < tolerance:
                break
            rospy.sleep(0.2)

    def get_distance(self, x1: float, y1: float, z1: float, x2: float, y2: float, z2: float) -> float:
        '''
        Calculate the Euclidean distance between two points in 3D space.
        '''

        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)

    def arm(self, arm: bool = True) -> None:
        '''
        Arm the drone.
        '''

        try:
            arm_service = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
            arm_service(arm)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def is_flipped(self) -> bool:
        '''
        Verifies if drone is flipped.
        '''

        PI_2 = math.pi / 2
        telem = self.get_telemetry()
        flipped = abs(telem.roll) > PI_2 or abs(telem.pitch) > PI_2
        return flipped

    def orbit(self, radius: float = 0.6, speed: float = 0.3) -> None:
        '''
        Make the drone orbit around a point.
        '''

        RADIUS = radius  # m
        SPEED = speed  # rad / s

        start = self.get_telemetry()
        start_stamp = rospy.get_rostime()

        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            angle = (rospy.get_rostime() - start_stamp).to_sec() * SPEED
            x = start.x + math.sin(angle) * RADIUS
            y = start.y + math.cos(angle) * RADIUS
            self.set_position(x=x, y=y, z=start.z)

            r.sleep()

    def send_msg(self, msg: str) -> None:
        '''
        Send a message to the drone.
        '''

        mavlink_pub = rospy.Publisher('mavlink/to', Mavlink, queue_size=1)

        # Sending a HEARTBEAT message:
        msg = mavutil.mavlink.MAVLink_heartbeat_message(
            mavutil.mavlink.MAV_TYPE_GCS, 0, 0, 0, 0, 0)
        msg.pack(mavutil.mavlink.MAVLink('', 2, 1))
        ros_msg = mavlink.convert_to_rosmsg(msg)

        mavlink_pub.publish(ros_msg)

    def set_mode(self, mode: DroneMode) -> None:
        '''
        Set the flight mode of the drone.
        '''

        try:
            self._set_mode(custom_mode=mode)
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to set mode {mode}: {e}")

    def calibrate_gyro(self):
        '''
        Calibrate the drone's gyro.
        '''

        rospy.loginfo('Calibrate gyro')
        if not self.send_command(command=mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, param1=1).success:
            return False

        calibrating = False
        while not rospy.is_shutdown():
            state = rospy.wait_for_message('mavros/state', State)
            if state.system_status == mavutil.mavlink.MAV_STATE_CALIBRATING or state.system_status == mavutil.mavlink.MAV_STATE_UNINIT:
                calibrating = True
            elif calibrating and state.system_status == mavutil.mavlink.MAV_STATE_STANDBY:
                rospy.loginfo('Calibrating finished')
                return True

    def toggle_aruco(self) -> None:
        '''
        Toggle the ArUco marker detection.
        '''

        try:
            aruco_client = dynamic_reconfigure.client.Client('aruco_detection')
            config = aruco_client.get_configuration()
            config['enable'] = not config['enable']
            aruco_client.update_configuration(config)
            rospy.loginfo(
                f"Aruco detection {'enabled' if config['enable'] else 'disabled'}")
        except Exception as e:
            rospy.logerr(f"Failed to toggle ArUco detection: {e}")

    def toggle_optical_flow(self) -> None:
        '''
        Toggle the optical flow detection.
        '''

        try:
            optical_flow_client = dynamic_reconfigure.client.Client(
                'optical_flow')
            config = optical_flow_client.get_configuration()
            config['enable'] = not config['enable']
            optical_flow_client.update_configuration(config)
            rospy.loginfo(
                f"Optical flow {'enabled' if config['enable'] else 'disabled'}")
        except Exception as e:
            rospy.logerr(f"Failed to toggle optical flow: {e}")

    def read_cparam(self, param_name: str) -> float:
        '''
        Read a parameter from flight controller.
        '''

        try:
            param_get = rospy.ServiceProxy('mavros/param/get', ParamGet)
            response = param_get(param_name)
            if response.success:
                return response.value.real
            else:
                rospy.logerr(f"Failed to read parameter {param_name}")
                return float("nan")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return float("nan")

    def write_cparam(self, param_name: str, value: float) -> bool:
        '''
        Write a parameter to flight controller.
        '''

        try:
            param_set = rospy.ServiceProxy('mavros/param/set', ParamSet)
            param_value = ParamValue(real=value)
            response = param_set(param_name, param_value)
            if response.success:
                return True
            else:
                rospy.logerr(f"Failed to write parameter {param_name}")
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    def haversine_distance(self, lat1, lon1, lat2, lon2, radius=6371000):
        '''
        Calculate the distance between two points on Earth specified by latitude/longitude.
        '''

        # Convert degrees to radians
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)

        # Haversine formula
        a = math.sin(dphi / 2.0) ** 2 + math.cos(phi1) * \
            math.cos(phi2) * math.sin(dlambda / 2.0) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        return radius * c

    def waypoint_navigate(self, waypoints: List[Waypoint]):
        '''
        Flight through the defined waypoints.
        '''

        for wp in waypoints:
            print(f"note: navigating to waypoint: ({wp.x}, {wp.y}, {wp.z})")
            self.navigate_wait(x=wp.x, y=wp.y, z=wp.z, speed=0.5)
            rospy.sleep(0.5)  # Pause at waypoint

    def global_waypoint_navigate(self, waypoints: List[GlobalWaypoint]):
        '''
        Flight through the defined global waypoints.
        '''

        for i, wp in enumerate(waypoints):
            print(f"Moving to waypoint {i+1}: ({wp.lat}, {wp.lon}, {wp.alt})")
            self.navigate_global_wait(lat=wp.lat, lon=wp.lon, z=wp.alt, speed=0.5)
            rospy.sleep(0.5)  # Pause at waypoint

    def follow(self):
        from .apps.follow import _follow_callback
        rospy.Subscriber('main_camera/image_raw_throttled', Image, _follow_callback, queue_size=1)
        rospy.spin()

    def gesture_control(self):
        from .apps.gesture_control import _gc_callback
        rospy.Subscriber('main_camera/image_raw_throttled', Image, _gc_callback, queue_size=1)
        rospy.spin()



