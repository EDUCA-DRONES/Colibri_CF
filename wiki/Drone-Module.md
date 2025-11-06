# Drone Module

The **Drone** module is the core component of Colibri Code Functions, providing comprehensive flight control, navigation, and telemetry capabilities for autonomous drone operations.

## Table of Contents

- [Overview](#overview)
- [Initialization](#initialization)
- [Flight Modes](#flight-modes)
- [Navigation Methods](#navigation-methods)
- [Landing Operations](#landing-operations)
- [Arming & Disarming](#arming--disarming)
- [Position & Velocity Control](#position--velocity-control)
- [Telemetry](#telemetry)
- [Waypoint Navigation](#waypoint-navigation)
- [Special Operations](#special-operations)
- [Calibration](#calibration)
- [Parameter Management](#parameter-management)
- [Person Following](#person-following)
- [Complete Examples](#complete-examples)

## Overview

The `Drone` class abstracts complex ROS services into intuitive Python methods, making autonomous flight programming simple and accessible. It handles all communication with the Clover framework and PX4/ArduPilot flight controller.

**Key Features:**
- Autonomous navigation in local and global coordinates
- Precise position and velocity control
- Real-time telemetry data
- Waypoint mission planning
- Person detection and following
- Flight controller parameter management
- Sensor calibration utilities

## Initialization

### Basic Initialization

```python
from colibricf.drone import Drone

# Initialize with default node name
drone = Drone()
```

### Custom Node Name

```python
# Initialize with custom ROS node name
drone = Drone(node_name="my_custom_flight_node")
```

### Parameters

- **`node_name`** (str, optional): ROS node name. Default: `"flight"`

**Note:** Each Python script should only initialize one Drone instance, as it creates a ROS node.

## Flight Modes

The `DroneMode` enumeration defines available flight control modes:

```python
from colibricf.drone import DroneMode

class DroneMode(enum.Enum):
    STABILIZED = "STABILIZED"      # Manual stabilized flight
    GUIDED = "GUIDED"              # GPS-guided autonomous flight  
    ACRO = "ACRO"                  # Acrobatic mode (no stabilization)
    RATTITUDE = "RATTITUDE"        # Rate + Attitude mode
    OFFBOARD = "OFFBOARD"          # Computer-controlled flight
    MISSION = "AUTO.MISSION"       # Pre-programmed mission mode
    RTL = "RTL"                    # Return to launch
```

### Setting Flight Mode

```python
# Set to guided mode for GPS navigation
drone.set_mode(DroneMode.GUIDED)

# Set to offboard mode for computer control
drone.set_mode(DroneMode.OFFBOARD)

# Return to launch
drone.set_mode(DroneMode.RTL)
```

## Navigation Methods

### navigate_wait()

Navigate to a position and wait until arrival.

```python
drone.navigate_wait(
    x=1.0,              # X coordinate (meters)
    y=0.5,              # Y coordinate (meters)
    z=1.5,              # Z coordinate (altitude, meters)
    yaw=0.0,            # Yaw angle (radians)
    speed=0.5,          # Movement speed (m/s)
    frame_id='body',    # Reference frame
    auto_arm=True       # Auto-arm if disarmed
)
```

**Parameters:**

- **`x`** (float): X position in meters
- **`y`** (float): Y position in meters  
- **`z`** (float): Z position (altitude) in meters
- **`yaw`** (float): Yaw angle in radians. Use `float('nan')` to maintain current heading
- **`speed`** (float): Maximum speed in m/s
- **`frame_id`** (str): Reference frame (`'body'`, `'map'`, or `'navigate_target'`)
- **`auto_arm`** (bool): Automatically arm motors if needed

**Frame Reference:**
- `'body'`: Relative to current drone position/orientation
- `'map'`: Relative to takeoff point (global frame)
- `'navigate_target'`: Relative to last navigation target

**Example:**

```python
# Move forward 2 meters (relative to drone)
drone.navigate_wait(x=2, y=0, z=1.0, frame_id='body')

# Move to absolute position (relative to takeoff)
drone.navigate_wait(x=5, y=3, z=2.0, frame_id='map')
```

### navigate_global_wait()

Navigate using GPS coordinates.

```python
drone.navigate_global_wait(
    lat=47.397743,      # Latitude
    lon=8.545594,       # Longitude
    z=10.0,             # Altitude above ground
    yaw=float('nan'),   # Yaw angle
    speed=2.0,          # Speed
    auto_arm=True       # Auto-arm
)
```

**Parameters:**

- **`lat`** (float): Latitude in decimal degrees
- **`lon`** (float): Longitude in decimal degrees
- **`z`** (float): Altitude in meters above ground level
- **`yaw`** (float): Yaw angle in radians
- **`speed`** (float): Maximum speed in m/s
- **`auto_arm`** (bool): Auto-arm before navigation

**Example:**

```python
# Fly to specific GPS coordinates
drone.navigate_global_wait(
    lat=47.3977,
    lon=8.5455,
    z=15.0,
    speed=3.0
)
```

### wait_arrival()

Wait until the drone reaches the target position.

```python
# Wait with default tolerance (0.1m)
drone.wait_arrival()

# Wait with custom tolerance
drone.wait_arrival(tolerance=0.2)
```

**Parameters:**

- **`tolerance`** (float): Distance tolerance in meters (default: 0.1)

## Landing Operations

### land_wait()

Land the drone and wait until landing is complete.

```python
drone.land_wait()
```

This method:
1. Initiates landing sequence
2. Waits until motors are disarmed
3. Automatically disarms after landing

**Example:**

```python
# Complete flight
drone.arm()
rospy.sleep(2)
drone.navigate_wait(z=1.5, auto_arm=True)
rospy.sleep(5)
drone.land_wait()  # Safe landing
```

## Arming & Disarming

### arm()

Arm or disarm the drone motors.

```python
# Arm the motors
drone.arm(arm=True)

# Disarm the motors
drone.arm(arm=False)
```

**Parameters:**

- **`arm`** (bool): `True` to arm, `False` to disarm

**Safety Notes:**
- Always arm in a safe area
- Propellers will spin immediately after arming
- Disarm only when landed

## Position & Velocity Control

### set_position()

Set target position (requires OFFBOARD mode).

```python
drone.set_position(
    x=1.0,
    y=0.5,
    z=2.0,
    yaw=0.0,
    frame_id='body'
)
```

### set_velocity()

Set target velocity.

```python
drone.set_velocity(
    vx=0.5,    # Velocity in X (m/s)
    vy=0.2,    # Velocity in Y (m/s)
    vz=0.1,    # Velocity in Z (m/s)
    frame_id='body'
)
```

### set_attitude()

Set attitude (orientation) and thrust.

```python
drone.set_attitude(
    roll=0.0,     # Roll angle (radians)
    pitch=0.0,    # Pitch angle (radians)
    yaw=0.0,      # Yaw angle (radians)
    thrust=0.5    # Thrust (0.0 to 1.0)
)
```

### set_yaw()

Set only the yaw angle.

```python
# Set yaw to 90 degrees (π/2 radians)
import math
drone.set_yaw(yaw=math.pi/2, frame_id='body')
```

## Telemetry

### get_telemetry()

Retrieve current drone state and position data.

```python
# Get telemetry in body frame
telemetry = drone.get_telemetry(frame_id='body')

# Access telemetry data
print(f"Position: x={telemetry.x:.2f}, y={telemetry.y:.2f}, z={telemetry.z:.2f}")
print(f"Velocity: vx={telemetry.vx:.2f}, vy={telemetry.vy:.2f}, vz={telemetry.vz:.2f}")
print(f"Attitude: roll={telemetry.roll:.2f}, pitch={telemetry.pitch:.2f}, yaw={telemetry.yaw:.2f}")
print(f"Battery: {telemetry.battery_voltage:.1f}V")
print(f"Armed: {telemetry.armed}")
```

**Available Telemetry Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `x`, `y`, `z` | float | Position coordinates (m) |
| `vx`, `vy`, `vz` | float | Velocity components (m/s) |
| `roll`, `pitch`, `yaw` | float | Attitude angles (radians) |
| `lat`, `lon`, `alt` | float | GPS coordinates |
| `battery_voltage` | float | Battery voltage (V) |
| `armed` | bool | Motor arm status |

## Waypoint Navigation

### Waypoint Class

Define waypoints for autonomous missions:

```python
from colibricf.drone import Waypoint

# Create waypoints
wp1 = Waypoint(x=0, y=2, z=1.5)
wp2 = Waypoint(x=2, y=2, z=1.5)
wp3 = Waypoint(x=2, y=0, z=1.5)
wp4 = Waypoint(x=0, y=0, z=1.5)
```

### waypoint_navigate()

Navigate through a list of waypoints.

```python
from colibricf.drone import Drone, Waypoint
import rospy

drone = Drone()

# Define waypoints (square pattern)
waypoints = [
    Waypoint(0, 2, 1.5),
    Waypoint(2, 2, 1.5),
    Waypoint(2, 0, 1.5),
    Waypoint(0, 0, 1.5),
]

# Arm and takeoff
drone.arm()
rospy.sleep(3)
drone.navigate_wait(z=1.5, auto_arm=True)

# Execute waypoint mission
drone.waypoint_navigate(waypoints)

# Land
drone.land_wait()
```

### GlobalWaypoint Class

Define GPS-based waypoints:

```python
from colibricf.drone import GlobalWaypoint

# Create global waypoints
gps_wp1 = GlobalWaypoint(lat=47.3977, lon=8.5455, alt=10.0)
gps_wp2 = GlobalWaypoint(lat=47.3980, lon=8.5460, alt=10.0)
```

### global_waypoint_navigate()

Navigate through GPS waypoints.

```python
from colibricf.drone import Drone, GlobalWaypoint

drone = Drone()

# Define GPS waypoints
waypoints = [
    GlobalWaypoint(lat=47.3977, lon=8.5455, alt=15.0),
    GlobalWaypoint(lat=47.3980, lon=8.5460, alt=15.0),
    GlobalWaypoint(lat=47.3975, lon=8.5465, alt=15.0),
]

# Execute mission
drone.arm()
rospy.sleep(3)
drone.global_waypoint_navigate(waypoints)
drone.land_wait()
```

## Special Operations

### orbit()

Make the drone orbit around its current position.

```python
# Orbit with 1.0m radius at 0.5 rad/s
drone.orbit(radius=1.0, speed=0.5)
```

**Parameters:**

- **`radius`** (float): Orbital radius in meters
- **`speed`** (float): Angular velocity in radians/second

**Note:** This is a blocking operation. Use Ctrl+C or implement a timeout to stop.

### follow()

Enable person detection and following mode.

```python
# Start person following
drone.follow()
```

**Requirements:**
- Camera must be enabled
- MediaPipe library installed
- Clear view of person

**How it works:**
1. Detects person using pose estimation
2. Centers person in camera view
3. Maintains distance by moving forward/backward
4. Rotates to keep person centered

### is_flipped()

Check if the drone is upside down.

```python
if drone.is_flipped():
    print("Warning: Drone is flipped!")
    drone.land_wait()
```

**Returns:** `bool` - True if drone roll or pitch exceeds 90°

## Distance Calculations

### get_distance()

Calculate Euclidean distance between two 3D points.

```python
# Calculate distance between two points
distance = drone.get_distance(
    x1=0, y1=0, z1=0,
    x2=3, y2=4, z2=0
)
print(f"Distance: {distance:.2f}m")  # Output: 5.00m
```

### haversine_distance()

Calculate distance between GPS coordinates.

```python
# Calculate distance between two GPS points
distance = drone.haversine_distance(
    lat1=47.3977,
    lon1=8.5455,
    lat2=47.3980,
    lon2=8.5460
)
print(f"GPS Distance: {distance:.2f}m")
```

**Parameters:**

- **`lat1`, `lon1`**: First GPS coordinate
- **`lat2`, `lon2`**: Second GPS coordinate  
- **`radius`** (optional): Earth radius in meters (default: 6,371,000)

## Calibration

### calibrate_gyro()

Calibrate the gyroscope.

```python
print("Starting gyro calibration...")
if drone.calibrate_gyro():
    print("Gyro calibrated successfully!")
else:
    print("Calibration failed")
```

**Important:**
- Drone must be on level surface
- Keep drone completely still during calibration
- Process takes ~30 seconds

### toggle_aruco()

Enable/disable ArUco marker detection.

```python
# Toggle ArUco detection on/off
drone.toggle_aruco()
```

### toggle_optical_flow()

Enable/disable optical flow sensor.

```python
# Toggle optical flow on/off
drone.toggle_optical_flow()
```

## Parameter Management

### read_cparam()

Read a flight controller parameter.

```python
# Read battery voltage multiplier
batt_mult = drone.read_cparam("BATT_V_MULT")
print(f"Battery multiplier: {batt_mult}")

# Read GPS type parameter
gps_type = drone.read_cparam("GPS_TYPE")
print(f"GPS Type: {gps_type}")
```

**Returns:** Parameter value as `float`, or `NaN` if failed

### write_cparam()

Write a flight controller parameter.

```python
# Set battery voltage multiplier
success = drone.write_cparam("BATT_V_MULT", 11.0)

if success:
    print("Parameter written successfully")
else:
    print("Failed to write parameter")
```

**Parameters:**

- **`param_name`** (str): Parameter name
- **`value`** (float): New parameter value

**Returns:** `bool` - Success status

**⚠️ Warning:** Incorrect parameters can cause flight issues. Only modify if you know what you're doing.

## Person Following

### centralize_in_target()

Internal method for centering target in camera view.

```python
# Used internally by follow() method
drone.centralize_in_target(center, landmark, width)
```

### follow_handle_move()

Internal method for maintaining distance during following.

```python
# Used internally by follow() method
drone.follow_handle_move(target_size)
```

**Note:** These methods are called automatically by `follow()`.

## Complete Examples

### Basic Flight Mission

```python
from colibricf.drone import Drone
import rospy

drone = Drone()

# Pre-flight check
telem = drone.get_telemetry()
print(f"Battery: {telem.battery_voltage:.1f}V")

if telem.battery_voltage < 10.5:
    print("Battery too low!")
    exit()

# Flight
drone.arm()
rospy.sleep(3)

# Takeoff
drone.navigate_wait(z=1.5, auto_arm=True)
rospy.sleep(2)

# Fly forward
drone.navigate_wait(x=3, y=0, z=1.5)
rospy.sleep(2)

# Return to start
drone.navigate_wait(x=0, y=0, z=1.5)
rospy.sleep(2)

# Land
drone.land_wait()
```

### Square Pattern Mission

```python
from colibricf.drone import Drone, Waypoint
import rospy

drone = Drone()

# Square waypoints (3m sides)
waypoints = [
    Waypoint(0, 0, 1.5),
    Waypoint(3, 0, 1.5),
    Waypoint(3, 3, 1.5),
    Waypoint(0, 3, 1.5),
    Waypoint(0, 0, 1.5),
]

# Execute
drone.arm()
rospy.sleep(3)
drone.navigate_wait(z=1.5, auto_arm=True)

drone.waypoint_navigate(waypoints)

drone.land_wait()
```

### Orbit Mission

```python
from colibricf.drone import Drone
import rospy
import threading

drone = Drone()

# Orbit for 20 seconds
def orbit_with_timeout():
    drone.orbit(radius=1.5, speed=0.5)

# Arm and position
drone.arm()
rospy.sleep(3)
drone.navigate_wait(z=2.0, auto_arm=True)

# Start orbit in thread
orbit_thread = threading.Thread(target=orbit_with_timeout)
orbit_thread.start()

# Stop after 20 seconds
rospy.sleep(20)
orbit_thread.join(timeout=1)

# Land
drone.land_wait()
```

### GPS Navigation Mission

```python
from colibricf.drone import Drone, GlobalWaypoint
import rospy

drone = Drone()

# GPS mission waypoints
waypoints = [
    GlobalWaypoint(lat=47.397743, lon=8.545594, alt=20.0),
    GlobalWaypoint(lat=47.397800, lon=8.545700, alt=20.0),
    GlobalWaypoint(lat=47.397743, lon=8.545594, alt=20.0),
]

# Execute GPS mission
drone.arm()
rospy.sleep(3)

drone.global_waypoint_navigate(waypoints)

drone.land_wait()
```

---

## See Also

- [Task Framework](Task-Framework.md) - Mission planning with error handling
- [Camera Module](Camera-Module.md) - Computer vision integration
- [Examples and Tutorials](Examples-and-Tutorials.md) - More practical examples
- [API Reference](API-Reference.md) - Complete API documentation

[← Back to Home](Home.md) | [Next: Camera Module →](Camera-Module.md)
