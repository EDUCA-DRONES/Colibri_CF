# Drone Control Module

The `Drone` class provides comprehensive flight control capabilities for Colibri drones using the Clover framework. It abstracts complex ROS services into intuitive Python methods.

## Overview

The Drone module is the core component for autonomous flight operations. It handles navigation, positioning, telemetry, and system configuration through a series of intuitive methods built on top of Clover's ROS services.

## Initialization

```python
from colibricf.drone import Drone

# Initialize the drone
drone = Drone(node_name="flight")
```

### Parameters

- `node_name` (str, optional): The ROS node name. Default is `"flight"`.

## Flight Modes

The `DroneMode` enum defines available flight modes:

```python
from colibricf.drone import DroneMode

class DroneMode(enum.Enum):
    STABILIZED = "STABILIZED"      # Manual stabilized flight
    GUIDED = "GUIDED"              # GPS guided flight
    ACRO = "ACRO"                  # Acrobatic mode
    RATTITUDE = "RATTITUDE"        # Mixed manual/attitude mode
    OFFBOARD = "OFFBOARD"          # Computer-controlled flight
    MISSION = "AUTO.MISSION"       # Autonomous mission mode
    RTL = "RTL"                    # Return to launch
```

## Navigation Methods

### navigate_wait()

Navigate to a position and wait for arrival.

```python
drone.navigate_wait(
    x=1.0,              # X coordinate (meters)
    y=0.5,              # Y coordinate (meters)
    z=1.0,              # Z coordinate (altitude in meters)
    yaw=0.0,            # Yaw angle (radians, NaN = no rotation)
    speed=0.5,          # Movement speed (m/s)
    frame_id='body',    # Reference frame ('body' or 'map')
    auto_arm=True       # Auto-arm before navigation
)
```

### navigate_global_wait()

Navigate using GPS coordinates and wait for arrival.

```python
drone.navigate_global_wait(
    lat=59.123456,      # Latitude
    lon=10.123456,      # Longitude
    z=1.0,              # Altitude (meters)
    yaw=float('nan'),   # Yaw angle
    speed=0.5,          # Movement speed
    auto_arm=True       # Auto-arm
)
```

### wait_arrival()

Wait until the drone reaches the target position.

```python
drone.wait_arrival(tolerance=0.1)  # tolerance in meters
```

## Landing Methods

### land_wait()

Land the drone and wait until it is completely landed.

```python
drone.land_wait()
```

This method automatically disarms the drone when landing is complete.

## Arming & Disarming

### arm()

Arm or disarm the drone.

```python
drone.arm(arm=True)   # Arm the drone
drone.arm(arm=False)  # Disarm the drone
```

## Position & Velocity Control

### set_position()

Set the target position (requires OFFBOARD mode).

```python
drone.set_position(x=1.0, y=0.5, z=1.0)
```

### set_velocity()

Set the target velocity (requires OFFBOARD mode).

```python
drone.set_velocity(vx=0.5, vy=0.2, vz=0.0)
```

### set_attitude()

Set the attitude (roll, pitch, yaw).

```python
drone.set_attitude(roll=0.0, pitch=0.0, yaw=0.0, thrust=0.5)
```

### set_yaw()

Set the yaw angle.

```python
drone.set_yaw(yaw=1.57)  # Radians
```

## Telemetry

### get_telemetry()

Retrieve current drone telemetry data.

```python
telemetry = drone.get_telemetry(frame_id='body')
print(f"Position: x={telemetry.x}, y={telemetry.y}, z={telemetry.z}")
print(f"Velocity: vx={telemetry.vx}, vy={telemetry.vy}, vz={telemetry.vz}")
print(f"Attitude: roll={telemetry.roll}, pitch={telemetry.pitch}, yaw={telemetry.yaw}")
print(f"Armed: {telemetry.armed}")
```

**Returns:** Telemetry object with position, velocity, and attitude data.

## Flight Mode Control

### set_mode()

Change the drone's flight mode.

```python
drone.set_mode(DroneMode.GUIDED)
drone.set_mode(DroneMode.RTL)
```

## Special Flight Operations

### orbit()

Make the drone orbit around its current position.

```python
drone.orbit(radius=1.0, speed=0.5)
```

### is_flipped()

Check if the drone is in an inverted position.

```python
if drone.is_flipped():
    print("Drone is flipped!")
```

**Returns:** Boolean value

## Distance Calculations

### get_distance()

Calculate Euclidean distance between two 3D points.

```python
distance = drone.get_distance(0, 0, 0, 1, 1, 1)
print(f"Distance: {distance:.2f} meters")
```

### haversine_distance()

Calculate distance between two GPS coordinates (on Earth's surface).

```python
distance = drone.haversine_distance(
    lat1=59.123456,
    lon1=10.123456,
    lat2=59.234567,
    lon2=10.234567
)

print(f"GPS Distance: {distance:.2f} meters")
```

## Calibration

### calibrate_gyro()

Calibrate the drone's gyroscope.

```python
if drone.calibrate_gyro():
    print("Gyro calibrated successfully")
else:
    print("Gyro calibration failed")
```

**Returns:** Boolean indicating success

## Advanced Features

### toggle_aruco()

Enable/disable ArUco marker detection.

```python
drone.toggle_aruco()
```

### toggle_optical_flow()

Enable/disable optical flow detection.

```python
drone.toggle_optical_flow()
```

## Flight Controller Parameters

### read_cparam()

Read a parameter from the flight controller (Pixhawk/Autopilot).

```python
battery_voltage = drone.read_cparam("BATT_V_MULT")
print(f"Battery voltage multiplier: {battery_voltage}")
```

**Returns:** Parameter value or NaN if failed

### write_cparam()

Write a parameter to the flight controller.

```python
success = drone.write_cparam(param_name="BATT_V_MULT", value=11.0)
if success:
    print("Parameter written successfully")
```

**Returns:** Boolean indicating success

## Notes

- Always ensure the drone is properly calibrated before flight
- Use appropriate safety distances when testing
- The `tolerance` parameter affects how long the drone waits after reaching the target
- Frame references are important: 'body' is relative to drone, 'map' is global
- Always include error handling in production code

## See Also

- [Clover Documentation](https://clover.coex.tech/en/programming)
- [ROS Navigation Stack](http://wiki.ros.org/navigation)
