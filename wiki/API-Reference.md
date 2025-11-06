# API Reference

Complete API reference for all Colibri Code Functions modules.

## Table of Contents

- [Drone API](#drone-api)
- [Camera API](#camera-api)
- [Servo API](#servo-api)
- [Task API](#task-api)
- [Computer Vision API](#computer-vision-api)

## Drone API

### Class: `Drone`

Main class for drone flight control.

#### Constructor

```python
Drone(node_name="flight")
```

**Parameters:**
- `node_name` (str, optional): ROS node name. Default: `"flight"`

#### Navigation Methods

##### `navigate_wait()`
```python
navigate_wait(x=0.0, y=0.0, z=0.0, yaw=float('nan'), speed=0.5, frame_id='body', auto_arm=True) -> None
```

Navigate to position and wait for arrival.

**Parameters:**
- `x` (float): X coordinate in meters
- `y` (float): Y coordinate in meters
- `z` (float): Z coordinate (altitude) in meters
- `yaw` (float): Yaw angle in radians
- `speed` (float): Speed in m/s
- `frame_id` (str): Frame reference ('body', 'map', 'navigate_target')
- `auto_arm` (bool): Auto-arm if disarmed

##### `navigate_global_wait()`
```python
navigate_global_wait(lat, lon, z=0.0, yaw=float('nan'), speed=0.5, frame_id='body', auto_arm=True) -> None
```

Navigate using GPS coordinates.

**Parameters:**
- `lat` (float): Latitude in decimal degrees
- `lon` (float): Longitude in decimal degrees
- `z` (float): Altitude in meters
- `yaw` (float): Yaw angle in radians
- `speed` (float): Speed in m/s
- `frame_id` (str): Frame reference
- `auto_arm` (bool): Auto-arm if disarmed

##### `wait_arrival()`
```python
wait_arrival(tolerance=0.1) -> None
```

Wait until drone reaches target.

**Parameters:**
- `tolerance` (float): Distance tolerance in meters

##### `waypoint_navigate()`
```python
waypoint_navigate(waypoints: List[Waypoint]) -> None
```

Navigate through waypoint list.

**Parameters:**
- `waypoints` (List[Waypoint]): List of Waypoint objects

##### `global_waypoint_navigate()`
```python
global_waypoint_navigate(waypoints: List[GlobalWaypoint]) -> None
```

Navigate through GPS waypoints.

**Parameters:**
- `waypoints` (List[GlobalWaypoint]): List of GlobalWaypoint objects

#### Control Methods

##### `arm()`
```python
arm(arm=True) -> None
```

Arm or disarm motors.

**Parameters:**
- `arm` (bool): True to arm, False to disarm

##### `land_wait()`
```python
land_wait() -> None
```

Land and wait until complete.

##### `set_position()`
```python
set_position(x, y, z, yaw=float('nan'), frame_id='body') -> None
```

Set target position.

##### `set_velocity()`
```python
set_velocity(vx, vy, vz, frame_id='body') -> None
```

Set target velocity.

##### `set_attitude()`
```python
set_attitude(roll, pitch, yaw, thrust) -> None
```

Set attitude and thrust.

##### `set_yaw()`
```python
set_yaw(yaw, frame_id='body') -> None
```

Set yaw angle.

##### `set_mode()`
```python
set_mode(mode: DroneMode) -> None
```

Set flight mode.

**Parameters:**
- `mode` (DroneMode): Flight mode enum value

#### Telemetry Methods

##### `get_telemetry()`
```python
get_telemetry(frame_id='body') -> TelemetryData
```

Get current telemetry data.

**Parameters:**
- `frame_id` (str): Frame reference

**Returns:** Object with fields:
- `x`, `y`, `z` (float): Position
- `vx`, `vy`, `vz` (float): Velocity
- `roll`, `pitch`, `yaw` (float): Attitude
- `lat`, `lon`, `alt` (float): GPS
- `battery_voltage` (float): Battery voltage
- `armed` (bool): Arm status

#### Utility Methods

##### `get_distance()`
```python
get_distance(x1, y1, z1, x2, y2, z2) -> float
```

Calculate 3D Euclidean distance.

**Returns:** Distance in meters

##### `haversine_distance()`
```python
haversine_distance(lat1, lon1, lat2, lon2, radius=6371000) -> float
```

Calculate GPS distance.

**Returns:** Distance in meters

##### `orbit()`
```python
orbit(radius=0.6, speed=0.3) -> None
```

Orbit current position.

**Parameters:**
- `radius` (float): Orbit radius in meters
- `speed` (float): Angular speed in rad/s

##### `is_flipped()`
```python
is_flipped() -> bool
```

Check if drone is upside down.

**Returns:** True if flipped

##### `follow()`
```python
follow() -> None
```

Start person following mode.

#### Calibration Methods

##### `calibrate_gyro()`
```python
calibrate_gyro() -> bool
```

Calibrate gyroscope.

**Returns:** True if successful

##### `toggle_aruco()`
```python
toggle_aruco() -> None
```

Toggle ArUco marker detection.

##### `toggle_optical_flow()`
```python
toggle_optical_flow() -> None
```

Toggle optical flow.

#### Parameter Methods

##### `read_cparam()`
```python
read_cparam(param_name: str) -> float
```

Read flight controller parameter.

**Returns:** Parameter value or NaN

##### `write_cparam()`
```python
write_cparam(param_name: str, value: float) -> bool
```

Write flight controller parameter.

**Returns:** True if successful

### Class: `Waypoint`

Local coordinate waypoint.

```python
Waypoint(x: float, y: float, z: float)
```

**Attributes:**
- `x` (float): X coordinate
- `y` (float): Y coordinate
- `z` (float): Z coordinate (altitude)

### Class: `GlobalWaypoint`

GPS coordinate waypoint.

```python
GlobalWaypoint(lat: float, lon: float, alt: float)
```

**Attributes:**
- `lat` (float): Latitude
- `lon` (float): Longitude
- `alt` (float): Altitude

### Enum: `DroneMode`

Flight mode enumeration.

```python
class DroneMode(enum.Enum):
    STABILIZED = "STABILIZED"
    GUIDED = "GUIDED"
    ACRO = "ACRO"
    RATTITUDE = "RATTITUDE"
    OFFBOARD = "OFFBOARD"
    MISSION = "AUTO.MISSION"
    RTL = "RTL"
```

## Camera API

### Class: `Camera`

Camera control and image capture.

#### Constructor

```python
Camera()
```

#### Methods

##### `retrieve_cv_frame()`
```python
retrieve_cv_frame() -> numpy.ndarray
```

Capture single frame.

**Returns:** OpenCV image (BGR format)

##### `save_image()`
```python
save_image(path: str) -> None
```

Save image with GPS EXIF data.

**Parameters:**
- `path` (str): Directory path

##### `read_qrcode()`
```python
read_qrcode() -> None
```

Start QR code detection (blocking).

##### `draw_face()`
```python
draw_face() -> None
```

Start face detection visualization (blocking).

## Servo API

### Class: `Servo`

Servo motor control.

#### Constructor

```python
Servo(gpio: int)
```

**Parameters:**
- `gpio` (int): GPIO pin number

#### Methods

##### `pwm_neutral()`
```python
pwm_neutral(sleep=0.5) -> None
```

Move to neutral position (1500µs).

**Parameters:**
- `sleep` (float): Hold time in seconds

##### `pwm_high()`
```python
pwm_high(sleep=0.5) -> None
```

Move to high position (2000µs).

**Parameters:**
- `sleep` (float): Hold time in seconds

##### `pwm_low()`
```python
pwm_low(sleep=0.5) -> None
```

Move to low position (500µs).

**Parameters:**
- `sleep` (float): Hold time in seconds

##### `set_pulsewidth()`
```python
set_pulsewidth(pulsewidth: int, sleep=0.5) -> None
```

Set custom pulse width.

**Parameters:**
- `pulsewidth` (int): Pulse width in microseconds
- `sleep` (float): Hold time in seconds

## Task API

### Class: `Task`

Abstract base class for missions.

#### Constructor

```python
Task(gpio: Union[int, None] = None)
```

**Parameters:**
- `gpio` (int, optional): Servo GPIO pin

#### Attributes

- `drone` (Drone): Shared drone instance
- `camera` (Camera): Shared camera instance
- `servo` (Servo, optional): Servo instance if GPIO provided

#### Abstract Methods

##### `mission()`
```python
@abstractmethod
def mission(self) -> None
```

Must be implemented by subclasses.

#### Methods

##### `run()`
```python
run() -> None
```

Execute mission with error handling.

##### `change_servo_pin()`
```python
change_servo_pin(gpio: int) -> None
```

Change servo GPIO pin.

**Parameters:**
- `gpio` (int): New GPIO pin

##### `return_to_launch_confirm()`
```python
return_to_launch_confirm() -> None
```

Prompt user to return to launch.

## Computer Vision API

### Module: `colibricf.cv.face_detect`

#### Function: `detectFace()`
```python
detectFace(frame) -> list
```

Detect faces in image.

**Parameters:**
- `frame` (numpy.ndarray): Grayscale image

**Returns:** List of (x, y, w, h) tuples

### Module: `colibricf.cv.qrcode`

QR code detection handled through `Camera.read_qrcode()`.

### Module: `colibricf.cv.follow`

Person following handled through `Drone.follow()`.

---

## Type Hints

```python
from typing import List, Union
import numpy as np

# Common types
Position = tuple[float, float, float]  # (x, y, z)
GPSCoordinate = tuple[float, float]     # (lat, lon)
Image = np.ndarray                       # OpenCV image
```

---

## See Also

- [Drone Module](Drone-Module.md) - Detailed drone documentation
- [Camera Module](Camera-Module.md) - Camera usage guide
- [Servo Module](Servo-Module.md) - Servo control guide
- [Task Framework](Task-Framework.md) - Task creation guide

[← Back to Home](Home.md)
