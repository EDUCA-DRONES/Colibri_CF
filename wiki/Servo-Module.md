# Servo Module

The **Servo** module provides precise control over servo motors connected to the drone's GPIO pins, enabling manipulation of camera gimbals, payload release mechanisms, and other servo-actuated systems.

## Table of Contents

- [Overview](#overview)
- [Initialization](#initialization)
- [Basic Operations](#basic-operations)
- [Custom Pulse Width Control](#custom-pulse-width-control)
- [Hardware Setup](#hardware-setup)
- [Applications](#applications)
- [Troubleshooting](#troubleshooting)
- [Complete Examples](#complete-examples)

## Overview

The Servo module uses the pigpio library to generate PWM (Pulse Width Modulation) signals for precise servo motor control. It supports standard hobby servos (180° rotation) and continuous rotation servos.

**Key Features:**
- ✅ Precise PWM control (1 microsecond resolution)
- ✅ Standard servo positions (low, neutral, high)
- ✅ Custom pulse width control
- ✅ Multiple servo support
- ✅ Easy integration with drone operations

**Technical Specifications:**
- **PWM Frequency**: 50 Hz (standard for servos)
- **Pulse Width Range**: 500-2500 microseconds
- **Resolution**: 1 microsecond steps
- **Supported Servos**: Standard hobby servos

## Initialization

### Basic Initialization

```python
from colibricf.servo import Servo

# Initialize servo on GPIO pin 14
servo = Servo(gpio=14)
```

**Parameters:**

- **`gpio`** (int): GPIO pin number where servo signal wire is connected

### GPIO Pin Selection

Available GPIO pins on Raspberry Pi (commonly used):

| GPIO Pin | Physical Pin | Notes |
|----------|-------------|-------|
| GPIO 14 | Pin 8 | UART TX (can be used for servo) |
| GPIO 15 | Pin 10 | UART RX (can be used for servo) |
| GPIO 17 | Pin 11 | Common choice |
| GPIO 18 | Pin 12 | Hardware PWM capable |
| GPIO 22 | Pin 15 | General purpose |
| GPIO 23 | Pin 16 | General purpose |
| GPIO 24 | Pin 18 | General purpose |
| GPIO 25 | Pin 22 | General purpose |

**Recommended:** Use GPIO 17, 18, 22, 23 for servo control to avoid conflicts with other functions.

### Error Handling

```python
from colibricf.servo import Servo

try:
    servo = Servo(gpio=14)
    print("Servo initialized successfully on GPIO 14")
except Exception as e:
    print(f"Failed to initialize servo: {e}")
    print("Check:")
    print("  1. pigpio daemon is running")
    print("  2. GPIO pin is not in use")
    print("  3. Correct pin number")
```

### Multiple Servos

```python
from colibricf.servo import Servo

# Initialize multiple servos
gimbal_pitch = Servo(gpio=14)  # Camera pitch
gimbal_roll = Servo(gpio=15)   # Camera roll
payload_release = Servo(gpio=17)  # Cargo release
```

## Basic Operations

### pwm_neutral()

Move servo to neutral/center position (1500 µs).

```python
servo = Servo(gpio=14)

# Move to neutral position
servo.pwm_neutral(sleep=0.5)
```

**Parameters:**

- **`sleep`** (float): Time to hold position in seconds (default: 0.5)

**Use Cases:**
- Center camera gimbal
- Home position for mechanisms
- Safe resting position

### pwm_high()

Move servo to maximum position (2000 µs).

```python
servo = Servo(gpio=14)

# Move to high position
servo.pwm_high(sleep=1.0)
```

**Parameters:**

- **`sleep`** (float): Hold time in seconds (default: 0.5)

**Typical Movement:**
- 180° servos: ~90° clockwise from neutral
- Camera tilt: Look up
- Cargo release: Open position

### pwm_low()

Move servo to minimum position (500 µs).

```python
servo = Servo(gpio=14)

# Move to low position
servo.pwm_low(sleep=1.0)
```

**Parameters:**

- **`sleep`** (float): Hold time in seconds (default: 0.5)

**Typical Movement:**
- 180° servos: ~90° counter-clockwise from neutral
- Camera tilt: Look down
- Cargo release: Closed position

### Position Summary

| Method | Pulse Width | Typical Angle | Use Case |
|--------|------------|---------------|----------|
| `pwm_low()` | 500 µs | -90° | Minimum position |
| `pwm_neutral()` | 1500 µs | 0° | Center position |
| `pwm_high()` | 2000 µs | +90° | Maximum position |

## Custom Pulse Width Control

### set_pulsewidth()

Set servo to any custom pulse width.

```python
servo = Servo(gpio=14)

# Custom position (1250 µs)
servo.set_pulsewidth(pulsewidth=1250, sleep=0.5)
```

**Parameters:**

- **`pulsewidth`** (int): Pulse width in microseconds (500-2500 typical)
- **`sleep`** (float): Hold time in seconds (default: 0.5)

### Pulse Width Guide

```python
servo = Servo(gpio=14)

# Fine position control
servo.set_pulsewidth(1000)  # ~45° left
servo.set_pulsewidth(1250)  # ~22.5° left
servo.set_pulsewidth(1500)  # Center
servo.set_pulsewidth(1750)  # ~22.5° right
servo.set_pulsewidth(2000)  # ~45° right
```

### Safe Ranges

**Conservative (most servos):**
- Minimum: 600 µs
- Maximum: 2400 µs

**Standard (typical servos):**
- Minimum: 500 µs
- Maximum: 2500 µs

**⚠️ Warning:** Exceeding servo's physical limits can damage the servo!

### Smooth Sweep

```python
from colibricf.servo import Servo
import time

servo = Servo(gpio=14)

# Smooth sweep from low to high
for pulse in range(500, 2001, 50):
    servo.set_pulsewidth(pulsewidth=pulse, sleep=0.1)
    time.sleep(0.05)

print("Sweep complete!")
```

## Hardware Setup

### Wiring Diagram

```
Servo Motor Connections:
┌─────────────┐
│ Servo Motor │
│             │
│  Brown/Black── GND (Ground)
│  Red ─────────── +5V (Power)
│  Orange/Yellow── GPIO Signal
└─────────────┘

Raspberry Pi:
┌──────────────┐
│ Raspberry Pi │
│              │
│  Pin 6 (GND)────── Brown/Black
│  Pin 2 (+5V)────── Red
│  Pin 11 (GPIO17)── Orange/Yellow
└──────────────┘
```

### Power Considerations

**Important Power Guidelines:**

1. **Small Servos (1-2)**: Can be powered directly from Raspberry Pi
   - Current: < 500mA total
   - Use 5V rail

2. **Multiple/Large Servos**: Require external power supply
   - Use BEC (Battery Eliminator Circuit)
   - Share ground with Raspberry Pi
   - Separate 5V power source

```
External Power Setup:
┌──────┐      ┌─────────┐
│ BEC  │──+5V─│ Servo 1 │
│      │──+5V─│ Servo 2 │
│      │──+5V─│ Servo 3 │
└──┬───┘      └─────────┘
   │
   GND──┬── Raspberry Pi GND
        └── Servo GND (all)

Raspberry Pi:
  GPIO 17 ───── Servo 1 Signal
  GPIO 18 ───── Servo 2 Signal
  GPIO 22 ───── Servo 3 Signal
```

### Servo Specifications

Typical hobby servo specs:

| Specification | Value |
|--------------|-------|
| Operating Voltage | 4.8-6.0V |
| Current (idle) | 10-20mA |
| Current (moving) | 200-800mA |
| Speed | 0.1-0.4 sec/60° |
| Torque | 3-20 kg·cm |
| Weight | 25-50g |

## Applications

### Camera Gimbal Control

```python
from colibricf.servo import Servo
from colibricf.drone import Drone
import rospy

drone = Drone()
gimbal_pitch = Servo(gpio=14)
gimbal_roll = Servo(gpio=15)

# Initialize gimbal to neutral
gimbal_pitch.pwm_neutral()
gimbal_roll.pwm_neutral()

# Takeoff
drone.arm()
rospy.sleep(3)
drone.navigate_wait(z=5.0, auto_arm=True)

# Look down for aerial photography
gimbal_pitch.pwm_low(sleep=1.0)

# Capture image
rospy.sleep(2)

# Return gimbal to neutral
gimbal_pitch.pwm_neutral()

# Land
drone.land_wait()
```

### Payload Release Mechanism

```python
from colibricf.servo import Servo
from colibricf.drone import Drone
import rospy

drone = Drone()
release_servo = Servo(gpio=17)

# Ensure payload is secured
release_servo.pwm_low(sleep=0.5)
print("Payload secured")

# Navigate to drop location
drone.arm()
rospy.sleep(3)
drone.navigate_wait(z=10.0, auto_arm=True)
drone.navigate_wait(x=20, y=10, z=10.0)

# Release payload
print("Releasing payload...")
release_servo.pwm_high(sleep=1.0)
print("Payload released!")

# Return mechanism to safe position
rospy.sleep(2)
release_servo.pwm_neutral()

# Return and land
drone.navigate_wait(x=0, y=0, z=10.0)
drone.land_wait()
```

### Landing Gear Control

```python
from colibricf.servo import Servo
from colibricf.drone import Drone
import rospy

drone = Drone()
landing_gear = Servo(gpio=18)

# Retract landing gear
def retract_gear():
    landing_gear.pwm_high(sleep=1.0)
    print("Landing gear retracted")

# Extend landing gear
def extend_gear():
    landing_gear.pwm_low(sleep=1.0)
    print("Landing gear extended")

# Ensure gear is extended
extend_gear()

# Takeoff
drone.arm()
rospy.sleep(3)
drone.navigate_wait(z=2.0, auto_arm=True)

# Retract gear for flight
retract_gear()

# Flight operations...
rospy.sleep(10)

# Prepare for landing
extend_gear()
rospy.sleep(2)

# Land
drone.land_wait()
```

### Scanning Mechanism

```python
from colibricf.servo import Servo
from colibricf.camera import Camera
import cv2
import rospy

camera = Camera()
scanner_servo = Servo(gpio=14)

# Scan from left to right, capturing images
angles = [500, 1000, 1500, 2000, 2500]

for i, pulse in enumerate(angles):
    # Move servo to position
    scanner_servo.set_pulsewidth(pulsewidth=pulse, sleep=0.5)
    rospy.sleep(0.5)
    
    # Capture image
    frame = camera.retrieve_cv_frame()
    cv2.imwrite(f'/home/pi/scan/scan_{i}.jpg', frame)
    print(f"Captured scan {i}")

# Return to neutral
scanner_servo.pwm_neutral()
```

## Troubleshooting

### Servo Not Moving

**Possible Causes:**

1. **pigpio daemon not running**
   ```bash
   # Check if running
   sudo systemctl status pigpiod
   
   # Start if not running
   sudo systemctl start pigpiod
   
   # Enable on boot
   sudo systemctl enable pigpiod
   ```

2. **Incorrect GPIO pin**
   ```python
   # Verify pin number
   servo = Servo(gpio=17)  # Check this matches physical connection
   ```

3. **Insufficient power**
   - Check servo power supply
   - Measure voltage (should be 4.8-6V)
   - Use external power for multiple servos

4. **Damaged servo**
   - Test servo with different pulse widths
   - Try on different GPIO pin
   - Test with servo tester

### Servo Jittering

**Solutions:**

1. **Add capacitor**
   - 470µF capacitor across power rails
   - Reduces noise from power supply

2. **Use external power**
   - Separate power supply for servos
   - Share ground with Raspberry Pi

3. **Software fixes**
   ```python
   # Increase hold time
   servo.pwm_neutral(sleep=1.0)  # Instead of 0.5
   
   # Disable servo when not in use
   # (pigpio automatically handles this)
   ```

### Servo Buzzing

**Causes & Solutions:**

- **Not at exact neutral**: Fine-tune pulse width
  ```python
  servo.set_pulsewidth(1500)  # Try 1480-1520 range
  ```

- **Mechanically bound**: Check for obstructions

- **Wrong pulse width**: Verify servo specifications

### Limited Range of Motion

**Solutions:**

1. **Check servo type**
   - 180° servos: 500-2500 µs
   - 90° servos: 1000-2000 µs

2. **Adjust pulse width range**
   ```python
   # For 90° servo
   servo.set_pulsewidth(1000)  # Left
   servo.set_pulsewidth(2000)  # Right
   ```

3. **Mechanical stops**
   - Check for physical obstructions
   - Ensure servo isn't mechanically limited

## Complete Examples

### Dual-Axis Gimbal

```python
from colibricf.servo import Servo
from colibricf.drone import Drone
import rospy
import time

drone = Drone()
pitch_servo = Servo(gpio=14)
roll_servo = Servo(gpio=15)

# Initialize gimbal
pitch_servo.pwm_neutral()
roll_servo.pwm_neutral()
time.sleep(1)

# Takeoff
drone.arm()
rospy.sleep(3)
drone.navigate_wait(z=3.0, auto_arm=True)

# Pan scan (roll left-right)
print("Pan scan...")
roll_servo.pwm_low(sleep=1.0)
roll_servo.pwm_neutral(sleep=1.0)
roll_servo.pwm_high(sleep=1.0)
roll_servo.pwm_neutral(sleep=1.0)

# Tilt scan (pitch up-down)
print("Tilt scan...")
pitch_servo.pwm_low(sleep=1.0)
pitch_servo.pwm_neutral(sleep=1.0)
pitch_servo.pwm_high(sleep=1.0)
pitch_servo.pwm_neutral(sleep=1.0)

# Land
drone.land_wait()
```

### Automated Delivery System

```python
from colibricf.servo import Servo
from colibricf.drone import Drone
import rospy

drone = Drone()
door_servo = Servo(gpio=17)

# Delivery coordinates
delivery_location = (20, 15)  # meters from start

# Close delivery door
door_servo.pwm_low(sleep=0.5)
print("Delivery door closed, payload secured")

# Navigate to delivery point
drone.arm()
rospy.sleep(3)
drone.navigate_wait(z=5.0, auto_arm=True)
drone.navigate_wait(x=delivery_location[0], y=delivery_location[1], z=5.0)

# Hover and deliver
print("Hovering over delivery point...")
rospy.sleep(3)

# Open door to release package
print("Delivering package...")
door_servo.pwm_high(sleep=2.0)
print("Package delivered!")

# Close door
door_servo.pwm_low(sleep=0.5)

# Return home
drone.navigate_wait(x=0, y=0, z=5.0)
drone.land_wait()
```

### Sequential Servo Control

```python
from colibricf.servo import Servo
import time

# Control multiple servos in sequence
servos = [
    Servo(gpio=14),
    Servo(gpio=15),
    Servo(gpio=17),
]

# Initialize all to neutral
for servo in servos:
    servo.pwm_neutral()
    time.sleep(0.2)

print("All servos initialized")

# Wave pattern
for _ in range(3):
    for servo in servos:
        servo.pwm_high(sleep=0.3)
        servo.pwm_low(sleep=0.3)
        servo.pwm_neutral(sleep=0.3)

print("Wave complete")
```

---

## See Also

- [Drone Module](Drone-Module.md) - Integrate with flight control
- [Task Framework](Task-Framework.md) - Use servos in missions
- [Camera Module](Camera-Module.md) - Gimbal for camera pointing
- [Examples and Tutorials](Examples-and-Tutorials.md) - More practical applications

[← Back to Home](Home.md) | [Next: Task Framework →](Task-Framework.md)
