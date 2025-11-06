# Getting Started with Colibri Code Functions

This guide will help you install, configure, and run your first Colibri drone mission.

## Table of Contents

- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Hardware Setup](#hardware-setup)
- [Configuration](#configuration)
- [Your First Flight](#your-first-flight)
- [Next Steps](#next-steps)

## Prerequisites

Before installing Colibri Code Functions, ensure you have:

### Hardware Requirements
- **Colibri Drone** (or compatible drone with Clover framework)
- **Raspberry Pi** (3B+ or 4 recommended) installed on the drone
- **Camera Module** (Raspberry Pi Camera or compatible)
- **MicroSD Card** (16GB or larger, Class 10 recommended)
- **Power Supply** appropriate for your drone
- **Computer** for development and drone connection

### Software Requirements
- **Operating System**: Raspbian/Ubuntu Linux (20.04 or later)
- **Python**: Version 3.7 or higher
- **ROS**: Melodic or Noetic
- **Clover Framework**: Must be installed and configured

### Skills Required
- Basic Python programming knowledge
- Understanding of drone operation fundamentals
- Familiarity with command-line interface (CLI)

## Installation

### Step 1: Install Clover Framework

First, ensure the Clover framework is properly installed on your drone's Raspberry Pi. Follow the official guide:

🔗 [Clover Installation Guide](https://clover.coex.tech/en/installation.html)

To verify Clover is installed, run:

```bash
roscore &
roslaunch clover clover.launch
```

### Step 2: Install Colibri Code Functions

There are three methods to install Colibri Code Functions:

#### Method 1: Install via pip (Recommended)

```bash
pip3 install colibricf
```

#### Method 2: Install from GitHub

```bash
# Clone the repository
git clone https://github.com/EDUCA-DRONES/Colibri_CF.git

# Navigate to directory
cd Colibri_CF

# Install the package
pip3 install .
```

#### Method 3: Install in Development Mode

For developers who want to modify the code:

```bash
# Clone the repository
git clone https://github.com/EDUCA-DRONES/Colibri_CF.git

# Navigate to directory
cd Colibri_CF

# Install in editable mode
pip3 install -e .
```

### Step 3: Verify Installation

Verify that the installation was successful:

```bash
python3 -c "import colibricf; print('Colibri CF installed successfully!')"
```

### Step 4: Install Additional Dependencies (Optional)

For computer vision features, install:

```bash
# For QR code detection
pip3 install pyzbar

# For face detection and following
pip3 install mediapipe

# For image EXIF data
pip3 install piexif
```

## Hardware Setup

### Physical Setup

1. **Assemble the Drone**: Follow manufacturer instructions
2. **Mount the Raspberry Pi**: Securely attach to the drone frame
3. **Connect the Camera**: Attach to Raspberry Pi CSI port
4. **Connect Servos** (optional): Connect to GPIO pins as needed
5. **Install Propellers**: Ensure correct rotation direction
6. **Battery Check**: Verify battery is charged and properly connected

### Wiring Diagram

```
Raspberry Pi GPIO Connections:
┌─────────────────┐
│  Raspberry Pi   │
│                 │
│  GPIO 14 ───────┼─── Servo 1 (Signal)
│  GPIO 15 ───────┼─── Servo 2 (Signal)
│  GPIO 17 ───────┼─── Optional Servo
│  CSI Port ──────┼─── Camera Module
│  USB ───────────┼─── Flight Controller
└─────────────────┘
```

### Camera Setup

Enable the camera on Raspberry Pi:

```bash
sudo raspi-config
# Navigate to: Interface Options → Camera → Enable
sudo reboot
```

Test the camera:

```bash
raspistill -o test.jpg
```

## Configuration

### ROS Environment Setup

Add to your `~/.bashrc`:

```bash
# ROS Setup
source /opt/ros/noetic/setup.bash  # or melodic
source ~/catkin_ws/devel/setup.bash

# Clover Configuration
export ROS_HOSTNAME=192.168.11.1  # Raspberry Pi IP
export ROS_MASTER_URI=http://192.168.11.1:11311
```

Apply changes:

```bash
source ~/.bashrc
```

### Network Configuration

Ensure your computer can communicate with the drone:

```bash
# Ping the drone
ping 192.168.11.1

# Test ROS connection
rostopic list
```

### Clover Configuration

Check that Clover services are running:

```bash
# List available services
rosservice list | grep clover

# Expected services:
# /get_telemetry
# /navigate
# /land
# /set_position
# etc.
```

## Your First Flight

### Safety First! ⚠️

Before flying:
- [ ] Clear area of obstacles and people
- [ ] Battery is fully charged
- [ ] Propellers are securely attached
- [ ] Emergency stop is ready (unplug battery)
- [ ] Flight controller is calibrated
- [ ] Compass is calibrated (if using GPS)

### Simple Takeoff and Land

Create a file named `first_flight.py`:

```python
#!/usr/bin/env python3

import rospy
from colibricf.drone import Drone

# Initialize the drone
drone = Drone()

print("Starting first flight mission...")

# Arm the motors
print("Arming motors...")
drone.arm()
rospy.sleep(3)

# Take off to 1 meter
print("Taking off to 1 meter...")
drone.navigate_wait(x=0, y=0, z=1.0, frame_id='body', auto_arm=True)

# Hover for 5 seconds
print("Hovering...")
rospy.sleep(5)

# Land
print("Landing...")
drone.land_wait()

print("Mission complete!")
```

Run the script:

```bash
chmod +x first_flight.py
python3 first_flight.py
```

### Understanding the Code

```python
# Initialize ROS node and drone services
drone = Drone()

# Arm the motors (required before flight)
drone.arm()
rospy.sleep(3)  # Wait for arming

# Navigate to position (relative to current position)
drone.navigate_wait(
    x=0,           # No movement in X
    y=0,           # No movement in Y  
    z=1.0,         # Move 1 meter up
    frame_id='body',  # Relative to drone
    auto_arm=True  # Auto-arm if needed
)

# Land and wait until fully landed
drone.land_wait()
```

### Simple Movement Example

Move in a square pattern:

```python
from colibricf.drone import Drone
import rospy

drone = Drone()

# Takeoff
drone.arm()
rospy.sleep(3)
drone.navigate_wait(z=1.0, auto_arm=True)

# Square pattern (2m sides)
print("Moving forward...")
drone.navigate_wait(x=2, y=0, z=1.0)

print("Moving right...")
drone.navigate_wait(x=2, y=2, z=1.0)

print("Moving backward...")
drone.navigate_wait(x=0, y=2, z=1.0)

print("Moving left...")
drone.navigate_wait(x=0, y=0, z=1.0)

# Land
drone.land_wait()
```

## Troubleshooting Common Issues

### Issue: "roscore not running"

**Solution:**
```bash
# Start roscore in a separate terminal
roscore &

# Or start Clover launch file
roslaunch clover clover.launch
```

### Issue: "Service /get_telemetry not available"

**Solution:**
```bash
# Restart Clover
sudo systemctl restart clover

# Or manually launch
roslaunch clover clover.launch
```

### Issue: "Drone doesn't arm"

**Possible causes:**
- Flight controller not calibrated
- Battery voltage too low
- Safety switch not disabled
- Flight mode incorrect

**Solution:**
```bash
# Check battery voltage
rostopic echo /mavros/battery

# Check flight controller status
rostopic echo /mavros/state
```

### Issue: "ModuleNotFoundError: No module named 'colibricf'"

**Solution:**
```bash
# Verify installation
pip3 list | grep colibricf

# Reinstall if needed
pip3 install --upgrade colibricf
```

### Issue: "Camera not found"

**Solution:**
```bash
# Test camera
raspistill -o test.jpg

# Check camera is enabled
vcgencmd get_camera

# Should show: supported=1 detected=1
```

## Next Steps

Now that you have your first flight working, explore:

1. **[Drone Module](Drone-Module.md)** - Learn advanced flight control
2. **[Task Framework](Task-Framework.md)** - Create complex missions
3. **[Camera Module](Camera-Module.md)** - Add computer vision
4. **[Examples and Tutorials](Examples-and-Tutorials.md)** - More practical examples

## Additional Resources

- **Clover Documentation**: https://clover.coex.tech/en/
- **ROS Tutorials**: http://wiki.ros.org/ROS/Tutorials
- **PX4 Documentation**: https://docs.px4.io/
- **GitHub Repository**: https://github.com/EDUCA-DRONES/Colibri_CF

## Getting Help

If you encounter issues:

1. Check the [Troubleshooting Guide](Troubleshooting.md)
2. Review the [FAQ](FAQ.md)
3. Search [GitHub Issues](https://github.com/EDUCA-DRONES/Colibri_CF/issues)
4. Open a new issue with detailed information

---

**Happy Flying! 🚁**

[← Back to Home](Home.md) | [Next: Drone Module →](Drone-Module.md)
