# Frequently Asked Questions (FAQ)

Common questions about Colibri Code Functions.

## General Questions

### What is Colibri Code Functions?

Colibri Code Functions is a Python library that simplifies drone programming by providing an easy-to-use interface for the Clover framework. It handles complex ROS services and provides intuitive methods for flight control, camera operations, and autonomous missions.

### Do I need a Colibri drone to use this library?

While optimized for Colibri drones, the library works with any drone running the Clover framework. This includes DIY drones built with:
- Raspberry Pi (3B+ or 4)
- PX4 or ArduPilot flight controller
- Clover framework installed

### What's the difference between Colibri CF and Clover?

- **Clover** is the underlying framework providing ROS services for drone control
- **Colibri CF** is a Python wrapper that simplifies Clover's API with intuitive methods
- Colibri CF uses Clover services but provides a more beginner-friendly interface

### Is this library suitable for beginners?

Yes! Colibri CF is designed to be beginner-friendly while still powerful for advanced users. The Task framework and high-level methods make it easy to:
- Write your first flight in minutes
- Focus on mission logic rather than low-level details
- Progress from simple to complex missions

## Installation & Setup

### What are the minimum requirements?

**Hardware:**
- Raspberry Pi 3B+ or 4
- Compatible flight controller (PX4/ArduPilot)
- Camera module (optional)
- Servo motors (optional)

**Software:**
- Linux (Raspbian/Ubuntu)
- Python 3.7+
- ROS (Melodic or Noetic)
- Clover framework

### Can I run this on my computer for simulation?

Yes, but you need:
1. ROS installed on your computer
2. Clover simulation environment
3. Gazebo for 3D simulation

Follow the Clover simulation guide: https://clover.coex.tech/en/simulation.html

### How do I update to the latest version?

```bash
pip3 install --upgrade colibricf
```

## Flight Control

### What coordinate systems does the library use?

Three main reference frames:

1. **'body'**: Relative to drone's current position and orientation
   ```python
   # Move 2m forward relative to drone
   drone.navigate_wait(x=2, y=0, z=1.0, frame_id='body')
   ```

2. **'map'**: Relative to takeoff point (global reference)
   ```python
   # Move to absolute position from takeoff
   drone.navigate_wait(x=5, y=3, z=2.0, frame_id='map')
   ```

3. **'navigate_target'**: Relative to last navigation target
   ```python
   drone.navigate_wait(x=1, y=1, z=1.0, frame_id='navigate_target')
   ```

### How do I convert between coordinate frames?

Use the telemetry system:

```python
# Get position in different frames
telem_body = drone.get_telemetry(frame_id='body')
telem_map = drone.get_telemetry(frame_id='map')

print(f"Body: ({telem_body.x}, {telem_body.y}, {telem_body.z})")
print(f"Map: ({telem_map.x}, {telem_map.y}, {telem_map.z})")
```

### What's the maximum flight range?

Depends on:
- **Battery capacity**: Typical 5-15 minutes flight time
- **WiFi range**: ~100m for manual control
- **GPS accuracy**: For autonomous GPS navigation
- **Regulations**: Follow local drone laws

Always test in safe, controlled environments first.

### Can I fly indoors?

Yes, with proper setup:

1. **Enable optical flow:**
   ```python
   drone.toggle_optical_flow()
   ```

2. **Use ArUco markers:**
   ```python
   drone.toggle_aruco()
   # Place markers on floor/walls
   ```

3. **Or use good lighting for optical flow**

Indoor flight is generally more challenging than outdoor GPS flight.

### What happens if battery runs low?

The drone should:
1. Trigger low battery failsafe
2. Automatically return to launch (RTL)
3. Land at launch position

**Best practice:** Monitor battery in your code:
```python
telem = drone.get_telemetry()
if telem.battery_voltage < 11.0:
    print("Battery low! Landing...")
    drone.land_wait()
```

## Camera & Computer Vision

### What resolution is the camera?

Depends on your camera module. Raspberry Pi Camera v2:
- Still images: Up to 3280 × 2464
- Video: 1080p at 30fps, 720p at 60fps

Access via OpenCV provides ~640×480 by default.

### Can I use multiple cameras?

Clover supports one main camera by default (`/main_camera/image_raw`). To use multiple cameras:
1. Add additional camera topics in ROS
2. Modify camera.py to subscribe to different topics
3. Create separate Camera instances for each

### How accurate is face detection?

Depends on:
- **Distance**: Works best 1-5 meters
- **Lighting**: Needs good, even lighting
- **Angle**: Best when face is front-facing
- **Movement**: Slower movement improves detection

Typical accuracy: 70-90% in good conditions.

### Can I train custom object detection models?

Yes! Use TensorFlow, PyTorch, or OpenCV with custom models:

```python
import cv2
from colibricf.camera import Camera

# Load your custom model
net = cv2.dnn.readNet('custom_model.weights', 'config.cfg')

camera = Camera()
frame = camera.retrieve_cv_frame()

# Run inference
# ... your custom detection code
```

## Task Framework

### Do I have to use the Task framework?

No, it's optional but recommended because it:
- Provides automatic error handling
- Ensures drone lands safely on errors
- Handles Ctrl+C gracefully
- Structures your code cleanly

You can use direct Drone() calls:
```python
drone = Drone()
drone.arm()
drone.navigate_wait(z=1.0, auto_arm=True)
# etc.
```

### Can I run multiple tasks sequentially?

Yes:

```python
task1 = SurveyMission()
task1.run()

rospy.sleep(5)

task2 = DeliveryMission()
task2.run()
```

But be careful with ROS node names (each creates a node).

### How do I pass parameters to tasks?

Through the constructor:

```python
class ParameterizedTask(Task):
    def __init__(self, altitude=5.0, speed=1.0, waypoints=None):
        super().__init__()
        self.altitude = altitude
        self.speed = speed
        self.waypoints = waypoints or []
    
    def mission(self):
        # Use self.altitude, self.speed, etc.
        pass

# Create with custom parameters
task = ParameterizedTask(altitude=10.0, speed=2.0, waypoints=my_waypoints)
task.run()
```

## Servo Control

### What servos are compatible?

Standard hobby servos (50Hz PWM):
- 180° rotation servos
- 90° servos
- Continuous rotation servos

**Examples:**
- SG90 (micro servo)
- MG996R (standard servo)
- HS-422 (standard servo)

### How many servos can I control?

Limited by available GPIO pins. Raspberry Pi has 20+ GPIO pins, but:
- Some pins are used by other functions
- Recommended: Use GPIO 14, 15, 17, 18, 22, 23, 24, 25
- Practical limit: 4-8 servos depending on configuration

### Do servos drain battery?

Yes, servos consume power:
- Idle: 10-20mA per servo
- Moving: 200-800mA per servo
- Multiple servos can significantly impact battery life

**Solution:** Use external power supply (BEC) for servos.

## Safety & Regulations

### Is it safe to fly autonomously?

Autonomous flight can be safe if you:
- ✅ Test in controlled environment first
- ✅ Have manual override ready
- ✅ Monitor battery and telemetry
- ✅ Use proper error handling (Task framework)
- ✅ Start with simple missions
- ✅ Keep drone in visual line of sight

Always follow safety guidelines!

### Do I need a license to fly?

Depends on your country and use case:
- **USA**: FAA Part 107 for commercial use
- **Europe**: Varies by country (A1/A2/A3 categories)
- **Other**: Check local regulations

**Recreational use** often has different (lighter) requirements.

### What are the weight restrictions?

Varies by jurisdiction:
- **USA**: <0.55 lbs (250g) for Part 107 exemption
- **Europe**: Categories based on weight
- Most Colibri drones: 200-500g depending on configuration

### Can I fly near people/buildings?

Usually NO without special permissions:
- Maintain safe distance from people
- No flights over crowds
- Respect privacy
- Follow local airspace restrictions

Check your local drone regulations!

## Troubleshooting

### Why won't my drone arm?

Common reasons:
1. Low battery (<10.5V)
2. Flight controller not calibrated
3. Safety switch not disabled
4. Sensors failing pre-arm checks

See [Troubleshooting Guide](Troubleshooting.md) for solutions.

### The drone drifts when hovering. Why?

Possible causes:
1. Gyroscope needs calibration
2. Optical flow disabled (indoor)
3. Poor GPS signal (outdoor)
4. Motor imbalance
5. Propeller damage

### My Python script hangs. What should I do?

1. **Check for blocking calls:**
   - `rospy.spin()` blocks forever
   - `drone.orbit()` is blocking

2. **Add timeouts:**
   ```python
   import signal
   signal.alarm(300)  # 5 minute timeout
   ```

3. **Use Ctrl+C:**
   - Task framework handles it gracefully
   - Drone will land safely

## Development

### Can I contribute to the project?

Yes! See [Contributing Guide](Contributing.md).

We welcome:
- Bug fixes
- New features
- Documentation improvements
- Examples and tutorials
- Bug reports

### How do I report a bug?

1. Check existing issues: https://github.com/EDUCA-DRONES/Colibri_CF/issues
2. If new, create issue with:
   - Clear description
   - Code to reproduce
   - Error messages
   - System info (Python version, ROS version, etc.)

### Where can I get help?

1. **Documentation**: Check all wiki pages
2. **GitHub Issues**: Search existing issues
3. **Clover Docs**: https://clover.coex.tech/en/
4. **ROS Answers**: https://answers.ros.org/

### Can I use this commercially?

Yes! Licensed under MIT License:
- ✅ Commercial use allowed
- ✅ Modification allowed
- ✅ Distribution allowed
- ✅ Private use allowed
- ⚠️ No warranty provided

See [LICENSE](https://github.com/EDUCA-DRONES/Colibri_CF/blob/main/LICENSE) for full terms.

---

## Still Have Questions?

- **GitHub Discussions**: https://github.com/EDUCA-DRONES/Colibri_CF/discussions
- **Issues**: https://github.com/EDUCA-DRONES/Colibri_CF/issues
- **Clover Community**: https://clover.coex.tech/en/

[← Back to Home](Home.md) | [Next: Contributing →](Contributing.md)
