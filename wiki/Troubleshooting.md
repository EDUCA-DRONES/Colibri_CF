# Troubleshooting Guide

Common issues and solutions for Colibri Code Functions.

## Table of Contents

- [Installation Issues](#installation-issues)
- [Connection Problems](#connection-problems)
- [Flight Issues](#flight-issues)
- [Camera Problems](#camera-problems)
- [Servo Issues](#servo-issues)
- [ROS Problems](#ros-problems)
- [Performance Issues](#performance-issues)

## Installation Issues

### "ModuleNotFoundError: No module named 'colibricf'"

**Problem:** Package not installed or not in Python path.

**Solutions:**

1. Verify installation:
```bash
pip3 list | grep colibricf
```

2. Reinstall:
```bash
pip3 install --upgrade colibricf
```

3. Check Python version:
```bash
python3 --version  # Should be 3.7+
```

4. Install from source:
```bash
git clone https://github.com/EDUCA-DRONES/Colibri_CF.git
cd Colibri_CF
pip3 install -e .
```

### "ImportError: No module named 'rospy'"

**Problem:** ROS not installed or sourced.

**Solutions:**

1. Install ROS:
```bash
# Follow official ROS installation
# http://wiki.ros.org/ROS/Installation
```

2. Source ROS environment:
```bash
source /opt/ros/noetic/setup.bash  # or melodic
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```

3. Verify ROS:
```bash
roscore &
# Should start without errors
```

### "ImportError: No module named 'clover'"

**Problem:** Clover framework not installed.

**Solution:**

```bash
# Follow Clover installation guide
# https://clover.coex.tech/en/installation.html
```

## Connection Problems

### Cannot Connect to Drone

**Problem:** Unable to communicate with drone.

**Diagnostic Steps:**

1. **Check network connection:**
```bash
ping 192.168.11.1  # Default Clover IP
```

2. **Verify ROS master:**
```bash
echo $ROS_MASTER_URI
# Should be: http://192.168.11.1:11311
```

3. **Check ROS topics:**
```bash
rostopic list
# Should show /mavros/*, /main_camera/*, etc.
```

4. **Test services:**
```bash
rosservice list | grep clover
# Should show /get_telemetry, /navigate, etc.
```

**Solutions:**

1. Configure ROS environment:
```bash
export ROS_MASTER_URI=http://192.168.11.1:11311
export ROS_HOSTNAME=<your_computer_ip>
```

2. Restart Clover services:
```bash
ssh pi@192.168.11.1
sudo systemctl restart clover
```

### "Service /get_telemetry not available"

**Problem:** Clover services not running.

**Solutions:**

1. Check Clover status:
```bash
ssh pi@192.168.11.1
systemctl status clover
```

2. Restart Clover:
```bash
sudo systemctl restart clover
```

3. Check logs:
```bash
journalctl -u clover -n 50
```

## Flight Issues

### Drone Won't Arm

**Problem:** Motors don't arm when calling `drone.arm()`.

**Possible Causes & Solutions:**

1. **Safety checks failing:**
```python
# Check telemetry
telem = drone.get_telemetry()
print(f"Battery: {telem.battery_voltage}V")
print(f"Mode: {telem.mode}")

# Battery must be >10.5V typically
```

2. **Flight controller not calibrated:**
```bash
# Calibrate accelerometer and compass
# Use QGroundControl or Mission Planner
```

3. **Wrong flight mode:**
```python
from colibricf.drone import DroneMode
drone.set_mode(DroneMode.OFFBOARD)
```

4. **Flight controller error:**
```bash
# Check MAVROS status
rostopic echo /mavros/state
```

### Drone Drifts or Won't Hold Position

**Problem:** Unstable hovering or position drift.

**Solutions:**

1. **Calibrate sensors:**
```python
drone.calibrate_gyro()
```

2. **Check optical flow:**
```python
# Enable optical flow for indoor flight
drone.toggle_optical_flow()
```

3. **Check ArUco markers:**
```python
# Enable for precise positioning
drone.toggle_aruco()
```

4. **Verify GPS (outdoor):**
```bash
rostopic echo /mavros/global_position/global
# Should show valid coordinates
```

5. **Check motor balance:**
```bash
# Ensure all motors are working equally
# Check propeller condition
```

### Navigation Commands Ignored

**Problem:** `navigate_wait()` doesn't move drone.

**Solutions:**

1. **Check if armed:**
```python
telem = drone.get_telemetry()
if not telem.armed:
    drone.arm()
    rospy.sleep(3)
```

2. **Verify frame_id:**
```python
# Use correct frame
drone.navigate_wait(x=1, y=0, z=1.5, frame_id='body')  # Relative
# vs
drone.navigate_wait(x=1, y=0, z=1.5, frame_id='map')  # Absolute
```

3. **Check OFFBOARD mode:**
```python
from colibricf.drone import DroneMode
drone.set_mode(DroneMode.OFFBOARD)
```

### Drone Lands Unexpectedly

**Problem:** Drone lands during mission.

**Causes:**

1. **Low battery:** Check voltage
2. **Timeout:** Mission taking too long
3. **Failsafe triggered:** Lost connection
4. **Code exception:** Error in mission

**Solutions:**

1. **Monitor battery:**
```python
telem = drone.get_telemetry()
if telem.battery_voltage < 11.0:
    print("Battery low, landing...")
    drone.land_wait()
```

2. **Use Task framework:**
```python
# Automatic error handling
class MyMission(Task):
    def mission(self):
        # Your code here
        pass

task = MyMission()
task.run()  # Auto-lands on error
```

## Camera Problems

### "Camera not found" or No Image

**Problem:** Cannot retrieve camera frames.

**Solutions:**

1. **Check camera hardware:**
```bash
raspistill -o test.jpg
# Should capture image
```

2. **Enable camera:**
```bash
sudo raspi-config
# Interface Options → Camera → Enable
sudo reboot
```

3. **Check camera topic:**
```bash
rostopic list | grep camera
# Should show /main_camera/image_raw
```

4. **Test camera topic:**
```bash
rostopic echo /main_camera/image_raw --noarr
# Should show messages
```

5. **View camera in rqt:**
```bash
rqt_image_view
# Select /main_camera/image_raw
```

### Face Detection Not Working

**Problem:** Faces not detected.

**Solutions:**

1. **Check lighting:** Ensure good lighting conditions

2. **Verify Haar Cascade:**
```python
import cv2
import os

# Check cascade file exists
cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
print(f"Cascade exists: {os.path.exists(cascade_path)}")
```

3. **Adjust detection parameters:**
```python
faces = cascade.detectMultiScale(
    gray,
    scaleFactor=1.2,  # Try different values (1.1-1.5)
    minNeighbors=3    # Try 3-6
)
```

4. **Test with saved image:**
```python
frame = cv2.imread('test_face.jpg')
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
faces = detectFace(gray)
print(f"Detected {len(faces)} faces")
```

### QR Codes Not Detected

**Problem:** QR code scanner not finding codes.

**Solutions:**

1. **Install pyzbar:**
```bash
sudo apt-get install libzbar0
pip3 install pyzbar
```

2. **Check QR code quality:**
   - Code should be clear and well-lit
   - Minimum size: 5cm at 1m distance
   - Good contrast (black on white)

3. **Test with static image:**
```python
from pyzbar import pyzbar
import cv2

image = cv2.imread('qr_test.jpg')
codes = pyzbar.decode(image)
print(f"Found {len(codes)} codes")
```

## Servo Issues

### Servo Not Moving

**Problem:** Servo doesn't respond to commands.

**Solutions:**

1. **Check pigpio daemon:**
```bash
sudo systemctl status pigpiod
# Should be active

# Start if needed
sudo systemctl start pigpiod
sudo systemctl enable pigpiod
```

2. **Verify GPIO pin:**
```python
# Check pin number is correct
servo = Servo(gpio=17)  # Verify this matches wiring
```

3. **Test servo:**
```python
from colibricf.servo import Servo

servo = Servo(gpio=14)
servo.pwm_neutral()  # Should move to center
```

4. **Check power:**
   - Servo has 5V power
   - Ground is connected
   - Signal wire on correct GPIO

5. **Test with different pulse widths:**
```python
for pw in [500, 1000, 1500, 2000, 2500]:
    servo.set_pulsewidth(pw)
    rospy.sleep(1)
```

### Servo Jitters or Shakes

**Problem:** Servo vibrates or is unstable.

**Solutions:**

1. **Add capacitor:**
   - 470µF across power rails
   - Reduces electrical noise

2. **Use external power:**
   - Don't power from Raspberry Pi
   - Use BEC or dedicated 5V supply

3. **Increase sleep time:**
```python
servo.pwm_neutral(sleep=1.0)  # Longer hold time
```

4. **Check for mechanical binding:**
   - Ensure servo can move freely
   - No excessive load

## ROS Problems

### "roscore not running"

**Solution:**

```bash
# Start roscore
roscore &

# Or start Clover (includes roscore)
roslaunch clover clover.launch
```

### ROS Node Already Running

**Problem:** "Node [/flight] already running"

**Solutions:**

1. **Kill existing node:**
```bash
rosnode kill /flight
```

2. **Use unique node name:**
```python
drone = Drone(node_name="my_unique_name")
```

3. **Restart all ROS nodes:**
```bash
killall rosmaster roscore
roscore &
```

### Topic Not Publishing

**Problem:** Expected ROS topic not available.

**Solutions:**

1. **List topics:**
```bash
rostopic list
```

2. **Check topic info:**
```bash
rostopic info /main_camera/image_raw
```

3. **Monitor topic:**
```bash
rostopic hz /main_camera/image_raw
# Shows publishing rate
```

4. **Restart publishers:**
```bash
ssh pi@192.168.11.1
sudo systemctl restart clover
```

## Performance Issues

### Slow Frame Rate

**Problem:** Camera capture is slow.

**Solutions:**

1. **Use throttled topic:**
```python
# Instead of '/main_camera/image_raw'
# Use '/main_camera/image_raw_throttled'
```

2. **Reduce resolution:**
```python
import cv2
frame = camera.retrieve_cv_frame()
small = cv2.resize(frame, (320, 240))
# Process small frame
```

3. **Limit processing rate:**
```python
rate = rospy.Rate(10)  # 10 Hz instead of maximum
while not rospy.is_shutdown():
    # Process
    rate.sleep()
```

### High CPU Usage

**Problem:** Raspberry Pi overheating or slow.

**Solutions:**

1. **Optimize image processing:**
```python
# Process ROI instead of full frame
h, w = frame.shape[:2]
roi = frame[h//4:3*h//4, w//4:3*w//4]
```

2. **Use GPU acceleration:**
```bash
# Install OpenCV with GPU support
# Or use hardware-accelerated codecs
```

3. **Reduce processing complexity:**
   - Lower image resolution
   - Simplify algorithms
   - Skip frames

### Memory Leaks

**Problem:** Memory usage increases over time.

**Solutions:**

1. **Release OpenCV windows:**
```python
cv2.destroyAllWindows()
```

2. **Clear large variables:**
```python
frame = None
del frame
```

3. **Monitor memory:**
```bash
top  # Watch memory usage
free -h  # Check available memory
```

---

## Getting Further Help

If issues persist:

1. **Check GitHub Issues:**
   - Search: https://github.com/EDUCA-DRONES/Colibri_CF/issues
   - Open new issue with:
     - Full error message
     - Code that reproduces problem
     - System information

2. **Review Logs:**
```bash
# System logs
journalctl -u clover -n 100

# ROS logs
cat ~/.ros/log/latest/*.log
```

3. **Clover Documentation:**
   - https://clover.coex.tech/en/

4. **ROS Documentation:**
   - http://wiki.ros.org/

---

[← Back to Home](Home.md) | [Next: FAQ →](FAQ.md)
