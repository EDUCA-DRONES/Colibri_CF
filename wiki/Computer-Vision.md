# Computer Vision Guide

Advanced computer vision capabilities for Colibri drones including face detection, person following, QR code scanning, and custom image processing pipelines.

## Table of Contents

- [Overview](#overview)
- [Face Detection](#face-detection)
- [Person Following](#person-following)
- [QR Code Detection](#qr-code-detection)
- [Custom CV Pipelines](#custom-cv-pipelines)
- [Integration with Flight](#integration-with-flight)
- [Performance Optimization](#performance-optimization)
- [Complete Applications](#complete-applications)

## Overview

Colibri Code Functions integrates powerful computer vision features through OpenCV, MediaPipe, and PyZBar libraries.

**Capabilities:**
- ✅ Real-time face detection using Haar Cascades
- ✅ Person pose estimation and following
- ✅ QR code and barcode scanning
- ✅ Custom OpenCV processing pipelines
- ✅ Vision-based navigation
- ✅ Object tracking and detection

**Requirements:**
```bash
# Core dependencies
pip3 install opencv-python
pip3 install numpy

# Face detection (included)
# Uses Haar Cascade classifiers

# Person following
pip3 install mediapipe

# QR code detection
pip3 install pyzbar
```

## Face Detection

### Using Built-in Face Detection

The Camera module includes Haar Cascade-based face detection:

```python
from colibricf.camera import Camera
import rospy

rospy.init_node('face_detection')
camera = Camera()

# Start face detection with visualization
camera.draw_face()
```

**Output:**
- Detects faces in camera view
- Draws bounding boxes around faces
- Shows coordinates of face centers
- Publishes annotated images to ROS topic

### Viewing Face Detection Results

```bash
# Terminal 1: Run face detection
python3 face_detect_script.py

# Terminal 2: View results
rqt_image_view
# Select topic: ~face_detect/debug
```

### Custom Face Detection

```python
import cv2
import rospy
from colibricf.camera import Camera
from colibricf.cv.face_detect import detectFace

camera = Camera()

while not rospy.is_shutdown():
    # Get frame
    frame = camera.retrieve_cv_frame()
    
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detect faces
    faces = detectFace(gray)
    
    # Process each face
    for (x, y, w, h) in faces:
        # Draw rectangle
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        
        # Get center point
        center_x = x + w // 2
        center_y = y + h // 2
        
        # Draw center
        cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
        
        # Display coordinates
        cv2.putText(frame, f"Face at ({center_x},{center_y})", 
                   (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 
                   0.5, (0, 255, 0), 2)
    
    # Show result
    cv2.imshow('Face Detection', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
```

### Face Detection Parameters

Customize detection sensitivity:

```python
import cv2

# Load cascade classifier
cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
face_cascade = cv2.CascadeClassifier(cascade_path)

# Detect with custom parameters
faces = face_cascade.detectMultiScale(
    gray_image,
    scaleFactor=1.4,    # Image pyramid scale (1.1-1.5)
    minNeighbors=4,     # Detection confidence (3-6)
    minSize=(30, 30)    # Minimum face size
)
```

**Parameter Guide:**

| Parameter | Range | Effect |
|-----------|-------|--------|
| scaleFactor | 1.1-1.5 | Lower = more detections, slower |
| minNeighbors | 3-6 | Higher = fewer false positives |
| minSize | (w, h) | Ignore faces smaller than this |

## Person Following

### Using Built-in Person Following

The drone can autonomously track and follow a person:

```python
from colibricf.task import Task
import rospy

class FollowPerson(Task):
    """Follow a person autonomously"""
    
    TAKEOFF_ALTITUDE = 2.0
    
    def mission(self):
        # Arm and takeoff
        self.drone.arm()
        rospy.sleep(3)
        self.drone.navigate_wait(z=self.TAKEOFF_ALTITUDE, auto_arm=True)
        
        # Start following
        print("Starting person following mode...")
        self.drone.follow()

task = FollowPerson()
task.run()
```

**How It Works:**
1. Uses MediaPipe Pose Landmarker for pose estimation
2. Detects person's body landmarks
3. Centers person in camera view (yaw rotation)
4. Maintains distance by moving forward/backward
5. Adjusts altitude to keep person centered

### Person Following Configuration

Internal configuration (from source):

```python
# Pose detection settings
detection_confidence = 0.65
num_poses = 1  # Track single person

# Centering thresholds
center_tolerance = 100  # pixels

# Distance control
target_size_min = 250
target_size_max = 340
```

### Manual Following Control

Create custom following behavior:

```python
from colibricf.drone import Drone
from colibricf.camera import Camera
import cv2
import mediapipe as mp
import rospy

drone = Drone()
camera = Camera()

# Initialize MediaPipe
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(
    min_detection_confidence=0.7,
    min_tracking_confidence=0.7
)

# Arm and takeoff
drone.arm()
rospy.sleep(3)
drone.navigate_wait(z=2.0, auto_arm=True)

while not rospy.is_shutdown():
    frame = camera.retrieve_cv_frame()
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    
    # Detect pose
    results = pose.process(rgb_frame)
    
    if results.pose_landmarks:
        # Get person position
        landmarks = results.pose_landmarks.landmark
        h, w, _ = frame.shape
        
        # Get nose position (landmark 0)
        nose = landmarks[0]
        nose_x = int(nose.x * w)
        nose_y = int(nose.y * h)
        
        # Calculate center offset
        center_x = w // 2
        offset_x = nose_x - center_x
        
        # Adjust yaw to center person
        if abs(offset_x) > 50:
            yaw_adjustment = offset_x * 0.001
            drone.set_yaw(yaw_adjustment, frame_id='body')
        
        # Draw landmarks
        mp.solutions.drawing_utils.draw_landmarks(
            frame,
            results.pose_landmarks,
            mp_pose.POSE_CONNECTIONS
        )
    
    cv2.imshow('Following', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

drone.land_wait()
cv2.destroyAllWindows()
```

## QR Code Detection

### Basic QR Code Scanning

```python
from colibricf.camera import Camera
import rospy

rospy.init_node('qr_scanner')
camera = Camera()

# Start continuous QR code detection
camera.read_qrcode()
```

**Output Example:**
```
Found QR-CODE with data https://example.com with center at x=320, y=240
Found CODE128 with data 123456789 with center at x=450, y=180
```

### Custom QR Code Handler

```python
import rospy
import cv2
from pyzbar import pyzbar
from colibricf.camera import Camera

camera = Camera()

def process_qr_codes():
    while not rospy.is_shutdown():
        frame = camera.retrieve_cv_frame()
        
        # Decode barcodes
        barcodes = pyzbar.decode(frame)
        
        for barcode in barcodes:
            # Extract data
            data = barcode.data.decode('utf-8')
            barcode_type = barcode.type
            
            # Get position
            (x, y, w, h) = barcode.rect
            center_x = x + w // 2
            center_y = y + h // 2
            
            # Draw box
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            
            # Display data
            text = f"{barcode_type}: {data}"
            cv2.putText(frame, text, (x, y-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            print(f"Detected {barcode_type}: {data} at ({center_x}, {center_y})")
            
            # Custom action based on QR code
            if data == "LAND":
                print("Landing command received!")
                # drone.land_wait()
            elif data.startswith("GOTO:"):
                coords = data.split(":")[1]
                print(f"Navigation command: {coords}")
        
        cv2.imshow('QR Scanner', frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

rospy.init_node('qr_processor')
process_qr_codes()
```

### QR Code-Based Navigation

```python
from colibricf.task import Task
from pyzbar import pyzbar
import cv2
import rospy

class QRNavigationTask(Task):
    """Navigate based on QR code waypoints"""
    
    def mission(self):
        # Arm and takeoff
        self.drone.arm()
        rospy.sleep(3)
        self.drone.navigate_wait(z=2.0, auto_arm=True)
        
        # Search for QR codes
        waypoints_found = []
        
        for i in range(10):  # Search for 10 seconds
            frame = self.camera.retrieve_cv_frame()
            barcodes = pyzbar.decode(frame)
            
            for barcode in barcodes:
                data = barcode.data.decode('utf-8')
                
                # Parse waypoint from QR code
                # Expected format: "WP:x,y,z"
                if data.startswith("WP:"):
                    coords = data.split(":")[1].split(",")
                    x, y, z = map(float, coords)
                    waypoints_found.append((x, y, z))
                    print(f"Waypoint found: ({x}, {y}, {z})")
            
            rospy.sleep(1)
        
        # Execute waypoints
        for x, y, z in waypoints_found:
            print(f"Flying to ({x}, {y}, {z})")
            self.drone.navigate_wait(x=x, y=y, z=z)
            rospy.sleep(2)

task = QRNavigationTask()
task.run()
```

## Custom CV Pipelines

### Color-Based Object Tracking

```python
import cv2
import numpy as np
from colibricf.camera import Camera
from colibricf.drone import Drone
import rospy

camera = Camera()
drone = Drone()

# Arm and position drone
drone.arm()
rospy.sleep(3)
drone.navigate_wait(z=2.0, auto_arm=True)

# Define color range (orange ball example)
lower_orange = np.array([5, 150, 150])
upper_orange = np.array([15, 255, 255])

while not rospy.is_shutdown():
    frame = camera.retrieve_cv_frame()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Create mask
    mask = cv2.inRange(hsv, lower_orange, upper_orange)
    
    # Clean up mask
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        # Get largest contour
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        
        if area > 500:  # Minimum area threshold
            # Get center
            M = cv2.moments(largest)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                # Draw on frame
                cv2.circle(frame, (cx, cy), 10, (0, 255, 0), -1)
                cv2.putText(frame, f"Target: ({cx}, {cy})", 
                           (cx-50, cy-20), cv2.FONT_HERSHEY_SIMPLEX, 
                           0.5, (0, 255, 0), 2)
                
                # Calculate offset from center
                h, w = frame.shape[:2]
                offset_x = cx - w // 2
                offset_y = cy - h // 2
                
                # Adjust drone position
                if abs(offset_x) > 30:
                    drone.set_position(y=offset_x * 0.001)
                if abs(offset_y) > 30:
                    drone.set_position(x=-offset_y * 0.001)
    
    cv2.imshow('Tracking', frame)
    cv2.imshow('Mask', mask)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

drone.land_wait()
cv2.destroyAllWindows()
```

### Line Following

```python
import cv2
import numpy as np
from colibricf.camera import Camera
from colibricf.drone import Drone
import rospy

camera = Camera()
drone = Drone()

drone.arm()
rospy.sleep(3)
drone.navigate_wait(z=1.5, auto_arm=True)

while not rospy.is_shutdown():
    frame = camera.retrieve_cv_frame()
    
    # Get region of interest (bottom half)
    h, w = frame.shape[:2]
    roi = frame[h//2:, :]
    
    # Convert to grayscale and threshold
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)
    
    # Find contours
    contours, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        # Get largest contour (the line)
        largest = max(contours, key=cv2.contourArea)
        
        # Get center of line
        M = cv2.moments(largest)
        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            
            # Calculate offset from center
            center = w // 2
            error = cx - center
            
            # Adjust yaw to follow line
            if abs(error) > 20:
                yaw_adjustment = error * 0.002
                drone.set_yaw(yaw_adjustment, frame_id='body')
            
            # Move forward
            drone.set_velocity(vx=0.3, vy=0, vz=0, frame_id='body')
    
    cv2.imshow('Line Following', binary)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

drone.land_wait()
cv2.destroyAllWindows()
```

## Integration with Flight

### Vision-Based Landing

```python
from colibricf.task import Task
import cv2
import numpy as np
import rospy

class VisionLanding(Task):
    """Land on visual marker"""
    
    def mission(self):
        # Takeoff
        self.drone.arm()
        rospy.sleep(3)
        self.drone.navigate_wait(z=5.0, auto_arm=True)
        
        # Search for landing pad (orange circle)
        self.search_and_land()
    
    def search_and_land(self):
        lower_orange = np.array([5, 150, 150])
        upper_orange = np.array([15, 255, 255])
        
        while True:
            frame = self.camera.retrieve_cv_frame()
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lower_orange, upper_orange)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                largest = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest)
                
                if area > 1000:
                    M = cv2.moments(largest)
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    h, w = frame.shape[:2]
                    offset_x = (cx - w//2) * 0.001
                    offset_y = (cy - h//2) * 0.001
                    
                    # Center over pad
                    if abs(offset_x) > 0.1 or abs(offset_y) > 0.1:
                        self.drone.set_position(y=offset_x, x=-offset_y)
                        rospy.sleep(0.5)
                    else:
                        # Centered, descend
                        telem = self.drone.get_telemetry()
                        if telem.z > 0.5:
                            self.drone.set_position(z=telem.z - 0.2)
                        else:
                            break
            
            rospy.sleep(0.1)

task = VisionLanding()
task.run()
```

## Performance Optimization

### Frame Rate Control

```python
import rospy
from colibricf.camera import Camera

camera = Camera()

# Process at 10 Hz instead of maximum
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    frame = camera.retrieve_cv_frame()
    
    # Your processing here
    
    rate.sleep()
```

### Image Downsampling

```python
import cv2

# Resize for faster processing
small = cv2.resize(frame, (320, 240))

# Process small frame
processed = process_image(small)

# Upscale results if needed
result = cv2.resize(processed, (640, 480))
```

### ROI Processing

```python
# Only process center region
h, w = frame.shape[:2]
roi = frame[h//4:3*h//4, w//4:3*w//4]

# Process only ROI
processed_roi = process_image(roi)
```

## Complete Applications

### Autonomous Search and Rescue

```python
from colibricf.task import Task
import cv2
import rospy

class SearchRescue(Task):
    """Search for people in distress"""
    
    def mission(self):
        from colibricf.cv.face_detect import detectFace
        
        # Search grid
        self.drone.arm()
        rospy.sleep(3)
        self.drone.navigate_wait(z=10.0, auto_arm=True)
        
        found_people = []
        
        for x in range(0, 50, 10):
            for y in range(0, 50, 10):
                print(f"Searching ({x}, {y})...")
                self.drone.navigate_wait(x=x, y=y, z=10.0)
                rospy.sleep(2)
                
                # Look for faces
                frame = self.camera.retrieve_cv_frame()
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                faces = detectFace(gray)
                
                if len(faces) > 0:
                    print(f"Found {len(faces)} person(s) at ({x}, {y})!")
                    found_people.append((x, y, len(faces)))
                    
                    # Save evidence
                    cv2.imwrite(f'/home/pi/rescue/found_{x}_{y}.jpg', frame)
        
        print(f"\nSearch complete. Found people at {len(found_people)} locations.")
        
        # Return to start
        self.drone.navigate_wait(x=0, y=0, z=10.0)

task = SearchRescue()
task.run()
```

---

## See Also

- [Camera Module](Camera-Module.md) - Basic camera operations
- [Drone Module](Drone-Module.md) - Flight control integration
- [Examples and Tutorials](Examples-and-Tutorials.md) - More CV examples

[← Back to Home](Home.md) | [Next: Examples →](Examples-and-Tutorials.md)
