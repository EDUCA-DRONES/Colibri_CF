# Camera Module

The **Camera** module provides powerful image capture and computer vision capabilities for Colibri drones. It integrates seamlessly with OpenCV for real-time image processing and analysis.

## Table of Contents

- [Overview](#overview)
- [Initialization](#initialization)
- [Basic Image Capture](#basic-image-capture)
- [Image Saving with GPS](#image-saving-with-gps)
- [QR Code Detection](#qr-code-detection)
- [Face Detection](#face-detection)
- [Computer Vision Integration](#computer-vision-integration)
- [Advanced Processing](#advanced-processing)
- [Performance Optimization](#performance-optimization)
- [Complete Examples](#complete-examples)

## Overview

The Camera module enables:
- **Frame Capture**: Get images from the drone's camera
- **QR Code Detection**: Automatic barcode/QR code scanning
- **Face Detection**: Real-time face recognition
- **Image Processing**: Full OpenCV integration
- **GPS Tagging**: Save images with location data
- **ROS Integration**: Subscribe to camera topics

**Supported Features:**
- ✅ Real-time frame capture
- ✅ QR/Barcode detection
- ✅ Face detection and tracking
- ✅ GPS EXIF metadata
- ✅ Custom image processing pipelines
- ✅ ROS topic subscriptions

## Initialization

### Basic Initialization

```python
from colibricf.camera import Camera

# Initialize camera
camera = Camera()
```

**Note:** The Camera class uses ROS's CvBridge for image format conversion between ROS messages and OpenCV format.

### With ROS Node

If you're using Camera in a standalone script:

```python
import rospy
from colibricf.camera import Camera

# Initialize ROS node first
rospy.init_node('camera_node')

# Then initialize camera
camera = Camera()
```

## Basic Image Capture

### retrieve_cv_frame()

Capture a single frame from the drone's camera.

```python
import cv2
from colibricf.camera import Camera

camera = Camera()

# Capture frame
frame = camera.retrieve_cv_frame()

# Display the frame
cv2.imshow('Drone Camera', frame)
cv2.waitKey(0)
cv2.destroyAllWindows()
```

**Returns:** OpenCV image (numpy array) in BGR format

**Frame Properties:**

```python
frame = camera.retrieve_cv_frame()

# Get dimensions
height, width, channels = frame.shape
print(f"Resolution: {width}x{height}")
print(f"Channels: {channels}")  # Typically 3 (BGR)

# Data type
print(f"Data type: {frame.dtype}")  # uint8
```

### Continuous Frame Capture

```python
import cv2
import rospy
from colibricf.camera import Camera

camera = Camera()

# Capture loop at 10 Hz
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    frame = camera.retrieve_cv_frame()
    
    cv2.imshow('Live Feed', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
    rate.sleep()

cv2.destroyAllWindows()
```

## Image Saving with GPS

### save_image()

Save image with embedded GPS EXIF metadata.

```python
from colibricf.camera import Camera

camera = Camera()

# Save image with GPS data
camera.save_image(path='/home/pi/photos')
```

**Parameters:**

- **`path`** (str): Directory path to save images

**Features:**
- Automatically adds timestamp to filename
- Embeds GPS coordinates in EXIF data
- Includes altitude information
- Format: `YYYY-MM-DD_HH-MM-SS.jpg`

**Example Output:**

```
Filename: 2025-01-15_14-30-45.jpg
EXIF GPS: 47.3977°N, 8.5455°E, Alt: 450m
```

### Complete Photo Mission

```python
from colibricf.drone import Drone
from colibricf.camera import Camera
import rospy

drone = Drone()
camera = Camera()

# Takeoff
drone.arm()
rospy.sleep(3)
drone.navigate_wait(z=5.0, auto_arm=True)

# Photo waypoints
waypoints = [(0, 0), (5, 0), (5, 5), (0, 5)]

for i, (x, y) in enumerate(waypoints):
    print(f"Moving to photo point {i+1}")
    drone.navigate_wait(x=x, y=y, z=5.0)
    rospy.sleep(1)
    
    # Capture photo with GPS
    camera.save_image('/home/pi/mission_photos')
    print(f"Photo {i+1} saved!")

# Return and land
drone.navigate_wait(x=0, y=0, z=5.0)
drone.land_wait()
```

## QR Code Detection

### read_qrcode()

Start continuous QR code detection.

```python
from colibricf.camera import Camera
import rospy

rospy.init_node('qr_scanner')
camera = Camera()

# Start QR code detection
camera.read_qrcode()
```

**Output Format:**

```
Found QR-CODE with data https://example.com with center at x=320, y=240
Found CODE128 with data 123456789 with center at x=450, y=180
```

**Detected Information:**
- **Barcode Type**: QR-CODE, CODE128, EAN13, etc.
- **Data**: Decoded content
- **Position**: Center coordinates (x, y) in pixels

### QR Code Detection with Action

```python
import rospy
from colibricf.camera import Camera
from colibricf.cv.qrcode import _qrcode_callback
from sensor_msgs.msg import Image

rospy.init_node('qr_action')

# Custom callback for QR codes
detected_codes = []

def qr_callback(msg):
    # Process QR code detection
    # Add your custom logic here
    pass

camera = Camera()

# Subscribe with custom callback
rospy.Subscriber('main_camera/image_raw_throttled', Image, qr_callback, queue_size=1)

rospy.spin()
```

### Supported Barcode Types

- QR Code
- Code 128
- Code 39
- EAN-8
- EAN-13
- UPC-A
- UPC-E
- DataMatrix
- PDF417

## Face Detection

### draw_face()

Detect and display faces in real-time.

```python
from colibricf.camera import Camera
import rospy

rospy.init_node('face_detector')
camera = Camera()

# Start face detection
camera.draw_face()
```

**Features:**
- Haar Cascade-based detection
- Real-time face tracking
- Bounding box overlay
- Position coordinates display
- Published to ROS topic: `~face_detect/debug`

### Face Detection Parameters

The face detection uses:
- **Scale Factor**: 1.4
- **Min Neighbors**: 4
- **Method**: Haar Cascade Classifier

### Viewing Face Detection

```bash
# In terminal 1: Run face detection
rosrun colibricf face_detect.py

# In terminal 2: View results
rqt_image_view
# Select topic: /face_detect/debug
```

## Computer Vision Integration

### Grayscale Conversion

```python
import cv2
from colibricf.camera import Camera

camera = Camera()
frame = camera.retrieve_cv_frame()

# Convert to grayscale
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

cv2.imshow('Grayscale', gray)
cv2.waitKey(0)
```

### Edge Detection

```python
import cv2
from colibricf.camera import Camera

camera = Camera()
frame = camera.retrieve_cv_frame()

# Convert to grayscale
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

# Canny edge detection
edges = cv2.Canny(gray, 50, 150)

cv2.imshow('Edges', edges)
cv2.waitKey(0)
```

### Color Detection

```python
import cv2
import numpy as np
from colibricf.camera import Camera

camera = Camera()
frame = camera.retrieve_cv_frame()

# Convert to HSV
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

# Define color range (red example)
lower_red = np.array([0, 100, 100])
upper_red = np.array([10, 255, 255])

# Create mask
mask = cv2.inRange(hsv, lower_red, upper_red)

# Apply mask
result = cv2.bitwise_and(frame, frame, mask=mask)

cv2.imshow('Red Objects', result)
cv2.waitKey(0)
```

### Contour Detection

```python
import cv2
from colibricf.camera import Camera

camera = Camera()
frame = camera.retrieve_cv_frame()

# Convert and threshold
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
_, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

# Find contours
contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

# Draw contours
cv2.drawContours(frame, contours, -1, (0, 255, 0), 2)

cv2.imshow('Contours', frame)
cv2.waitKey(0)
```

### Object Tracking

```python
import cv2
from colibricf.camera import Camera
from colibricf.drone import Drone
import numpy as np

camera = Camera()
drone = Drone()

# Define color to track (orange ball example)
lower_orange = np.array([5, 150, 150])
upper_orange = np.array([15, 255, 255])

while True:
    frame = camera.retrieve_cv_frame()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Create mask
    mask = cv2.inRange(hsv, lower_orange, upper_orange)
    
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        # Get largest contour
        largest = max(contours, key=cv2.contourArea)
        
        # Get center
        M = cv2.moments(largest)
        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            
            # Draw center
            cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
            
            # Calculate offset from center
            height, width = frame.shape[:2]
            offset_x = cx - width // 2
            offset_y = cy - height // 2
            
            print(f"Object at: ({cx}, {cy}), Offset: ({offset_x}, {offset_y})")
            
            # Optionally adjust drone position based on offset
            # drone.set_position(y=offset_x * 0.001, x=-offset_y * 0.001)
    
    cv2.imshow('Tracking', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
```

## Advanced Processing

### Image Filtering

```python
import cv2
from colibricf.camera import Camera

camera = Camera()
frame = camera.retrieve_cv_frame()

# Gaussian blur
blurred = cv2.GaussianBlur(frame, (5, 5), 0)

# Bilateral filter (edge-preserving)
bilateral = cv2.bilateralFilter(frame, 9, 75, 75)

# Median filter
median = cv2.medianBlur(frame, 5)

cv2.imshow('Original', frame)
cv2.imshow('Blurred', blurred)
cv2.imshow('Bilateral', bilateral)
cv2.waitKey(0)
```

### Image Enhancement

```python
import cv2
import numpy as np
from colibricf.camera import Camera

camera = Camera()
frame = camera.retrieve_cv_frame()

# Increase brightness
brightened = cv2.convertScaleAbs(frame, alpha=1.2, beta=30)

# Increase contrast
contrasted = cv2.convertScaleAbs(frame, alpha=1.5, beta=0)

# Histogram equalization (grayscale)
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
equalized = cv2.equalizeHist(gray)

cv2.imshow('Brightened', brightened)
cv2.imshow('Contrasted', contrasted)
cv2.waitKey(0)
```

### Feature Detection

```python
import cv2
from colibricf.camera import Camera

camera = Camera()
frame = camera.retrieve_cv_frame()

gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

# Detect corners (Harris)
corners = cv2.cornerHarris(gray, 2, 3, 0.04)
frame[corners > 0.01 * corners.max()] = [0, 0, 255]

# Or use ORB features
orb = cv2.ORB_create()
keypoints, descriptors = orb.detectAndCompute(gray, None)
frame_with_kp = cv2.drawKeypoints(frame, keypoints, None, color=(0, 255, 0))

cv2.imshow('Features', frame_with_kp)
cv2.waitKey(0)
```

## Performance Optimization

### Frame Rate Control

```python
import rospy
from colibricf.camera import Camera

camera = Camera()

# Process at 5 Hz instead of maximum
rate = rospy.Rate(5)

while not rospy.is_shutdown():
    frame = camera.retrieve_cv_frame()
    # Process frame...
    rate.sleep()
```

### Image Resizing

```python
import cv2
from colibricf.camera import Camera

camera = Camera()
frame = camera.retrieve_cv_frame()

# Resize for faster processing
small_frame = cv2.resize(frame, (320, 240))

# Process small frame
# ...

# Resize results back if needed
result = cv2.resize(processed, (640, 480))
```

### Region of Interest (ROI)

```python
import cv2
from colibricf.camera import Camera

camera = Camera()
frame = camera.retrieve_cv_frame()

# Only process center region
height, width = frame.shape[:2]
roi = frame[
    height//4:3*height//4,
    width//4:3*width//4
]

# Process only ROI
# ...
```

## Complete Examples

### Aerial Photography Mission

```python
from colibricf.drone import Drone
from colibricf.camera import Camera
import rospy
import cv2

drone = Drone()
camera = Camera()

# Survey grid
altitude = 10.0
spacing = 5.0
grid_size = 3

drone.arm()
rospy.sleep(3)
drone.navigate_wait(z=altitude, auto_arm=True)

for i in range(grid_size):
    for j in range(grid_size):
        x = i * spacing
        y = j * spacing
        
        print(f"Grid point ({i}, {j})")
        drone.navigate_wait(x=x, y=y, z=altitude)
        rospy.sleep(1)
        
        # Capture photo
        frame = camera.retrieve_cv_frame()
        cv2.imwrite(f'/home/pi/survey/photo_{i}_{j}.jpg', frame)

drone.navigate_wait(x=0, y=0, z=altitude)
drone.land_wait()
```

### QR Code Search Mission

```python
from colibricf.drone import Drone
from colibricf.camera import Camera
import rospy

rospy.init_node('qr_search')

drone = Drone()
camera = Camera()

# Start QR detection in background
import threading
qr_thread = threading.Thread(target=camera.read_qrcode)
qr_thread.daemon = True
qr_thread.start()

# Search pattern
drone.arm()
rospy.sleep(3)
drone.navigate_wait(z=1.5, auto_arm=True)

search_points = [
    (0, 0), (2, 0), (4, 0),
    (4, 2), (2, 2), (0, 2),
    (0, 4), (2, 4), (4, 4),
]

for x, y in search_points:
    drone.navigate_wait(x=x, y=y, z=1.5)
    rospy.sleep(3)  # Allow time for QR detection

drone.land_wait()
```

### Vision-Based Landing

```python
import cv2
import numpy as np
from colibricf.drone import Drone
from colibricf.camera import Camera
import rospy

drone = Drone()
camera = Camera()

# Takeoff
drone.arm()
rospy.sleep(3)
drone.navigate_wait(z=3.0, auto_arm=True)

# Look for landing marker (orange circle)
while True:
    frame = camera.retrieve_cv_frame()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Detect orange
    lower = np.array([5, 150, 150])
    upper = np.array([15, 255, 255])
    mask = cv2.inRange(hsv, lower, upper)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        
        if area > 1000:  # Large enough marker
            M = cv2.moments(largest)
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            
            height, width = frame.shape[:2]
            center_x = width // 2
            center_y = height // 2
            
            # Calculate offset
            dx = (cx - center_x) * 0.001
            dy = (cy - center_y) * 0.001
            
            # Adjust position
            if abs(dx) > 0.1 or abs(dy) > 0.1:
                drone.set_position(y=dx, x=-dy)
                rospy.sleep(0.5)
            else:
                # Centered, descend
                telem = drone.get_telemetry()
                if telem.z > 0.3:
                    drone.set_position(z=telem.z - 0.1)
                else:
                    break
    
    rospy.sleep(0.1)

# Final landing
drone.land_wait()
```

---

## See Also

- [Drone Module](Drone-Module.md) - Flight control integration
- [Computer Vision Guide](Computer-Vision.md) - Advanced CV techniques
- [Task Framework](Task-Framework.md) - Mission integration
- [Examples and Tutorials](Examples-and-Tutorials.md) - More examples

[← Back to Home](Home.md) | [Next: Servo Module →](Servo-Module.md)
