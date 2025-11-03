# Camera Module

The `Camera` class provides image capture and computer vision processing capabilities for Colibri drones. It integrates with OpenCV and the drone's main camera sensor through ROS topics.

## Overview

The Camera module enables real-time image processing directly on the drone, including QR code detection, frame capture, and vision-based navigation. It uses ROS image topics and OpenCV for efficient processing.

## Initialization

```python
from colibricf.camera import Camera

# Initialize the camera
camera = Camera()
```

The Camera class automatically initializes a ROS node for computer vision operations and sets up the CvBridge for image format conversion.

## Basic Image Capture

### retrieve_cv_frame()

Capture a single frame from the drone's main camera.

```python
import cv2
from colibricf.camera import Camera

camera = Camera()

# Get a single frame
frame = camera.retrieve_cv_frame()

# Display the frame
cv2.imshow('Drone Camera', frame)
cv2.waitKey(0)
cv2.destroyAllWindows()
```

**Returns:** OpenCV image matrix (numpy array) in BGR format

### Frame Properties

Once you have a frame, you can work with it using standard OpenCV operations:

```python
frame = camera.retrieve_cv_frame()

# Get frame dimensions
height, width = frame.shape[:2]
print(f"Frame resolution: {width}x{height}")

# Save the frame
cv2.imwrite('drone_capture.jpg', frame)

# Display frame info
print(f"Frame shape: {frame.shape}")
print(f"Data type: {frame.dtype}")
```

## QR Code Detection

### get_qrcode_sub()

Start a subscriber that continuously processes frames for QR codes.

```python
from colibricf.camera import Camera

camera = Camera()

# Start QR code detection
subscriber = camera.get_qrcode_sub()

# The callback will print detected QR codes automatically
# Detection output format:
# Found <barcode_type> with data <data> with center at x=<x>, y=<y>
```

**Returns:** ROS Subscriber object

### QR Code Detection Details

The QR code detector automatically processes incoming frames and prints information about detected codes:

```python
# Output example:
# Found QR-CODE with data https://example.com with center at x=320, y=240
```

**Information provided:**
- `barcode_type`: Type of barcode (QR-CODE, CODE128, etc.)
- `data`: Decoded data from the barcode
- `x`: X-coordinate of barcode center in pixels
- `y`: Y-coordinate of barcode center in pixels

### Complete QR Code Example

```python
from colibricf.camera import Camera
import rospy

camera = Camera()

# Start QR code detection
qrcode_sub = camera.get_qrcode_sub()

# Keep the node running to detect QR codes
try:
    rospy.spin()
except KeyboardInterrupt:
    print("QR code detection stopped")
```

## Image Processing Examples

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

### Face Detection

```python
import cv2
from colibricf.camera import Camera

camera = Camera()

# Load cascade classifier
face_cascade = cv2.CascadeClassifier(
    cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
)

frame = camera.retrieve_cv_frame()
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

# Detect faces
faces = face_cascade.detectMultiScale(gray, 1.1, 4)

# Draw rectangles around faces
for (x, y, w, h) in faces:
    cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)

cv2.imshow('Face Detection', frame)
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

# Define range for red color
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

# Convert to grayscale
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

# Threshold
_, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

# Find contours
contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

# Draw contours
frame_with_contours = frame.copy()
cv2.drawContours(frame_with_contours, contours, -1, (0, 255, 0), 2)

cv2.imshow('Contours', frame_with_contours)
cv2.waitKey(0)
```

## Real-time Processing Loop

```python
import cv2
import rospy
from colibricf.camera import Camera

camera = Camera()

# Process frames at regular intervals
rate = rospy.Rate(10)  # 10 Hz

while not rospy.is_shutdown():
    frame = camera.retrieve_cv_frame()
    
    # Your image processing here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    cv2.imshow('Live Feed', gray)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
    rate.sleep()

cv2.destroyAllWindows()
```

## Integration with Drone Control

### Vision-Based Navigation

```python
import cv2
from colibricf.camera import Camera
from colibricf.drone import Drone

camera = Camera()
drone = Drone()

# Get frame
frame = camera.retrieve_cv_frame()
height, width = frame.shape[:2]
center_x = width // 2
center_y = height // 2

# Find red object
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
lower_red = np.array([0, 100, 100])
upper_red = np.array([10, 255, 255])
mask = cv2.inRange(hsv, lower_red, upper_red)

# Find contours
contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

if contours:
    # Get largest contour
    largest_contour = max(contours, key=cv2.contourArea)
    M = cv2.moments(largest_contour)
    
    if M["m00"] > 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        
        # Calculate deviation from center
        dx = cx - center_x
        dy = cy - center_y
        
        # Adjust drone position
        if abs(dx) > 20:
            drone.set_position(y=dx * 0.001)
        if abs(dy) > 20:
            drone.set_position(x=-dy * 0.001)
```

## Camera Calibration Tips

- **Lighting**: Ensure adequate and consistent lighting for best results
- **Resolution**: The camera resolution affects processing speed and accuracy
- **Focus**: Adjust camera focus for different altitudes
- **Thermal Effects**: Consider thermal drift in calibration

## Performance Considerations

- Image processing is computationally intensive
- Use smaller image resolutions for real-time processing
- Consider frame skipping for high-frequency operations
- ROS subscription callbacks can be throttled to reduce CPU load

## Common Issues

### No Image Received

```python
# Check if camera topic is available
import rospy
from sensor_msgs.msg import Image

try:
    msg = rospy.wait_for_message('main_camera/image_raw', Image, timeout=5)
    print("Camera is working!")
except rospy.ROSException:
    print("Camera not found or not publishing")
```

### Slow Frame Capture

Use throttled topics for non-critical operations:
```python
# Instead of 'main_camera/image_raw', use:
# 'main_camera/image_raw_throttled' for QR code detection
```

## See Also

- [OpenCV Documentation](https://docs.opencv.org/)
- [PyZBar Documentation](https://github.com/NaturalHistoryMuseum/pyzbar)
- [ROS Image Pipeline](http://wiki.ros.org/image_pipeline)
- [Drone Module](drone.md)
