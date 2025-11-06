# Examples and Tutorials

Practical examples and step-by-step tutorials for common Colibri drone applications.

## Table of Contents

- [Basic Flight Missions](#basic-flight-missions)
- [Photography and Surveying](#photography-and-surveying)
- [Delivery Systems](#delivery-systems)
- [Inspection and Monitoring](#inspection-and-monitoring)
- [Search and Rescue](#search-and-rescue)
- [Advanced Missions](#advanced-missions)

## Basic Flight Missions

### Simple Takeoff and Land

```python
#!/usr/bin/env python3
from colibricf.drone import Drone
import rospy

drone = Drone()

# Arm motors
drone.arm()
rospy.sleep(3)

# Takeoff to 1.5 meters
drone.navigate_wait(z=1.5, auto_arm=True)

# Hover for 5 seconds
rospy.sleep(5)

# Land
drone.land_wait()
```

### Square Flight Pattern

```python
from colibricf.drone import Drone, Waypoint
import rospy

drone = Drone()

# Define square waypoints (3x3 meters)
waypoints = [
    Waypoint(0, 0, 1.5),
    Waypoint(3, 0, 1.5),
    Waypoint(3, 3, 1.5),
    Waypoint(0, 3, 1.5),
    Waypoint(0, 0, 1.5),
]

# Execute mission
drone.arm()
rospy.sleep(3)
drone.navigate_wait(z=1.5, auto_arm=True)

drone.waypoint_navigate(waypoints)

drone.land_wait()
```

### Figure-8 Pattern

```python
from colibricf.drone import Drone
import rospy
import math

drone = Drone()

drone.arm()
rospy.sleep(3)
drone.navigate_wait(z=2.0, auto_arm=True)

# Figure-8 pattern
radius = 2.0
for t in range(0, 360, 10):
    rad = math.radians(t)
    x = radius * math.sin(rad)
    y = radius * math.sin(2 * rad) / 2
    drone.navigate_wait(x=x, y=y, z=2.0, speed=0.8)

drone.navigate_wait(x=0, y=0, z=2.0)
drone.land_wait()
```

## Photography and Surveying

### Aerial Photography Grid

```python
from colibricf.task import Task
import cv2
import rospy

class SurveyGrid(Task):
    """Capture photos in grid pattern"""
    
    def __init__(self, rows=4, cols=4, spacing=5.0, altitude=10.0):
        super().__init__()
        self.rows = rows
        self.cols = cols
        self.spacing = spacing
        self.altitude = altitude
    
    def mission(self):
        print(f"Starting {self.rows}x{self.cols} survey grid")
        
        # Takeoff
        self.drone.arm()
        rospy.sleep(3)
        self.drone.navigate_wait(z=self.altitude, auto_arm=True)
        
        photo_count = 0
        
        for i in range(self.rows):
            for j in range(self.cols):
                x = i * self.spacing
                y = j * self.spacing
                
                print(f"Grid point ({i}, {j})")
                self.drone.navigate_wait(x=x, y=y, z=self.altitude)
                rospy.sleep(1)
                
                # Capture photo
                frame = self.camera.retrieve_cv_frame()
                filename = f'/home/pi/survey/grid_{i}_{j}.jpg'
                cv2.imwrite(filename, frame)
                photo_count += 1
        
        print(f"Survey complete! Captured {photo_count} photos")
        
        # Return home
        self.drone.navigate_wait(x=0, y=0, z=self.altitude)

# Execute 5x5 grid at 15m altitude
task = SurveyGrid(rows=5, cols=5, spacing=10.0, altitude=15.0)
task.run()
```

### Panoramic Photography

```python
from colibricf.task import Task
import cv2
import rospy
import math

class Panorama(Task):
    """Capture 360° panoramic photos"""
    
    def __init__(self, altitude=5.0, num_photos=8):
        super().__init__(gpio=14)  # Gimbal servo
        self.altitude = altitude
        self.num_photos = num_photos
    
    def mission(self):
        print("Starting panoramic capture")
        
        # Takeoff
        self.drone.arm()
        rospy.sleep(3)
        self.drone.navigate_wait(z=self.altitude, auto_arm=True)
        
        # Center gimbal
        self.servo.pwm_neutral()
        rospy.sleep(1)
        
        # Capture photos in circle
        angle_step = 2 * math.pi / self.num_photos
        
        for i in range(self.num_photos):
            angle = i * angle_step
            yaw = angle
            
            print(f"Photo {i+1}/{self.num_photos} at {math.degrees(angle):.1f}°")
            
            # Rotate to angle
            self.drone.set_yaw(yaw=yaw)
            rospy.sleep(2)
            
            # Capture
            frame = self.camera.retrieve_cv_frame()
            cv2.imwrite(f'/home/pi/panorama/pano_{i:02d}.jpg', frame)
        
        print("Panorama complete!")

task = Panorama(altitude=8.0, num_photos=12)
task.run()
```

## Delivery Systems

### Package Delivery

```python
from colibricf.task import Task
import rospy

class PackageDelivery(Task):
    """Autonomous package delivery mission"""
    
    def __init__(self, delivery_point, package_weight=0.5):
        super().__init__(gpio=17)  # Release mechanism servo
        self.delivery_point = delivery_point
        self.package_weight = package_weight
        self.FLIGHT_ALT = 15.0
        self.DROP_ALT = 3.0
    
    def mission(self):
        print("=== PACKAGE DELIVERY MISSION ===")
        
        # Preflight check
        self.preflight_check()
        
        # Secure package
        print("[1/6] Securing package...")
        self.servo.pwm_low(sleep=1.0)
        
        # Takeoff
        print("[2/6] Taking off...")
        self.drone.arm()
        rospy.sleep(3)
        self.drone.navigate_wait(z=self.FLIGHT_ALT, auto_arm=True)
        
        # Navigate to delivery point
        print(f"[3/6] Flying to delivery point...")
        self.drone.navigate_wait(
            x=self.delivery_point[0],
            y=self.delivery_point[1],
            z=self.FLIGHT_ALT
        )
        
        # Descend for delivery
        print("[4/6] Descending for delivery...")
        self.drone.navigate_wait(
            x=self.delivery_point[0],
            y=self.delivery_point[1],
            z=self.DROP_ALT
        )
        rospy.sleep(2)
        
        # Release package
        print("[5/6] Releasing package...")
        self.servo.pwm_high(sleep=2.0)
        print("Package delivered!")
        
        # Return to neutral
        self.servo.pwm_neutral()
        
        # Return home
        print("[6/6] Returning to launch...")
        self.drone.navigate_wait(x=0, y=0, z=self.FLIGHT_ALT)
        
        print("Mission complete!")
    
    def preflight_check(self):
        telem = self.drone.get_telemetry()
        if telem.battery_voltage < 11.0:
            raise Exception(f"Battery too low: {telem.battery_voltage:.1f}V")
        print(f"Preflight OK - Battery: {telem.battery_voltage:.1f}V")

# Execute delivery
delivery_location = (50, 30)  # 50m x, 30m y
task = PackageDelivery(delivery_location, package_weight=0.8)
task.run()
```

### Multi-Stop Delivery Route

```python
from colibricf.task import Task
import rospy

class DeliveryRoute(Task):
    """Multiple delivery stops"""
    
    def __init__(self, stops):
        super().__init__(gpio=17)
        self.stops = stops  # List of (x, y, package_id)
        self.FLIGHT_ALT = 20.0
    
    def mission(self):
        print(f"Delivery route with {len(self.stops)} stops")
        
        # Secure all packages
        self.servo.pwm_low(sleep=0.5)
        
        # Takeoff
        self.drone.arm()
        rospy.sleep(3)
        self.drone.navigate_wait(z=self.FLIGHT_ALT, auto_arm=True)
        
        for i, (x, y, pkg_id) in enumerate(self.stops):
            print(f"\n[Stop {i+1}/{len(self.stops)}] Package: {pkg_id}")
            
            # Navigate
            self.drone.navigate_wait(x=x, y=y, z=self.FLIGHT_ALT)
            
            # Descend
            self.drone.navigate_wait(x=x, y=y, z=3.0)
            rospy.sleep(1)
            
            # Deliver
            print(f"Delivering package {pkg_id}...")
            self.servo.pwm_high(sleep=1.5)
            self.servo.pwm_low(sleep=0.5)
            
            # Climb
            self.drone.navigate_wait(x=x, y=y, z=self.FLIGHT_ALT)
        
        # Return home
        self.drone.navigate_wait(x=0, y=0, z=self.FLIGHT_ALT)
        print("\nAll deliveries complete!")

# Define delivery route
stops = [
    (20, 10, "PKG-001"),
    (30, 25, "PKG-002"),
    (15, 35, "PKG-003"),
]

task = DeliveryRoute(stops)
task.run()
```

## Inspection and Monitoring

### Structure Inspection

```python
from colibricf.task import Task
import cv2
import rospy
import math

class StructureInspection(Task):
    """Inspect structure by orbiting around it"""
    
    def __init__(self, structure_location, radius=5.0, altitude=10.0):
        super().__init__(gpio=14)  # Camera gimbal
        self.structure = structure_location
        self.radius = radius
        self.altitude = altitude
    
    def mission(self):
        print("Starting structure inspection")
        
        # Navigate to structure
        self.drone.arm()
        rospy.sleep(3)
        self.drone.navigate_wait(
            x=self.structure[0],
            y=self.structure[1],
            z=self.altitude,
            auto_arm=True
        )
        
        # Point camera at structure
        self.servo.pwm_low(sleep=1.0)
        
        # Orbit inspection - capture every 30°
        num_photos = 12
        angle_step = 2 * math.pi / num_photos
        
        for i in range(num_photos):
            angle = i * angle_step
            
            # Calculate position on circle
            x = self.structure[0] + self.radius * math.cos(angle)
            y = self.structure[1] + self.radius * math.sin(angle)
            
            # Move to position
            self.drone.navigate_wait(x=x, y=y, z=self.altitude)
            
            # Point at structure center
            yaw_to_center = math.atan2(
                self.structure[1] - y,
                self.structure[0] - x
            )
            self.drone.set_yaw(yaw=yaw_to_center)
            rospy.sleep(1)
            
            # Capture photo
            frame = self.camera.retrieve_cv_frame()
            cv2.imwrite(f'/home/pi/inspection/angle_{i:02d}.jpg', frame)
            print(f"Photo {i+1}/{num_photos} at {math.degrees(angle):.1f}°")
        
        # Return gimbal to neutral
        self.servo.pwm_neutral()
        
        print("Inspection complete!")

# Inspect structure at (25, 25)
task = StructureInspection(
    structure_location=(25, 25),
    radius=8.0,
    altitude=12.0
)
task.run()
```

### Pipeline/Power Line Patrol

```python
from colibricf.task import Task
import cv2
import rospy

class PipelinePatrol(Task):
    """Follow and inspect pipeline or power line"""
    
    def __init__(self, waypoints, altitude=15.0):
        super().__init__()
        self.waypoints = waypoints
        self.altitude = altitude
        self.inspection_interval = 10.0  # meters
    
    def mission(self):
        print("Starting pipeline patrol")
        
        self.drone.arm()
        rospy.sleep(3)
        self.drone.navigate_wait(z=self.altitude, auto_arm=True)
        
        for i in range(len(self.waypoints) - 1):
            start = self.waypoints[i]
            end = self.waypoints[i + 1]
            
            print(f"Segment {i+1}: ({start}) to ({end})")
            
            # Fly segment
            self.drone.navigate_wait(x=end[0], y=end[1], z=self.altitude)
            
            # Capture inspection photo
            frame = self.camera.retrieve_cv_frame()
            cv2.imwrite(f'/home/pi/patrol/segment_{i:02d}.jpg', frame)
            
            rospy.sleep(2)
        
        # Return to start
        self.drone.navigate_wait(
            x=self.waypoints[0][0],
            y=self.waypoints[0][1],
            z=self.altitude
        )
        
        print("Patrol complete!")

# Pipeline waypoints
pipeline = [
    (0, 0),
    (50, 0),
    (100, 20),
    (150, 20),
    (200, 40),
]

task = PipelinePatrol(pipeline, altitude=20.0)
task.run()
```

## Search and Rescue

### Area Search Pattern

```python
from colibricf.task import Task
import cv2
import rospy

class AreaSearch(Task):
    """Search area using lawnmower pattern"""
    
    def __init__(self, area_size=50, lane_spacing=10.0, altitude=15.0):
        super().__init__()
        self.area_size = area_size
        self.lane_spacing = lane_spacing
        self.altitude = altitude
    
    def mission(self):
        from colibricf.cv.face_detect import detectFace
        
        print("Starting area search mission")
        
        self.drone.arm()
        rospy.sleep(3)
        self.drone.navigate_wait(z=self.altitude, auto_arm=True)
        
        detections = []
        num_lanes = int(self.area_size / self.lane_spacing)
        
        for lane in range(num_lanes):
            y = lane * self.lane_spacing
            
            # Fly lane (alternate direction for efficiency)
            if lane % 2 == 0:
                # Left to right
                for x in range(0, self.area_size, 5):
                    self.search_point(x, y, detections)
            else:
                # Right to left
                for x in range(self.area_size, 0, -5):
                    self.search_point(x, y, detections)
        
        print(f"\nSearch complete! Found {len(detections)} potential targets")
        
        # Return to start
        self.drone.navigate_wait(x=0, y=0, z=self.altitude)
    
    def search_point(self, x, y, detections):
        from colibricf.cv.face_detect import detectFace
        
        self.drone.navigate_wait(x=x, y=y, z=self.altitude)
        
        # Look for people
        frame = self.camera.retrieve_cv_frame()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = detectFace(gray)
        
        if len(faces) > 0:
            print(f"FOUND: {len(faces)} person(s) at ({x}, {y})")
            detections.append((x, y, len(faces)))
            cv2.imwrite(f'/home/pi/search/found_{x}_{y}.jpg', frame)

task = AreaSearch(area_size=100, lane_spacing=15.0, altitude=20.0)
task.run()
```

## Advanced Missions

### Waypoint Racing Course

```python
from colibricf.drone import Drone, Waypoint
import rospy
import time

drone = Drone()

# Race course waypoints
course = [
    Waypoint(0, 0, 2),
    Waypoint(10, 0, 2),
    Waypoint(10, 10, 4),
    Waypoint(20, 10, 4),
    Waypoint(20, 0, 2),
    Waypoint(30, 0, 6),
    Waypoint(30, 10, 6),
    Waypoint(0, 10, 2),
    Waypoint(0, 0, 2),
]

print("=== RACE COURSE ===")
print(f"Waypoints: {len(course)}")

# Start
drone.arm()
rospy.sleep(3)

start_time = time.time()

# Race through course at max speed
drone.navigate_wait(z=2.0, auto_arm=True)

for i, wp in enumerate(course):
    print(f"Waypoint {i+1}/{len(course)}")
    drone.navigate_wait(x=wp.x, y=wp.y, z=wp.z, speed=2.0)

elapsed = time.time() - start_time

# Finish
drone.land_wait()

print(f"\nRace complete!")
print(f"Time: {elapsed:.1f} seconds")
```

### Autonomous Follow-Me

```python
from colibricf.task import Task
import rospy

class FollowMe(Task):
    """Follow person autonomously"""
    
    FOLLOW_ALTITUDE = 2.5
    FOLLOW_DISTANCE = 3.0  # meters behind person
    
    def mission(self):
        print("Starting Follow-Me mode")
        print("Stand in front of drone to start following")
        
        # Arm and takeoff
        self.drone.arm()
        rospy.sleep(3)
        self.drone.navigate_wait(z=self.FOLLOW_ALTITUDE, auto_arm=True)
        
        # Start person following
        print("Following enabled - move around!")
        self.drone.follow()

task = FollowMe()
task.run()
```

---

## See Also

- [Drone Module](Drone-Module.md) - Flight control reference
- [Camera Module](Camera-Module.md) - Image capture reference
- [Task Framework](Task-Framework.md) - Mission structure
- [Computer Vision](Computer-Vision.md) - CV capabilities

[← Back to Home](Home.md) | [Next: API Reference →](API-Reference.md)
