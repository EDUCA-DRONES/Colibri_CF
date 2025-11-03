# Task Framework

The `Task` class provides an abstract base for creating complex drone missions. It simplifies mission development by handling initialization, error management, and resource cleanup automatically.

## Overview

The Task framework allows developers to focus on mission logic by providing a structured interface for drone operations. It integrates all Colibri modules (Drone, Camera, Servo) and includes built-in error handling and recovery mechanisms.

## Architecture

The Task class follows the Abstract Base Class (ABC) pattern, requiring developers to implement the `mission()` method that contains the actual flight logic.

### Class Structure

```python
from colibricf.task import Task
from abc import abstractmethod

class Task(ABC):
    """Base class for all drone missions"""
    
    drone = Drone()          # Shared drone instance
    camera = Camera()        # Shared camera instance
    
    def __init__(self, gpio:int = -1):
        """Initialize with servo GPIO pin"""

        if gpio != -1:
          self.servo = Servo(gpio)
    
    @abstractmethod
    def mission(self):
        """Implementation required by subclasses"""
        raise Exception("Need implementation.")
    
    def run(self):
        """Execute mission with error handling"""
        # Implementation...
    
    def return_to_launch_confirm(self):
        """Confirm before returning to launch"""
        # Implementation...
    
    def change_servo_pin(self, gpio):
        """Change servo GPIO pin"""
        # Implementation...
```

## Creating a Task

### Basic Task Implementation

```python
from colibricf.task import Task
import rospy

class SimpleFlyTask(Task):
    """A simple task that takes off and lands"""
    
    def mission(self):
        print("Starting simple fly task")
        
        # Take off
        self.drone.arm()
        rospy.sleep(3)
        self.drone.navigate_wait(z=1.0, auto_arm=True)
        
        print("Hovering at 1 meter")
        rospy.sleep(3)
        
        print("Mission complete!")

# Execute the task
task = SimpleFlyTask()
task.run()
```

### Task with Servo Control

```python
from colibricf.task import Task
import rospy

class ServoTestTask(Task):
    """A task that controls servo motor"""
    
    def mission(self):
        print("Starting servo test task")
        
        # Take off
        self.drone.arm()
        rospy.sleep(3)
        self.drone.navigate_wait(z=1.0, auto_arm=True)
        
        # Control servo
        print("Moving servo...")
        self.servo.pwm_low(sleep=1.0)
        rospy.sleep(1)
        self.servo.pwm_neutral(sleep=1.0)
        rospy.sleep(1)
        self.servo.pwm_high(sleep=1.0)
        
        print("Mission complete!")

# Execute the task
task = GimbalTestTask(servo=17)
task.run()
```

### Task with Camera Operations

```python
from colibricf.task.servo import Task
import cv2
import rospy

class PhotographyTask(Task):
    """A task that captures photos during flight"""
    
    def mission(self):
        print("Starting photography task")
        
        # Take off
        self.drone.arm()
        rospy.sleep(3)
        self.drone.navigate_wait(z=1.0, auto_arm=True)
        
        # Move to waypoint
        self.drone.navigate_wait(x=2.0, y=0, z=1.0)
        
        # Capture image
        print("Capturing image...")
        frame = self.camera.retrieve_cv_frame()
        cv2.imwrite('drone_photo_1.jpg', frame)
        
        # Move to next waypoint
        self.drone.navigate_wait(x=0, y=2.0, z=1.0)
        
        # Capture second image
        print("Capturing second image...")
        frame = self.camera.retrieve_cv_frame()
        cv2.imwrite('drone_photo_2.jpg', frame)
        
        print("Mission complete!")

# Execute the task
task = PhotographyTask()
task.run()
```

## Task Execution

### run()

Execute the mission with automatic error handling and recovery.

```python
task = MyTask()
task.run()
```

**What `run()` does:**
1. Executes the `mission()` method
2. Catches `KeyboardInterrupt` for graceful shutdown (Ctrl+C)
3. Catches all exceptions and logs errors
4. Automatically lands the drone in case of errors
5. Ensures cleanup in all scenarios

### Mission Workflow

```python
# Execution flow:
# 1. mission() starts
# 2. Try to execute mission logic
# 3. On error/interrupt: print error message
# 4. Always: land the drone
# 5. Task ends
```

## Advanced Tasks

### Multi-Waypoint Mission

```python
from colibricf.task import Task
import rospy

class WaypointMissionTask(Task):
    """A task that visits multiple waypoints"""
    
    def __init__(self, servo=14):
        super().__init__(servo=servo)
        self.waypoints = [
            (0, 0, 1),      # x, y, z
            (2, 0, 1),
            (2, 2, 1),
            (0, 2, 1),
            (0, 0, 1),
        ]
    
    def mission(self):
        print("Starting waypoint mission")
        
        # Arm and takeoff
        self.drone.arm()
        rospy.sleep(3)
        self.drone.navigate_wait(z=1.0, auto_arm=True)
        
        # Visit all waypoints
        for i, (x, y, z) in enumerate(self.waypoints):
            print(f"Moving to waypoint {i+1}: ({x}, {y}, {z})")
            self.drone.navigate_wait(x=x, y=y, z=z, speed=0.5)
            rospy.sleep(1)  # Pause at waypoint
        
        print("All waypoints visited!")

# Execute the task
task = WaypointMissionTask()
task.run()
```

### Surveying / Grid Mission

```python
from colibricf.task import Task
import cv2
import rospy

class SurveyMissionTask(Task):
    """A task that surveys an area in a grid pattern"""
    
    def __init__(self, grid_size=3, spacing=1.0, servo=14):
        super().__init__(servo=servo)
        self.grid_size = grid_size
        self.spacing = spacing
    
    def mission(self):
        print(f"Starting survey mission ({self.grid_size}x{self.grid_size} grid)")
        
        # Arm and takeoff
        self.drone.arm()
        rospy.sleep(3)
        self.drone.navigate_wait(z=2.0, auto_arm=True)
        
        # Generate grid waypoints
        waypoints = []
        for i in range(self.grid_size):
            for j in range(self.grid_size):
                x = i * self.spacing
                y = j * self.spacing
                waypoints.append((x, y, 2.0))
        
        # Execute survey
        for idx, (x, y, z) in enumerate(waypoints):
            print(f"Grid point {idx+1}/{len(waypoints)}")
            self.drone.navigate_wait(x=x, y=y, z=z, speed=1.0)
            
            # Capture image at each point
            frame = self.camera.retrieve_cv_frame()
            cv2.imwrite(f'survey_{idx:03d}.jpg', frame)
            
            rospy.sleep(0.5)
        
        print("Survey complete!")

# Execute the task
task = SurveyMissionTask(grid_size=4, spacing=1.5)
task.run()
```

### QR Code Detection Mission

```python
from colibricf.task import Task
import rospy

class QRCodeSearchTask(Task):
    """A task that searches for QR codes"""
    
    def mission(self):
        print("Starting QR code search task")
        
        # Start QR code detection
        qr_sub = self.camera.get_qrcode_sub()
        
        # Arm and takeoff
        self.drone.arm()
        rospy.sleep(3)
        self.drone.navigate_wait(z=1.0, auto_arm=True)
        
        # Search pattern
        search_points = [
            (0, 0), (1, 0), (2, 0),
            (2, 1), (1, 1), (0, 1),
            (0, 2), (1, 2), (2, 2),
        ]
        
        print("Searching for QR codes...")
        for i, (x, y) in enumerate(search_points):
            print(f"Checking point {i+1}/{len(search_points)}")
            self.drone.navigate_wait(x=x, y=y, z=1.0, speed=0.5)
            rospy.sleep(2)  # Allow rospy to detect QR code
        
        print("Search complete!")

# Execute the task
task = QRCodeSearchTask()
task.run()
```

## Changing Servo Configuration

### change_servo_pin()

Change the GPIO pin for servo control.

```python
task = MyTask(servo=14)  # Initially GPIO 14

# Later, switch to different pin
task.change_servo_pin(gpio=17)

# The servo will now use GPIO 17
task.servo.pwm_neutral()
```

**Parameters:**

- `gpio` (int): New GPIO pin number

## Error Handling

### Automatic Error Recovery

```python
from colibricf.task import Task

class ErrorHandlingTask(Task):
    """Demonstrates error handling"""
    
    def mission(self):
        try:
            self.drone.arm()
            rospy.sleep(1)
            self.drone.navigate_wait(z=1.0, auto_arm=True)
            
            # If error occurs here...
            # result = some_risky_operation()
            
            self.drone.land_wait()
            
        except Exception as e:
            print(f"Error in mission: {e}")
            # run() will automatically land
            raise

# The run() method ensures landing even on error
task = ErrorHandlingTask()
task.run()
```

### Handling Keyboard Interrupt

```python
# User can press Ctrl+C anyrospy during execution
# run() catches it and lands the drone safely

# Example: 
# task = MyTask()
# task.run()  # Running...
# [User presses Ctrl+C]
# Output: "Aborting"
# Landing...
# Mission interrupted but safely
```

## Complete Mission Examples

### Perimeter Patrol Task

```python
from colibricf.task import Task
import rospy

class PerimeterPatrolTask(Task):
    """Patrol the perimeter of a square area"""
    
    def __init__(self, perimeter_size=3.0, servo=14):
        super().__init__(servo=servo)
        self.perimeter_size = perimeter_size
    
    def mission(self):
        print("Starting perimeter patrol")
        
        # Takeoff
        self.drone.arm()
        rospy.sleep(1)
        self.drone.navigate_wait(z=1.0, auto_arm=True)
        
        # Patrol points (square perimeter)
        p = self.perimeter_size
        patrol_points = [
            (0, 0),
            (p, 0),
            (p, p),
            (0, p),
            (0, 0),  # Return to start
        ]
        
        # Execute patrol
        for point in patrol_points:
            print(f"Patrol point: {point}")
            self.drone.navigate_wait(x=point[0], y=point[1], z=1.0)
            rospy.sleep(1)
        
        print("Patrol complete!")

# Execute
task = PerimeterPatrolTask(perimeter_size=5.0)
task.run()
```

### Orbit and Capture Task

```python
from colibricf.task import Task
import cv2
import rospy

class OrbitCaptureTask(Task):
    """Orbit a point and capture images"""
    
    def mission(self):
        print("Starting orbit capture mission")
        
        # Takeoff and position
        self.drone.arm()
        rospy.sleep(1)
        self.drone.navigate_wait(z=1.0, auto_arm=True)
        
        # Orbit while capturing
        print("Starting orbit...")
        start_rospy = rospy.rospy()
        orbit_duration = 10.0  # seconds
        
        frame_count = 0
        while rospy.rospy() - start_rospy < orbit_duration:
            self.drone.orbit(radius=1.0, speed=0.5)
            
            # Capture every 2 seconds
            if frame_count % 10 == 0:
                frame = self.camera.retrieve_cv_frame()
                cv2.imwrite(f'orbit_{frame_count}.jpg', frame)
            
            frame_count += 1
            rospy.sleep(0.1)
        
        print("Orbit complete!")

# Execute
task = OrbitCaptureTask()
task.run()
```

## Best Practices

1. **Always initialize properly**: Call `super().__init__()` in your task
2. **Use meaningful names**: Task class names should describe the mission
3. **Add logging**: Use `print()` for progress updates
4. **Handle timing**: Use `rospy.sleep()` to allow operations to complete
5. **Test incrementally**: Test each part of the mission separately first
6. **Add safety rospyouts**: Prevent infinite loops
7. **Document waypoints**: Comment your flight paths

## Debugging Tasks

```python
from colibricf.task import Task
import rospy

class DebugTask(Task):
    """Task with debugging output"""
    
    def mission(self):
        print("=== MISSION START ===")
        
        # Print telemetry before
        telem = self.drone.get_telemetry()
        print(f"Initial position: x={telem.x}, y={telem.y}, z={telem.z}")
        
        # Execute mission
        self.drone.arm()
        rospy.sleep(1)
        
        # Print telemetry after
        telem = self.drone.get_telemetry()
        print(f"Final position: x={telem.x}, y={telem.y}, z={telem.z}")
        
        print("=== MISSION END ===")

# Execute
task = DebugTask()
task.run()
```

## See Also

- [Drone Module](drone.md)
- [Camera Module](camera.md)
- [Servo Module](servo.md)
- [Clover Programming Guide](https://clover.coex.tech/programming)
