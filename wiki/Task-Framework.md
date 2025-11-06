# Task Framework

The **Task Framework** provides a structured approach to creating autonomous drone missions with built-in error handling, resource management, and automatic landing in case of failures.

## Table of Contents

- [Overview](#overview)
- [Architecture](#architecture)
- [Creating Tasks](#creating-tasks)
- [Task Execution](#task-execution)
- [Error Handling](#error-handling)
- [Servo Integration](#servo-integration)
- [Best Practices](#best-practices)
- [Mission Templates](#mission-templates)
- [Complete Examples](#complete-examples)

## Overview

The Task framework simplifies mission development by:

- **Automatic Resource Management**: Initializes drone, camera, and servo modules
- **Error Handling**: Catches exceptions and ensures safe landing
- **Keyboard Interrupt**: Graceful shutdown on Ctrl+C
- **Clean API**: Abstract base class for mission implementation
- **Reusability**: Create modular, reusable mission components

**Key Components:**

```python
from colibricf.task import Task

class Task(ABC):
    drone = Drone()          # Shared drone instance
    camera = Camera()        # Shared camera instance
    servo = Servo()          # Optional servo instance
    
    def __init__(self, gpio=None):
        # Initializes servo if GPIO provided
        pass
    
    @abstractmethod
    def mission(self):
        # Your mission code goes here
        pass
    
    def run(self):
        # Executes mission with error handling
        pass
```

## Architecture

### Abstract Base Class Pattern

The Task class uses Python's Abstract Base Class (ABC) to enforce mission implementation:

```python
from abc import ABC, abstractmethod

class Task(ABC):
    @abstractmethod
    def mission(self):
        """Must be implemented by subclasses"""
        raise Exception("Need implementation.")
```

**Benefits:**
- Forces implementation of mission logic
- Provides consistent structure
- Enables code reuse
- Simplifies error handling

### Shared Resources

All tasks share drone and camera instances:

```python
class Task(ABC):
    drone = Drone()      # Single drone instance for all tasks
    camera = Camera()    # Single camera instance for all tasks
```

**Advantages:**
- No ROS node conflicts
- Efficient resource usage
- Consistent state across tasks

## Creating Tasks

### Minimal Task

```python
from colibricf.task import Task
import rospy

class SimpleFlight(Task):
    """Basic takeoff and land mission"""
    
    def mission(self):
        print("Starting simple flight")
        
        # Arm and takeoff
        self.drone.arm()
        rospy.sleep(3)
        self.drone.navigate_wait(z=1.0, auto_arm=True)
        
        # Hover
        rospy.sleep(5)
        
        print("Mission complete!")

# Execute
task = SimpleFlight()
task.run()
```

### Task with Parameters

```python
from colibricf.task import Task
import rospy

class HeightFlight(Task):
    """Takeoff to specified height"""
    
    def __init__(self, altitude=1.0, hover_time=5):
        super().__init__()
        self.altitude = altitude
        self.hover_time = hover_time
    
    def mission(self):
        print(f"Taking off to {self.altitude}m")
        
        self.drone.arm()
        rospy.sleep(3)
        self.drone.navigate_wait(z=self.altitude, auto_arm=True)
        
        print(f"Hovering for {self.hover_time} seconds")
        rospy.sleep(self.hover_time)
        
        print("Done!")

# Execute with custom parameters
task = HeightFlight(altitude=2.5, hover_time=10)
task.run()
```

### Task with Servo

```python
from colibricf.task import Task
import rospy

class GimbalTest(Task):
    """Test camera gimbal movement"""
    
    def __init__(self):
        super().__init__(gpio=14)  # Initialize servo on GPIO 14
    
    def mission(self):
        # Arm and takeoff
        self.drone.arm()
        rospy.sleep(3)
        self.drone.navigate_wait(z=2.0, auto_arm=True)
        
        # Test gimbal
        print("Testing gimbal positions...")
        self.servo.pwm_low(sleep=1.0)
        self.servo.pwm_neutral(sleep=1.0)
        self.servo.pwm_high(sleep=1.0)
        self.servo.pwm_neutral(sleep=1.0)
        
        print("Gimbal test complete!")

task = GimbalTest()
task.run()
```

## Task Execution

### run() Method

The `run()` method provides automatic error handling:

```python
def run(self):
    try:
        self.mission()
    except KeyboardInterrupt:
        print("warning: aborting task")
    except Exception as e:
        print(f"ERROR: {e}")
    finally:
        print('note: landing')
        self.drone.land_wait()
```

**Flow:**
1. Execute `mission()` method
2. Catch keyboard interrupts (Ctrl+C)
3. Catch all other exceptions
4. **Always** land the drone in `finally` block
5. Print status messages

### Direct Mission Call (Not Recommended)

```python
# Don't do this - no error handling!
task = MyTask()
task.mission()  # ⚠️ No automatic landing on error

# Do this instead - with error handling
task = MyTask()
task.run()  # ✅ Automatic landing on error
```

## Error Handling

### Automatic Error Recovery

The framework automatically handles errors:

```python
from colibricf.task import Task
import rospy

class RiskyMission(Task):
    def mission(self):
        self.drone.arm()
        rospy.sleep(3)
        self.drone.navigate_wait(z=2.0, auto_arm=True)
        
        # If this fails, drone still lands safely
        result = some_risky_operation()
        
        # More flight operations...

task = RiskyMission()
task.run()  # Drone lands even if exception occurs
```

### Custom Error Handling

```python
from colibricf.task import Task
import rospy

class SmartMission(Task):
    def mission(self):
        try:
            self.drone.arm()
            rospy.sleep(3)
            self.drone.navigate_wait(z=2.0, auto_arm=True)
            
            # Risky operation
            self.complex_operation()
            
        except ValueError as e:
            print(f"Value error occurred: {e}")
            # Return to safe position
            self.drone.navigate_wait(x=0, y=0, z=2.0)
            
        except Exception as e:
            print(f"Unexpected error: {e}")
            # run() will handle landing
            raise

    def complex_operation(self):
        # Operation that might fail
        pass

task = SmartMission()
task.run()
```

### Keyboard Interrupt Handling

```python
from colibricf.task import Task
import rospy

class LongMission(Task):
    def mission(self):
        self.drone.arm()
        rospy.sleep(3)
        self.drone.navigate_wait(z=2.0, auto_arm=True)
        
        # Long operation (user can press Ctrl+C to stop)
        for i in range(100):
            print(f"Waypoint {i}/100")
            self.drone.navigate_wait(x=i*0.5, y=0, z=2.0)
            rospy.sleep(1)

task = LongMission()
task.run()
# User presses Ctrl+C -> automatic safe landing
```

## Servo Integration

### Changing Servo Pin

```python
from colibricf.task import Task

class MultiServoTask(Task):
    def __init__(self):
        super().__init__(gpio=14)  # Start with GPIO 14
    
    def mission(self):
        # Use GPIO 14
        self.servo.pwm_neutral()
        
        # Switch to different servo
        self.change_servo_pin(gpio=17)
        self.servo.pwm_neutral()

task = MultiServoTask()
task.run()
```

### Task Without Servo

```python
from colibricf.task import Task

class NoServoTask(Task):
    def __init__(self):
        super().__init__(gpio=None)  # No servo
    
    def mission(self):
        # Only drone and camera operations
        self.drone.navigate_wait(z=1.0, auto_arm=True)
        frame = self.camera.retrieve_cv_frame()

task = NoServoTask()
task.run()
```

## Best Practices

### 1. Use Descriptive Class Names

```python
# Good
class PerimeterSurvey(Task):
    pass

class PayloadDelivery(Task):
    pass

# Avoid
class Task1(Task):
    pass

class MyMission(Task):
    pass
```

### 2. Add Documentation

```python
class SearchAndRescue(Task):
    """
    Search mission that scans a grid pattern looking for targets.
    
    Parameters:
        grid_size (int): Size of search grid
        altitude (float): Search altitude in meters
        speed (float): Flight speed in m/s
    """
    
    def __init__(self, grid_size=5, altitude=3.0, speed=1.0):
        super().__init__()
        self.grid_size = grid_size
        self.altitude = altitude
        self.speed = speed
    
    def mission(self):
        # Mission implementation
        pass
```

### 3. Use Progress Messages

```python
class VerboseMission(Task):
    def mission(self):
        print("=== MISSION START ===")
        
        print("[1/5] Arming...")
        self.drone.arm()
        rospy.sleep(3)
        
        print("[2/5] Taking off...")
        self.drone.navigate_wait(z=2.0, auto_arm=True)
        
        print("[3/5] Moving to target...")
        self.drone.navigate_wait(x=5, y=5, z=2.0)
        
        print("[4/5] Returning home...")
        self.drone.navigate_wait(x=0, y=0, z=2.0)
        
        print("[5/5] Mission complete!")
```

### 4. Modular Design

```python
class ModularMission(Task):
    def mission(self):
        self.preflight_check()
        self.takeoff()
        self.execute_waypoints()
        self.return_home()
    
    def preflight_check(self):
        telem = self.drone.get_telemetry()
        if telem.battery_voltage < 11.0:
            raise ValueError("Battery too low!")
    
    def takeoff(self):
        self.drone.arm()
        rospy.sleep(3)
        self.drone.navigate_wait(z=2.0, auto_arm=True)
    
    def execute_waypoints(self):
        for wp in self.waypoints:
            self.drone.navigate_wait(x=wp[0], y=wp[1], z=2.0)
            rospy.sleep(1)
    
    def return_home(self):
        self.drone.navigate_wait(x=0, y=0, z=2.0)
```

### 5. Safety Timeouts

```python
import signal
from colibricf.task import Task

class TimeoutMission(Task):
    def mission(self):
        # Set maximum mission time
        signal.alarm(300)  # 5 minute timeout
        
        try:
            # Mission operations
            self.drone.arm()
            rospy.sleep(3)
            self.drone.navigate_wait(z=2.0, auto_arm=True)
            
            # Long operations...
            
        except Exception as e:
            print(f"Mission timeout or error: {e}")
        finally:
            signal.alarm(0)  # Cancel timeout
```

## Mission Templates

### Survey Mission Template

```python
from colibricf.task import Task
from colibricf.drone import Waypoint
import rospy

class SurveyMission(Task):
    """Grid-based aerial survey"""
    
    def __init__(self, grid_size=5, spacing=2.0, altitude=5.0):
        super().__init__()
        self.grid_size = grid_size
        self.spacing = spacing
        self.altitude = altitude
    
    def mission(self):
        # Generate grid waypoints
        waypoints = []
        for i in range(self.grid_size):
            for j in range(self.grid_size):
                x = i * self.spacing
                y = j * self.spacing
                waypoints.append(Waypoint(x, y, self.altitude))
        
        # Execute survey
        self.drone.arm()
        rospy.sleep(3)
        self.drone.navigate_wait(z=self.altitude, auto_arm=True)
        
        for i, wp in enumerate(waypoints):
            print(f"Grid point {i+1}/{len(waypoints)}")
            self.drone.navigate_wait(x=wp.x, y=wp.y, z=wp.z)
            rospy.sleep(0.5)
        
        # Return to start
        self.drone.navigate_wait(x=0, y=0, z=self.altitude)

task = SurveyMission(grid_size=4, spacing=3.0, altitude=8.0)
task.run()
```

### Photography Mission Template

```python
from colibricf.task import Task
import cv2
import rospy

class PhotographyMission(Task):
    """Capture photos at specified waypoints"""
    
    def __init__(self, waypoints, photo_dir='/home/pi/photos'):
        super().__init__()
        self.waypoints = waypoints
        self.photo_dir = photo_dir
    
    def mission(self):
        self.drone.arm()
        rospy.sleep(3)
        self.drone.navigate_wait(z=5.0, auto_arm=True)
        
        for i, (x, y) in enumerate(self.waypoints):
            print(f"Photo point {i+1}/{len(self.waypoints)}")
            self.drone.navigate_wait(x=x, y=y, z=5.0)
            rospy.sleep(2)
            
            # Capture photo
            frame = self.camera.retrieve_cv_frame()
            filename = f"{self.photo_dir}/photo_{i:03d}.jpg"
            cv2.imwrite(filename, frame)
            print(f"Saved: {filename}")
        
        self.drone.navigate_wait(x=0, y=0, z=5.0)

# Define waypoints
waypoints = [(0, 5), (5, 5), (5, 0), (0, 0)]
task = PhotographyMission(waypoints)
task.run()
```

### Inspection Mission Template

```python
from colibricf.task import Task
import rospy

class InspectionMission(Task):
    """Orbit inspection around a point"""
    
    def __init__(self, center_x=0, center_y=0, altitude=3.0, radius=2.0):
        super().__init__(gpio=14)  # Gimbal servo
        self.center_x = center_x
        self.center_y = center_y
        self.altitude = altitude
        self.radius = radius
    
    def mission(self):
        # Move to inspection point
        self.drone.arm()
        rospy.sleep(3)
        self.drone.navigate_wait(
            x=self.center_x, 
            y=self.center_y, 
            z=self.altitude, 
            auto_arm=True
        )
        
        # Point camera at target
        self.servo.pwm_low(sleep=1.0)
        
        # Orbit inspection
        import threading
        orbit_thread = threading.Thread(
            target=lambda: self.drone.orbit(radius=self.radius, speed=0.3)
        )
        orbit_thread.start()
        rospy.sleep(20)  # Orbit for 20 seconds
        
        # Return camera to neutral
        self.servo.pwm_neutral()

task = InspectionMission(center_x=10, center_y=5, altitude=5.0)
task.run()
```

## Complete Examples

### Delivery Mission

```python
from colibricf.task import Task
from colibricf.drone import Waypoint
import rospy

class DeliveryMission(Task):
    """Autonomous delivery mission"""
    
    def __init__(self, delivery_location, return_home=True):
        super().__init__(gpio=17)  # Payload release servo
        self.delivery_location = delivery_location
        self.return_home = return_home
        self.FLIGHT_ALTITUDE = 10.0
    
    def mission(self):
        print("=== DELIVERY MISSION START ===")
        
        # Secure payload
        print("[1/6] Securing payload...")
        self.servo.pwm_low(sleep=0.5)
        
        # Takeoff
        print("[2/6] Taking off...")
        self.drone.arm()
        rospy.sleep(3)
        self.drone.navigate_wait(z=self.FLIGHT_ALTITUDE, auto_arm=True)
        
        # Navigate to delivery point
        print(f"[3/6] Flying to delivery location...")
        self.drone.navigate_wait(
            x=self.delivery_location[0],
            y=self.delivery_location[1],
            z=self.FLIGHT_ALTITUDE
        )
        
        # Deliver payload
        print("[4/6] Delivering payload...")
        rospy.sleep(2)
        self.servo.pwm_high(sleep=2.0)
        print("Payload delivered!")
        self.servo.pwm_neutral()
        
        # Return home
        if self.return_home:
            print("[5/6] Returning to launch point...")
            self.drone.navigate_wait(x=0, y=0, z=self.FLIGHT_ALTITUDE)
        
        print("[6/6] Mission complete!")

# Execute delivery mission
delivery_point = (25, 15)  # 25m x, 15m y from start
task = DeliveryMission(delivery_point, return_home=True)
task.run()
```

### Multi-Stage Mission

```python
from colibricf.task import Task
import rospy
import cv2

class MultiStageMission(Task):
    """Complex multi-stage mission"""
    
    def mission(self):
        print("=== MULTI-STAGE MISSION ===")
        
        # Stage 1: Preflight
        if not self.stage_preflight():
            raise Exception("Preflight failed!")
        
        # Stage 2: Survey
        self.stage_survey()
        
        # Stage 3: Photo capture
        self.stage_photography()
        
        # Stage 4: Return
        self.stage_return()
        
        print("All stages complete!")
    
    def stage_preflight(self):
        print("\n[STAGE 1] Preflight checks")
        telem = self.drone.get_telemetry()
        
        if telem.battery_voltage < 11.0:
            print("Battery too low!")
            return False
        
        print(f"Battery: {telem.battery_voltage:.1f}V - OK")
        return True
    
    def stage_survey(self):
        print("\n[STAGE 2] Survey mission")
        self.drone.arm()
        rospy.sleep(3)
        self.drone.navigate_wait(z=5.0, auto_arm=True)
        
        # Survey waypoints
        waypoints = [(0, 5), (5, 5), (5, 0)]
        for wp in waypoints:
            self.drone.navigate_wait(x=wp[0], y=wp[1], z=5.0)
            rospy.sleep(1)
    
    def stage_photography(self):
        print("\n[STAGE 3] Photography")
        for i in range(3):
            frame = self.camera.retrieve_cv_frame()
            cv2.imwrite(f'/home/pi/mission/stage3_{i}.jpg', frame)
            rospy.sleep(2)
    
    def stage_return(self):
        print("\n[STAGE 4] Return to launch")
        self.drone.navigate_wait(x=0, y=0, z=5.0)

task = MultiStageMission()
task.run()
```

---

## See Also

- [Drone Module](Drone-Module.md) - Core flight operations
- [Camera Module](Camera-Module.md) - Vision integration
- [Servo Module](Servo-Module.md) - Hardware control
- [Examples and Tutorials](Examples-and-Tutorials.md) - More mission examples

[← Back to Home](Home.md) | [Next: Computer Vision →](Computer-Vision.md)
