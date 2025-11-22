# Task Framework

The `Task` class provides an abstract base for creating complex drone missions. It simplifies mission development by handling initialization, error management, and resource cleanup automatically.

## Overview

The Task framework allows developers to focus on mission logic by providing a structured interface for drone operations. It integrates all Colibri modules (Drone, Camera, Servo) and includes built-in error handling and recovery mechanisms.

## Architecture

The Task class follows the Abstract Base Class (ABC) pattern, requiring developers to implement the `mission()` method that contains the actual flight logic.

### Class Structure

```python
import rospy
from abc import ABC, abstractmethod
from .drone import Drone, DroneMode
from .camera import Camera
from .servo import Servo
from typing import Union
from .files.logger import Logger

class Task(ABC):
    '''
    An abstract class to write mission.
    '''

    def __init__(self, gpio: Union[int, None] = None) -> None:
        self.drone = Drone()
        self.logger = Logger()
        self.logger.start()
        rospy.sleep(3)

        if gpio != None:
            self.servo = Servo(gpio)

        self.camera = Camera()

    @abstractmethod
    def mission(self) -> None:
        raise Exception("Need implementation.")

    def run(self) -> None:
        '''
        A secure method to run a mission. Useful in most cases.
        '''

        try:
            rospy.logwarn('Starting task.')
            self.mission()

        except KeyboardInterrupt:
            rospy.logwarn('Aborting task.')
            rospy.sleep(0.5)

        except Exception as e:
            rospy.logerr(e)

        finally:
            self.drone.land_wait()
            self.camera.stop()
            rospy.sleep(3)
            self.logger.stop()
    
```

## Creating a Task

### Basic Task Implementation

```python
from colibricf.task import Task
import rospy

class SimpleFlyTask(Task):
    """A simple task that takes off and lands"""

    TAKEOFF_ALTITUDE = 1.0
    
    def mission(self):
        print("Starting simple fly task")
        
        # Take off
        self.drone.arm()
        rospy.sleep(3)
        self.drone.navigate_wait(z=self.TAKEOFF_ALTITUDE, auto_arm=True)

# Execute the task
task = SimpleFlyTask()
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
5. Automatically stops recording and logs registering.
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
                waypoints.append((x, y, 0))
        
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

### Orbit and Capture Task

```python
from colibricf.task import Task
import cv2
import rospy

class OrbitCaptureTask(Task):
    """Orbit a point and capture images"""

    TAKEOFF_ALTITUDE = 1.0
    
    def mission(self):
        print("Starting orbit capture mission")
        
        # Takeoff and position
        self.drone.arm()
        rospy.sleep(1)
        self.drone.navigate_wait(z=self.TAKEOFF_ALTITUDE, auto_arm=True)
        
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

1. **Use meaningful names**: Task class names should describe the mission
3. **Handle timing**: Use `rospy.sleep()` to allow operations to complete
4. **Test incrementally**: Test each part of the mission separately first
5. **Add safety rospyouts**: Prevent infinite loops
6. **Document waypoints**: Comment your flight paths

## See Also

- [Drone Module](drone.md)
- [Camera Module](camera.md)
- [Servo Module](servo.md)
- [Clover Programming Guide](https://clover.coex.tech/en/programming.html)
