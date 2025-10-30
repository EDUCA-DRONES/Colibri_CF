# Servo Control Module

The `Servo` class provides precise control over servo motors connected to the Colibri drone's GPIO pins. It enables manipulation of camera gimbals, cargo payloads, and other mechanical systems.

## Overview

The Servo module uses PiGPIO (pigpio) to generate PWM (Pulse Width Modulation) signals for servo motor control. This allows precise positioning of servo-actuated mechanisms on the drone.

## Initialization

```python
from colibricf import Servo

# Initialize servo on GPIO pin 14
servo = Servo(gpio=14)
```

### Parameters

- `gpio` (int): GPIO pin number where the servo is connected

### GPIO Pin Selection

Common GPIO pins available on Raspberry Pi / Clover:
- GPIO 14, 15, 17, 18, 22, 23, 24, 25, etc.

Consult your specific hardware documentation for available pins.

### Error Handling

```python
from colibricf import Servo

try:
    servo = Servo(gpio=14)
    print("Servo initialized successfully")
except Exception as e:
    print(f"Failed to initialize servo: {e}")
```

## Servo Positioning

### Standard Positions

Servo motors typically work with these standard pulse widths:

| Position | Pulse Width | Usage |
|----------|------------|-------|
| Neutral | 1500 µs | Center/resting position |
| High | 2000 µs | Maximum rotation (limited safe range) |
| Low | 500 µs | Minimum rotation (limited safe range) |

### pwm_neutral()

Move servo to neutral position (1500 µs).

```python
servo = Servo(gpio=14)
servo.pwm_neutral(sleep=0.5)
```

**Parameters:**

- `sleep` (float): Time to hold position in seconds (default: 0.5s)

**Usage:** Use this to center the servo or return to home position.

### pwm_high()

Move servo to maximum safe position (2000 µs).

```python
servo.pwm_high(sleep=1.0)
```

**Parameters:**

- `sleep` (float): Time to hold position in seconds (default: 0.5s)

**Usage:** Rotate servo to its highest position (typically ~90° rotation).

### pwm_low()

Move servo to minimum safe position (500 µs).

```python
servo.pwm_low(sleep=1.0)
```

**Parameters:**

- `sleep` (float): Time to hold position in seconds (default: 0.5s)

**Usage:** Rotate servo to its lowest position (typically -90° rotation).

## Custom Pulse Width

### set_pulsewidth()

Set servo to any custom pulse width (advanced users only).

```python
servo.set_pulsewidth(pulsewidth=1500, sleep=0.5)
```

**Parameters:**

- `pulsewidth` (int): Pulse width in microseconds (typically 500-2500 µs)
- `sleep` (float): Time to hold position in seconds (default: 0.5s)

### Safe Pulse Width Ranges

```python
servo = Servo(gpio=14)

# Safe range (most servos)
servo.set_pulsewidth(pulsewidth=1000)  # ~45° left
servo.set_pulsewidth(pulsewidth=1500)  # Center
servo.set_pulsewidth(pulsewidth=2000)  # ~45° right

# Extreme ranges (use with caution)
servo.set_pulsewidth(pulsewidth=600)   # May strain servo
servo.set_pulsewidth(pulsewidth=2400)  # May strain servo
```

## Complete Examples

### Camera Gimbal Control

```python
from colibricf import Servo, Drone
import time

# Initialize servo for camera gimbal on pin 17
gimbal = Servo(gpio=17)

# Look down
gimbal.pwm_high(sleep=1.0)
time.sleep(1)

# Look straight
gimbal.pwm_neutral(sleep=1.0)
time.sleep(1)

# Look up
gimbal.pwm_low(sleep=1.0)
time.sleep(1)
```

### Multi-Servo Control

```python
from colibricf import Servo
import time

# Multiple servos on different pins
servo1 = Servo(gpio=14)  # Gimbal pitch
servo2 = Servo(gpio=15)  # Gimbal roll

# Sweep servo 1
servo1.pwm_low(sleep=0.5)
time.sleep(0.5)
servo1.pwm_neutral(sleep=0.5)
time.sleep(0.5)
servo1.pwm_high(sleep=0.5)

# Move servo 2
servo2.pwm_neutral(sleep=1.0)
```

### Smooth Sweep Movement

```python
from colibricf import Servo
import time

servo = Servo(gpio=14)

# Smooth sweep from low to high
for pulsewidth in range(500, 2001, 50):
    servo.set_pulsewidth(pulsewidth=pulsewidth, sleep=0.1)

print("Sweep complete")
```

### Payload Release Mechanism

```python
from colibricf import Servo
import time

# Servo controlling cargo release mechanism
release_servo = Servo(gpio=18)

# Hold cargo (neutral/low position)
release_servo.pwm_low(sleep=0.5)
print("Cargo secured")

# Later: release cargo
time.sleep(5)
release_servo.pwm_high(sleep=1.0)
print("Cargo released")

# Return servo to safe position
time.sleep(2)
release_servo.pwm_neutral(sleep=0.5)
```

### Servo Sweep Test

```python
from colibricf import Servo
import time

def test_servo(gpio_pin):
    """Test servo movement at a specific GPIO pin"""
    servo = Servo(gpio=gpio_pin)
    
    print(f"Testing servo on GPIO {gpio_pin}")
    
    positions = [
        ("Low", "pwm_low"),
        ("Neutral", "pwm_neutral"),
        ("High", "pwm_high")
    ]
    
    for name, method in positions:
        print(f"  Moving to {name}...")
        getattr(servo, method)(sleep=1.0)
        time.sleep(0.5)
    
    print(f"Test complete for GPIO {gpio_pin}")

# Test multiple servos
for pin in [14, 15, 17]:
    test_servo(pin)
    print()
```

## Integration with Drone Tasks

```python
from colibricf import Task, Servo
import time

class GimbalControlTask(Task):
    """A task that demonstrates gimbal control"""
    
    def __init__(self, servo_pin=17):
        super().__init__(servo=servo_pin)
    
    def mission(self):
        print("Starting gimbal control mission")
        
        # Arm and takeoff
        self.drone.arm()
        time.sleep(1)
        self.drone.navigate_wait(z=1.0, auto_arm=True)
        
        # Move servo while hovering
        print("Tilting camera down")
        self.servo.pwm_high(sleep=1.0)
        time.sleep(2)
        
        print("Tilting camera up")
        self.servo.pwm_low(sleep=1.0)
        time.sleep(2)
        
        print("Centering camera")
        self.servo.pwm_neutral(sleep=1.0)
        
        # Return and land
        print("Returning to start")
        self.drone.navigate_wait(x=0, y=0, z=1.0)
        time.sleep(1)

# Run the task
task = GimbalControlTask(servo_pin=17)
task.run()
```

## Troubleshooting

### Servo Not Responding

```python
from colibricf import Servo

try:
    servo = Servo(gpio=14)
    servo.pwm_neutral()
except Exception as e:
    print(f"Error: {e}")
    print("Check:")
    print("  - GPIO pin number is correct")
    print("  - Servo is powered")
    print("  - pigpio daemon is running")
```

### Servo Jitter or Instability

```python
# Increase sleep time between commands
servo.pwm_neutral(sleep=1.0)  # Increased from 0.5s
servo.pwm_high(sleep=1.0)
```

### Servo Buzzing at Rest

- Ensure proper pulse width (1500 for neutral)
- Reduce sleep time (servo disengages when not moving)
- Check power supply capacity
- Verify servo is not mechanically stuck

## Performance Tips

- **Smooth Movements**: Increase sleep time for smoother motion
- **Power Management**: Use dedicated servo power supply for multiple servos
- **GPIO Spacing**: Use non-adjacent GPIO pins to reduce interference
- **Cable Length**: Keep servo cables short to minimize noise

## Technical Specifications

### PiGPIO PWM Details

- **PWM Frequency**: Standard 50 Hz for servo control
- **Pulse Range**: 500-2500 microseconds (typical)
- **Resolution**: 1 microsecond steps
- **Max Servos**: Limited by GPIO pins (20+ on Raspberry Pi)

### Standard Servo Characteristics

- **Operating Voltage**: 4.8-6V (typically)
- **Speed**: 0.1-0.4 seconds per 60° (model dependent)
- **Torque**: Varies by model (typically 3-20 kg·cm)
- **Weight**: 25-50g per servo

## Safety Considerations

⚠️ **Important:**
- Never exceed specified pulse width range for your servo model
- Ensure proper power supply to prevent servo damage
- Test servo movement before attaching to drone
- Use mechanical stops to prevent over-rotation
- Disconnect servo when not in use to preserve battery

## See Also

- [pigpio Documentation](http://abyz.me.uk/rpi/pigpio/)
- [Servo Motor Basics](https://en.wikipedia.org/wiki/Servo_motor)
- [PWM Explained](https://en.wikipedia.org/wiki/Pulse-width_modulation)
- [Drone Module](drone.md)
- [Task Framework](task.md)
