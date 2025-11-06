# Colibri Code Functions Wiki

Welcome to the **Colibri Code Functions** wiki! This documentation provides comprehensive guidance for programming and controlling Colibri drones using the Clover framework.

## 📖 About

Colibri Code Functions is a Python library that simplifies drone programming by providing an intuitive interface for autonomous flight operations. Built on top of the Clover framework, it abstracts complex ROS services into easy-to-use Python methods.

**Key Features:**
- 🚁 Simplified drone flight control
- 📷 Computer vision and image processing
- 🎮 Servo motor control for gimbals and payloads
- 🎯 Task-based mission framework
- 🤖 AI-powered capabilities (face detection, person following)

## 🚀 Quick Links

### Getting Started
- **[Getting Started Guide](Getting-Started.md)** - Installation, setup, and your first flight
- **[Examples and Tutorials](Examples-and-Tutorials.md)** - Practical code examples and use cases
- **[API Reference](API-Reference.md)** - Complete method signatures and parameters

### Core Modules
- **[Drone Module](Drone-Module.md)** - Flight control, navigation, and telemetry
- **[Camera Module](Camera-Module.md)** - Image capture and processing
- **[Servo Module](Servo-Module.md)** - Hardware control for servos
- **[Task Framework](Task-Framework.md)** - Mission planning and execution

### Advanced Topics
- **[Computer Vision Guide](Computer-Vision.md)** - Face detection, QR codes, and object tracking
- **[Troubleshooting](Troubleshooting.md)** - Common issues and solutions
- **[FAQ](FAQ.md)** - Frequently asked questions

### Development
- **[Contributing Guide](Contributing.md)** - How to contribute to the project

## 🎯 Quick Start Example

```python
from colibricf.drone import Drone
import rospy

# Initialize drone
drone = Drone()

# Arm and takeoff
drone.arm()
rospy.sleep(3)
drone.navigate_wait(z=1.5, auto_arm=True)

# Hover for 5 seconds
rospy.sleep(5)

# Land
drone.land_wait()
```

## 📦 Installation

```bash
# Install via pip
pip3 install colibricf

# Or install from source
git clone https://github.com/EDUCA-DRONES/Colibri_CF.git
cd Colibri_CF
pip3 install -e .
```

## 🎓 Learning Path

### For Beginners
1. Start with the [Getting Started Guide](Getting-Started.md)
2. Read the [Drone Module](Drone-Module.md) documentation
3. Try the basic [Examples and Tutorials](Examples-and-Tutorials.md)

### For Intermediate Users
1. Explore the [Task Framework](Task-Framework.md)
2. Learn about [Camera Module](Camera-Module.md) capabilities
3. Study [Computer Vision Guide](Computer-Vision.md) for AI features

### For Advanced Users
1. Review the complete [API Reference](API-Reference.md)
2. Understand [Servo Module](Servo-Module.md) for custom hardware
3. Contribute via the [Contributing Guide](Contributing.md)

## 🛠️ System Requirements

- **Hardware**: Colibri drone with Raspberry Pi (or compatible)
- **OS**: Linux (Raspbian/Ubuntu recommended)
- **Python**: 3.7 or higher
- **Framework**: Clover framework installed and configured
- **ROS**: ROS Melodic or Noetic

## 📝 Project Structure

```
colibricf/
├── drone.py          # Core flight control module
├── camera.py         # Image capture and processing
├── servo.py          # Servo motor control
├── task.py           # Mission framework
└── cv/               # Computer vision utilities
    ├── face_detect.py
    ├── follow.py
    └── qrcode.py
```

## 🤝 Community & Support

- **GitHub Repository**: [EDUCA-DRONES/Colibri_CF](https://github.com/EDUCA-DRONES/Colibri_CF)
- **Issues**: Report bugs and request features via [GitHub Issues](https://github.com/EDUCA-DRONES/Colibri_CF/issues)
- **License**: MIT License

## 🔗 External Resources

- [Clover Documentation](https://clover.coex.tech/en/)
- [ROS Documentation](http://wiki.ros.org/)
- [OpenCV Documentation](https://docs.opencv.org/)

## ⚠️ Safety Notice

Always follow drone safety guidelines:
- Test in a safe, controlled environment
- Keep a safe distance from people and obstacles
- Have a manual override ready
- Follow local drone regulations
- Never fly over crowds or restricted areas

---

**Developed by:** Educa Drones Team  
**Version:** 1.1.4  
**Last Updated:** 2025

[Start Learning →](Getting-Started.md)
