# Colibri Code Functions

A Python library for controlling Colibri drones using the Clover framework.

**Colibri Code Functions** is a collaborative project developed by the Educa Drones team. It provides a simplified and intuitive interface for programming drone operations, built on top of the Clover framework for autonomous flight control.

## Overview

This project abstracts complex drone control operations into easy-to-use modules, allowing developers to focus on mission logic rather than low-level flight operations. It's designed specifically for the Colibri drone line and integrates seamlessly with the Clover ecosystem.

## Features

- **Camera Control** - Manage drone camera operations and image capture
- **Servo Control** - Control servo mechanisms for camera gimbal or payload manipulation
- **Drone Flight Control** - Execute takeoff, landing, and autonomous flight operations
- **Task Management** - Structure and schedule complex drone missions

## Prerequisites

Before installing Colibri Code Functions, ensure you have the following:

- Python 3.7 or higher
- The [Clover Framework](https://clover.coex.tech/en/) installed and configured
- A compatible Colibri drone (optional, but highly recommend)

> **Note:** The Clover framework is required for this library to function properly. Please install it first following the [official documentation](https://clover.coex.tech/en/).

## Installation

Install Colibri Code Functions via pip:

```bash
pip3 install colibricf
```

Or, for development installation from the repository:

```bash
git clone https://github.com/EDUCA-DRONES/Colibri_CF.git
cd Colibri_CF
pip3 install -e .
```

## Quick Start

```python
import rospy
from colibricf.task import Task

class Mission(Task):

    TAKEOFF_ALTITUDE = 1.4

    def mission(self):
        self.drone.arm()
        rospy.sleep(2)
        self.drone.navigate_wait(x=0, y=0, z=self.TAKEOFF_ALTITUDE, frame_id='body', auto_arm=True)
        # You don't need to explicitly call the land method.

Mission().run()
```

## Project Structure

```
.
├── docs
│   ├── camera.md
│   ├── drone.md
│   ├── README.md
│   ├── servo.md
│   └── task.md
├── examples
│   ├── camera.py
│   ├── flight.py
│   ├── follow.py
│   └── waypoint.py
├── LICENSE
├── pyproject.toml
├── README.md
└── src
    └── colibricf
        ├── camera.py
        ├── cv
        │   ├── face_detect.py
        │   ├── follow.py
        │   ├── haar_cascade
        │   │   ├── haarcascade_eye.xml
        │   │   ├── haarcascade_fullbody.xml
        │   │   ├── haarcascade_upperbody.xml
        │   │   └── haarcascade.xml
        │   ├── __init__.py
        │   ├── models
        │   │   └── pose_landmarker_full.task
        │   ├── qrcode.py
        │   └── utils.py
        ├── drone.py
        ├── __init__.py
        ├── servo.py
        └── task.py


```

## Documentation

- [Drone Control](docs/drone.md) - Flight operations and navigation
- [Camera Module](docs/camera.md) - Image capture and video streaming
- [Servo Control](docs/servo.md) - Gimbal and payload manipulation
- [Task Framework](docs/task.md) - Mission planning and execution

## Examples

See the `example.py` file for complete usage examples.

## Contributing

We welcome contributions! Please feel free to submit pull requests or open issues for bugs and feature requests.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Support

For issues, questions, or contributions, please visit the [GitHub repository](https://github.com/EDUCA-DRONES/Colibri_CF).

For more information about the Clover framework, visit: https://clover.coex.tech/en/

---

**Developed by:** Educa Drones
