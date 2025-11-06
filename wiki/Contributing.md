# Contributing to Colibri Code Functions

Thank you for your interest in contributing to Colibri Code Functions! This guide will help you get started.

## Table of Contents

- [Code of Conduct](#code-of-conduct)
- [How Can I Contribute?](#how-can-i-contribute)
- [Development Setup](#development-setup)
- [Coding Standards](#coding-standards)
- [Pull Request Process](#pull-request-process)
- [Testing](#testing)
- [Documentation](#documentation)

## Code of Conduct

### Our Pledge

We are committed to providing a welcoming and inspiring community for all. Please be respectful and constructive in your interactions.

### Expected Behavior

- Use welcoming and inclusive language
- Be respectful of differing viewpoints
- Accept constructive criticism gracefully
- Focus on what is best for the community
- Show empathy towards others

## How Can I Contribute?

### Reporting Bugs

Before creating a bug report:
1. **Check existing issues** to avoid duplicates
2. **Collect information** about the bug
3. **Provide detailed description**

**Good Bug Report Includes:**
```markdown
**Description**
Clear description of the issue

**To Reproduce**
1. Step 1
2. Step 2
3. See error

**Expected Behavior**
What should happen

**Actual Behavior**
What actually happens

**Environment**
- OS: Ubuntu 20.04
- Python version: 3.8.10
- ROS version: Noetic
- Colibri CF version: 1.1.4
- Hardware: Raspberry Pi 4

**Error Messages**
```
Paste full error message
```

**Code Sample**
```python
# Minimal code to reproduce
from colibricf.drone import Drone
# ...
```
```

### Suggesting Features

Feature requests are welcome! Please:
1. **Check if feature already exists** or is planned
2. **Explain use case** and benefits
3. **Provide examples** if possible

**Good Feature Request:**
```markdown
**Feature Description**
Brief description of the feature

**Use Case**
Why is this feature needed? What problem does it solve?

**Proposed Solution**
How should it work?

**Example Usage**
```python
# How the feature would be used
drone.new_feature(param=value)
```

**Alternatives Considered**
Other ways to achieve the same goal
```

### Contributing Code

We welcome code contributions! You can contribute:

- **Bug fixes**
- **New features**
- **Performance improvements**
- **Code refactoring**
- **Test coverage**
- **Documentation improvements**

## Development Setup

### 1. Fork and Clone

```bash
# Fork the repository on GitHub first
# Then clone your fork
git clone https://github.com/YOUR_USERNAME/Colibri_CF.git
cd Colibri_CF
```

### 2. Set Up Development Environment

```bash
# Create virtual environment (recommended)
python3 -m venv venv
source venv/bin/activate

# Install in development mode
pip3 install -e .

# Install development dependencies
pip3 install pytest black flake8
```

### 3. Create a Branch

```bash
# Create branch for your feature/fix
git checkout -b feature/your-feature-name
# or
git checkout -b fix/bug-description
```

**Branch Naming:**
- `feature/feature-name` - New features
- `fix/bug-description` - Bug fixes
- `docs/documentation-update` - Documentation
- `refactor/code-improvement` - Refactoring

### 4. Make Changes

Follow [Coding Standards](#coding-standards) when making changes.

### 5. Test Your Changes

```bash
# Run tests (if available)
pytest

# Test manually with drone
python3 your_test_script.py
```

### 6. Commit Changes

```bash
# Stage changes
git add .

# Commit with descriptive message
git commit -m "Add feature: description of feature"
```

**Commit Message Format:**
```
<type>: <subject>

<body>

<footer>
```

**Types:**
- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation
- `style`: Formatting, missing semicolons, etc.
- `refactor`: Code restructuring
- `test`: Adding tests
- `chore`: Maintenance

**Example:**
```
feat: Add GPS waypoint navigation

Implement global_waypoint_navigate() method to support
GPS-based autonomous missions.

- Add GlobalWaypoint class
- Implement navigation logic
- Add error handling
- Update documentation

Closes #123
```

### 7. Push and Create Pull Request

```bash
# Push to your fork
git push origin feature/your-feature-name
```

Then create Pull Request on GitHub.

## Coding Standards

### Python Style Guide

Follow **PEP 8** with these specifics:

#### Formatting

```python
# Line length: 100 characters max
# Indentation: 4 spaces (no tabs)
# Blank lines: 2 before class/function definitions

class MyClass:
    """Class docstring."""
    
    def __init__(self):
        """Constructor docstring."""
        self.value = 0
    
    def my_method(self, param: str) -> bool:
        """
        Method docstring.
        
        Args:
            param: Parameter description
            
        Returns:
            True if successful
        """
        return True
```

#### Naming Conventions

```python
# Classes: PascalCase
class DroneController:
    pass

# Functions/methods: snake_case
def navigate_to_position():
    pass

# Constants: UPPER_CASE
MAX_ALTITUDE = 100

# Private methods: _leading_underscore
def _internal_method():
    pass
```

#### Type Hints

Use type hints for function signatures:

```python
from typing import List, Optional, Union

def navigate(x: float, y: float, z: float, 
             frame_id: str = 'body') -> None:
    """Navigate to position."""
    pass

def get_waypoints() -> List[Waypoint]:
    """Get list of waypoints."""
    return []
```

#### Docstrings

Use Google-style docstrings:

```python
def complex_function(param1: int, param2: str, 
                     param3: Optional[float] = None) -> dict:
    """
    Brief description of function.
    
    More detailed description if needed. Explain what the function
    does, any important details, etc.
    
    Args:
        param1: Description of param1
        param2: Description of param2  
        param3: Optional description. Defaults to None.
    
    Returns:
        Dictionary containing results with keys:
        - 'key1': Description
        - 'key2': Description
    
    Raises:
        ValueError: If param1 is negative
        IOError: If operation fails
    
    Example:
        >>> result = complex_function(42, "test")
        >>> print(result['key1'])
        value
    """
    if param1 < 0:
        raise ValueError("param1 must be positive")
    
    return {'key1': 'value', 'key2': param2}
```

### Code Quality Tools

```bash
# Format code with black
black src/colibricf/

# Check style with flake8
flake8 src/colibricf/

# Type check with mypy (optional)
mypy src/colibricf/
```

### ROS-Specific Guidelines

```python
# Import rospy at module level
import rospy

# Initialize node in __init__ or main
def __init__(self):
    rospy.init_node('node_name')

# Use rospy.loginfo instead of print for logging
rospy.loginfo("Information message")
rospy.logwarn("Warning message")
rospy.logerr("Error message")

# Handle ROS shutdown gracefully
while not rospy.is_shutdown():
    # Do work
    rospy.sleep(0.1)
```

## Pull Request Process

### Before Submitting

- [ ] Code follows style guidelines
- [ ] Comments added to complex code
- [ ] Docstrings updated
- [ ] Tests added/updated (if applicable)
- [ ] Documentation updated (if needed)
- [ ] No breaking changes (or clearly documented)
- [ ] Commits are clean and meaningful

### PR Description Template

```markdown
## Description
Brief description of changes

## Type of Change
- [ ] Bug fix
- [ ] New feature
- [ ] Breaking change
- [ ] Documentation update

## Testing
How was this tested?
- [ ] Tested on real hardware
- [ ] Tested in simulation
- [ ] Unit tests added

## Checklist
- [ ] Code follows style guidelines
- [ ] Self-review completed
- [ ] Comments added
- [ ] Documentation updated
- [ ] No new warnings
- [ ] Tests pass

## Related Issues
Fixes #(issue number)
```

### Review Process

1. **Automated checks** run on PR
2. **Maintainer review** of code
3. **Discussion** if changes needed
4. **Approval** and merge

## Testing

### Manual Testing

Always test on real hardware or simulation:

```python
# Create test script
from colibricf.drone import Drone
import rospy

drone = Drone()

# Test your changes
drone.your_new_feature()

# Verify behavior
telem = drone.get_telemetry()
print(f"Result: {telem}")
```

### Unit Tests (Future)

When adding tests:

```python
# tests/test_drone.py
import pytest
from colibricf.drone import Drone

def test_waypoint_creation():
    """Test Waypoint class."""
    from colibricf.drone import Waypoint
    
    wp = Waypoint(1.0, 2.0, 3.0)
    assert wp.x == 1.0
    assert wp.y == 2.0
    assert wp.z == 3.0

def test_distance_calculation():
    """Test distance calculation."""
    drone = Drone()
    
    distance = drone.get_distance(0, 0, 0, 3, 4, 0)
    assert distance == 5.0
```

Run tests:
```bash
pytest tests/
```

## Documentation

### Code Documentation

- Add docstrings to all public functions/classes
- Include examples in docstrings
- Explain complex algorithms
- Document parameters and return values

### Wiki Documentation

When adding features, update wiki:

1. **API Reference**: Add method signatures
2. **Module Guide**: Add usage examples
3. **Examples**: Add practical examples
4. **Troubleshooting**: Add common issues

### Example Documentation

```python
def new_feature(param: str, value: int = 10) -> bool:
    """
    Brief description of new feature.
    
    This feature does X, Y, and Z. It's useful when you want to
    accomplish ABC.
    
    Args:
        param: Description of parameter
        value: Optional value (default: 10)
    
    Returns:
        True if operation succeeded
    
    Raises:
        ValueError: If param is empty
    
    Example:
        Basic usage:
        >>> drone.new_feature("test")
        True
        
        With custom value:
        >>> drone.new_feature("test", value=20)
        True
    """
    if not param:
        raise ValueError("param cannot be empty")
    
    # Implementation
    return True
```

## Questions?

- **GitHub Discussions**: Ask questions
- **Issues**: Report bugs or request features
- **Email**: educadrones.contato@gmail.com

## License

By contributing, you agree that your contributions will be licensed under the MIT License.

---

**Thank you for contributing to Colibri Code Functions!** 🚁

Your contributions help make drone programming more accessible to everyone.

[← Back to Home](Home.md)
