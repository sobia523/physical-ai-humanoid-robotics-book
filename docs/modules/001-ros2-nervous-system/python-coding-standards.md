# Python Coding Standards for ROS 2 Examples

## Purpose

This document establishes coding standards for Python examples in the ROS 2 Nervous System module. These standards ensure consistency, readability, and educational value across all code examples.

## General Python Standards

### Code Style
- Follow PEP 8 style guidelines
- Use 4 spaces for indentation (no tabs)
- Maximum line length of 99 characters
- Use descriptive variable and function names
- Use snake_case for variables, functions, and methods
- Use PascalCase for class names

### Import Organization
- Standard library imports first
- Third-party imports second (including ROS packages)
- Local application imports last
- Separate each group with a blank line
- Use absolute imports when possible

```python
# Correct import order
import os
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from my_package.my_module import my_function
```

## ROS 2 Specific Standards

### Node Structure
All ROS 2 Python nodes should follow this basic structure:

```python
#!/usr/bin/env python3
"""
Module: example_node.py
Description: Brief description of what this node does
Author: Your Name
Date: YYYY-MM-DD
"""

import rclpy
from rclpy.node import Node
# Other imports...

class ExampleNode(Node):
    """
    ExampleNode - A class representing a ROS 2 node

    This node demonstrates the standard structure for ROS 2 nodes
    in the ROS 2 Nervous System module.
    """

    def __init__(self):
        """
        Initialize the ExampleNode.

        Sets up publishers, subscribers, services, and other components.
        """
        super().__init__('example_node')  # Node name follows snake_case

        # Class variables initialization
        self.example_counter = 0

        # Setup publishers, subscribers, etc.
        self.publisher_ = self.create_publisher(String, 'topic_name', 10)

        # Setup timer if needed
        self.timer = self.create_timer(0.5, self.timer_callback)

        # Log initialization
        self.get_logger().info('Example node initialized')

    def timer_callback(self):
        """
        Callback function for the timer.

        This function is called periodically based on the timer setup.
        """
        # Function implementation with clear comments
        msg = String()
        msg.data = f'Example message: {self.example_counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        self.example_counter += 1

def main(args=None):
    """
    Main function to run the ExampleNode.

    Initializes ROS 2, creates the node, and spins to keep it alive.
    """
    rclpy.init(args=args)
    example_node = ExampleNode()

    try:
        rclpy.spin(example_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        example_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Commenting Standards

### Module-level Comments
Every Python file should start with a docstring explaining the purpose of the module:

```python
"""
Module: node_name.py
Description: Detailed description of what this module/node does
Author: Author Name
Date: YYYY-MM-DD
"""
```

### Class-level Comments
Each class should have a docstring explaining its purpose:

```python
class MyNode(Node):
    """
    MyNode - A class representing a ROS 2 node

    This node performs specific functionality related to...
    """
```

### Method-level Comments
Each method should have a docstring explaining its purpose, parameters, and return values:

```python
def process_data(self, input_data):
    """
    Process input data and return processed result.

    Args:
        input_data: The data to be processed

    Returns:
        The processed data
    """
```

### Inline Comments
Use inline comments sparingly but when needed to explain complex logic:

```python
# Normalize the sensor reading to the expected range
normalized_value = (raw_value - min_val) / (max_val - min_val)
```

## Error Handling Standards

### Basic Error Handling
All code should include appropriate error handling:

```python
def safe_divide(self, numerator, denominator):
    """
    Safely divide two numbers, handling division by zero.

    Args:
        numerator: The number to be divided
        denominator: The number to divide by

    Returns:
        The result of division, or None if division by zero
    """
    try:
        result = numerator / denominator
        return result
    except ZeroDivisionError:
        self.get_logger().error(f'Division by zero: {numerator} / {denominator}')
        return None
    except Exception as e:
        self.get_logger().error(f'Unexpected error in division: {e}')
        return None
```

### ROS 2 Specific Error Handling
Handle ROS 2 specific errors appropriately:

```python
def create_publisher_with_retry(self, msg_type, topic_name, qos_profile, max_retries=3):
    """
    Create a publisher with retry logic.

    Args:
        msg_type: The message type for the publisher
        topic_name: The name of the topic
        qos_profile: QoS profile to use
        max_retries: Maximum number of retry attempts

    Returns:
        The created publisher or None if failed
    """
    for attempt in range(max_retries):
        try:
            publisher = self.create_publisher(msg_type, topic_name, qos_profile)
            self.get_logger().info(f'Successfully created publisher for {topic_name}')
            return publisher
        except Exception as e:
            self.get_logger().warn(f'Attempt {attempt + 1} to create publisher failed: {e}')
            if attempt == max_retries - 1:
                self.get_logger().error(f'Failed to create publisher after {max_retries} attempts')
                return None
            # Brief delay before retry
            time.sleep(0.1)

    return None
```

## Code Structure Standards

### Class Organization
Organize class components in this order:
1. Class docstring
2. `__init__` method
3. Public methods
4. Private methods (prefixed with `_`)
5. Property getters/setters

### Method Organization
Within methods, follow this pattern:
1. Method docstring
2. Parameter validation
3. Main logic
4. Return statement

## Example: Complete Node Following Standards

```python
#!/usr/bin/env python3
"""
Module: temperature_monitor.py
Description: Monitors temperature sensor data and alerts if thresholds are exceeded
Author: Student Name
Date: 2025-12-19
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import String


class TemperatureMonitorNode(Node):
    """
    TemperatureMonitorNode - Monitors temperature and alerts on threshold violations.

    This node subscribes to temperature data and publishes alerts when
    temperature goes above or below specified thresholds.
    """

    def __init__(self):
        """Initialize the TemperatureMonitorNode."""
        super().__init__('temperature_monitor')

        # Configuration parameters
        self.temp_threshold_high = self.declare_parameter('temp_threshold_high', 30.0).value
        self.temp_threshold_low = self.declare_parameter('temp_threshold_low', 10.0).value

        # Internal state
        self.last_temperature = None

        # Setup publishers and subscribers
        self.temp_subscriber = self.create_subscription(
            Float32,
            'temperature',
            self.temperature_callback,
            10
        )

        self.alert_publisher = self.create_publisher(String, 'temperature_alert', 10)

        self.get_logger().info(
            f'Temperature monitor initialized with thresholds: '
            f'high={self.temp_threshold_high}, low={self.temp_threshold_low}'
        )

    def temperature_callback(self, msg):
        """
        Handle incoming temperature messages.

        Args:
            msg: Float32 message containing temperature value in Celsius
        """
        current_temp = msg.data
        self.last_temperature = current_temp

        # Check for threshold violations
        if current_temp > self.temp_threshold_high:
            self.get_logger().warn(f'Temperature HIGH alert: {current_temp} > {self.temp_threshold_high}')
            self.publish_alert(f'TEMPERATURE_HIGH: {current_temp}°C')
        elif current_temp < self.temp_threshold_low:
            self.get_logger().warn(f'Temperature LOW alert: {current_temp} < {self.temp_threshold_low}')
            self.publish_alert(f'TEMPERATURE_LOW: {current_temp}°C')
        else:
            # Temperature is within normal range
            pass

    def publish_alert(self, alert_msg):
        """
        Publish an alert message.

        Args:
            alert_msg: String containing the alert message
        """
        try:
            alert = String()
            alert.data = alert_msg
            self.alert_publisher.publish(alert)
        except Exception as e:
            self.get_logger().error(f'Failed to publish alert: {e}')


def main(args=None):
    """Main function to run the TemperatureMonitorNode."""
    rclpy.init(args=args)
    node = TemperatureMonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Temperature monitor shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Testing Standards

### Unit Test Structure
When creating unit tests, follow this pattern:

```python
import unittest
from unittest.mock import Mock, patch
import rclpy
from rclpy.node import Node


class TestExampleNode(unittest.TestCase):
    """Unit tests for ExampleNode."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        rclpy.init()
        self.node = ExampleNode()

    def tearDown(self):
        """Tear down test fixtures after each test method."""
        self.node.destroy_node()
        rclpy.shutdown()

    def test_something(self):
        """Test specific functionality."""
        # Test implementation
        pass


if __name__ == '__main__':
    unittest.main()
```

## Documentation Standards

### README Files
Each example should include a README.md file with:
- Brief description of the example
- How to run the example
- Expected behavior
- Any configuration parameters

### Inline Documentation
- Use comments to explain "why" not "what" (the code shows what it does)
- Document complex algorithms with references to sources
- Explain assumptions and design decisions

## Validation Checklist

Before finalizing any code example, verify:
- [ ] Follows PEP 8 style guidelines
- [ ] Includes appropriate docstrings for modules, classes, and methods
- [ ] Has meaningful variable and function names
- [ ] Includes error handling where appropriate
- [ ] Follows ROS 2 Python node patterns
- [ ] Has clear, educational comments
- [ ] Includes parameter declarations where needed
- [ ] Follows proper resource cleanup patterns