# ROS 2 Workspace for Nervous System Examples

## Overview

This workspace contains example packages for the ROS 2 Nervous System module. The structure follows standard ROS 2 conventions and is designed to support the educational examples throughout the module.

## Workspace Structure

```
ros2-workspace/
├── src/
│   └── ros2_nervous_system_examples/
│       ├── package.xml          # Package manifest
│       ├── setup.py             # Python package setup
│       ├── setup.cfg            # Setup configuration
│       └── ros2_nervous_system_examples/
│           ├── __init__.py      # Python module init
│           ├── simple_publisher.py
│           ├── simple_subscriber.py
│           ├── simple_service_server.py
│           ├── simple_service_client.py
│           ├── urdf_loader.py
│           └── python_agent.py
```

## Building the Workspace

To build this workspace:

```bash
cd ros2-workspace
colcon build --packages-select ros2_nervous_system_examples
source install/setup.bash
```

## Running Examples

After building, you can run the examples using:

```bash
# Source the workspace
source install/setup.bash

# Run a specific example
ros2 run ros2_nervous_system_examples simple_publisher
```

## Package Contents

The `ros2_nervous_system_examples` package includes:

- **simple_publisher**: Basic publisher node example
- **simple_subscriber**: Basic subscriber node example
- **simple_service_server**: Service server example
- **simple_service_client**: Service client example
- **urdf_loader**: URDF loading and processing example
- **python_agent**: Python agent integration example

These examples will be expanded and referenced throughout the module chapters.