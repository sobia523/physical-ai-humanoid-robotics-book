# Integrated Simulation Configuration

This directory contains complete configuration files for the integrated humanoid robot simulation environment. The configuration includes all necessary components for a complete digital twin simulation with physics, sensors, navigation, and visualization.

## Directory Structure

```
integrated-sim/
├── complete_simulation.launch.py    # Main launch file orchestrating the entire simulation
├── robot_config.yaml               # Robot-specific configuration parameters
├── environment_config.yaml         # Environment and world configuration
├── parameters_config.yaml          # Detailed parameter configuration for all nodes
└── README.md                       # This documentation file
```

## Configuration Files Overview

### 1. complete_simulation.launch.py
The main launch file that orchestrates the complete simulation environment:
- Starts Gazebo with the humanoid navigation world
- Loads the robot model and publishes TF transforms
- Spawns the humanoid robot in the simulation
- Launches navigation and sensor processing nodes
- Sets up proper timing with timers for component synchronization

### 2. robot_config.yaml
Contains robot-specific configuration:
- Sensor definitions and parameters (LiDAR, IMU, depth camera)
- Navigation controller settings
- Physics properties
- ROS integration parameters
- Performance limits

### 3. environment_config.yaml
Defines the simulation environment:
- World settings and gravity
- Static and dynamic objects in the environment
- Navigation goals and waypoints
- Lighting and physics properties
- Sensor simulation models
- Simulation scenarios
- Safety limits

### 4. parameters_config.yaml
Detailed parameter configuration for all system components:
- Navigation planner parameters (global and local)
- Robot localization (EKF) settings
- Sensor processing parameters
- Control system PID settings
- Simulation timing and physics parameters
- Performance optimization settings

## Usage

To launch the complete integrated simulation:

```bash
# Source your ROS 2 workspace
source ~/your_robot_ws/install/setup.bash

# Launch the complete simulation
ros2 launch your_robot_bringup complete_simulation.launch.py
```

## Key Features

1. **Complete Integration**: All components (physics, sensors, navigation) work together seamlessly
2. **Modular Configuration**: Each aspect of the simulation is configurable through YAML files
3. **Realistic Sensor Simulation**: Includes noise models and realistic sensor characteristics
4. **Navigation Capabilities**: Complete navigation stack with obstacle avoidance
5. **Performance Optimized**: Configured for optimal simulation performance
6. **Extensible Design**: Easy to modify for different scenarios or robot configurations

## Customization

To customize the simulation for your specific needs:

1. **Modify Robot Configuration**: Update `robot_config.yaml` for different robot parameters
2. **Change Environment**: Edit `environment_config.yaml` to modify the world setup
3. **Adjust Navigation Parameters**: Tune navigation settings in `parameters_config.yaml`
4. **Add New Scenarios**: Define new simulation scenarios in the environment config

## Dependencies

This integrated simulation requires:
- ROS 2 Humble Hawksbill
- Gazebo Garden or Harmonic
- robot_state_publisher
- joint_state_publisher
- robot_localization
- navigation2 stack
- Your robot description and Gazebo packages

## Troubleshooting

Common issues and solutions:

- **Simulation runs slowly**: Check the physics parameters in `parameters_config.yaml` and reduce complexity if needed
- **Robot doesn't move**: Verify that all required nodes are running and TF frames are properly connected
- **Sensor data issues**: Check the sensor configuration in `robot_config.yaml` and topic remappings
- **Navigation problems**: Review the navigation parameters in `parameters_config.yaml`

## Performance Notes

The configuration is optimized for:
- Real-time simulation (1x speed)
- Reasonable CPU usage (target &lt;80%)
- Stable operation with multiple sensors
- Proper timing synchronization between components