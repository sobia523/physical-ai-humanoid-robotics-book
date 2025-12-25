# Cross-References Template for Module 2: The Digital Twin

This document provides templates for cross-referencing between different parts of the digital twin simulation module and other modules in the textbook.

> **Note**: This is a template file with example links. These links are intentionally pointing to non-existent files as examples. This file should not be included in the build or should be excluded from link validation.

## Chapter Cross-References

### Linking to Other Chapters in Module 2
```
For more information about physics simulation, see [Physics Simulation in Gazebo](./chapters/02/gazebo-physics-theory.md).

For details on Unity integration, refer to [Unity for High-Fidelity Rendering](./chapters/03/unity-rendering-theory.md).

Sensor simulation concepts are covered in [Sensor Simulation](./chapters/04/sensor-theory.md).

For complete integration examples, see [Practical Integration & Exercises](./chapters/05/end-to-end-example.md).
```

### Linking to Other Modules
```
For ROS 2 fundamentals that are essential for understanding this module, see [Module 1: ROS 2 Nervous System](../ros2-nervous-system/chapters/01/introduction.md).

For URDF modeling concepts that apply to our robot models, see [Module 1: URDF and Robot Modeling](../ros2-nervous-system/chapters/04/urdf-humanoids.md).
```

## Configuration File References

### Linking to Configuration Files
```
The complete robot URDF model is available in [simple_robot.urdf](./configs/simple_robot.urdf).

For Gazebo world configurations, see [worlds directory](./configs/worlds/).

For ROS 2 topic standards, refer to [ros2_topic_standards.md](./configs/ros2_topic_standards.md).

For simulation configuration schema, see [simulation_config_schema.yaml](./configs/simulation_config_schema.yaml).
```

## Code Example References

### Linking to Code Templates
```
For a complete Gazebo launch file template, see [code_snippet_templates.md](./configs/code_snippet_templates.md#Gazebo-Launch-File-Template).

For ROS 2 LiDAR publisher implementation, refer to [code_snippet_templates.md](./configs/code_snippet_templates.md#ROS-2-LiDAR-Publisher-Template).

For Unity ROS-TCP-Connector setup, see [code_snippet_templates.md](./configs/code_snippet_templates.md#Unity-ROS-TCP-Connector-Template).
```

## Asset References

### Linking to Visual Assets
```
For digital twin concept visualization, see [digital-twin-concept-diagram.png](./assets/digital-twin-concept-diagram.png).

For hardware requirements diagrams, see the [assets directory](./assets/).
```

## Standard Cross-Reference Format

### For Internal Links (Same Module)
```
[Link Text](./relative/path/to/target.md)
```

### For External Links (Other Modules)
```
[Link Text](../module-number-subdirectory/path/to/target.md)
```

### For Configuration Files
```
[Link Text](./configs/filename.ext)
```

### For Assets
```
[Link Text](./assets/filename.ext)
```

## Common Cross-Reference Phrases

### Forward References (Looking Ahead)
- "In the next chapter on [Physics Simulation in Gazebo](./chapters/02/gazebo-physics-theory.md), we will explore..."
- "We will cover this topic in more detail in [Unity for High-Fidelity Rendering](./chapters/03/unity-rendering-theory.md)."
- "Practical applications of this concept are demonstrated in [Practical Integration & Exercises](./chapters/05/end-to-end-example.md)."

### Backward References (Looking Back)
- "As discussed in [Introduction to Digital Twins](./chapters/01/introduction-to-digital-twins.md)..."
- "Recall from [Benefits of Simulation](./chapters/01/benefits-of-simulation.md) that..."
- "Building on the concepts from [Platform Overview](./chapters/01/platform-overview.md)..."
- "Additional examples can be found in [code_snippet_templates.md](./configs/code_snippet_templates.md)."

### Related Content
- "For related information, see [Examples](./chapters/01/examples.md)."
- "See also [References](./chapters/01/references.md) for further reading."