# Unity Scene Templates for Humanoid Robotics

This directory contains Unity scene templates and examples for humanoid robot digital twin simulation.

## Files

- `robot-scene-template.unity`: Basic Unity scene template for humanoid robot integration
- `interaction-example.unity`: Complete example scene demonstrating human-robot interactions
- `validate-scenes.py`: Python script for validating Unity scene structure and components

## Scene Components

### Interaction Example Scene Structure
The `interaction-example.unity` file contains:

1. **RobotModel**: A humanoid robot with appropriate tags, mesh renderer, and scripts
2. **HumanModel**: A virtual human with appropriate tags, mesh renderer, and scripts
3. **InteractionZone**: A trigger zone for detecting human-robot interactions
4. **InteractionUI**: UI elements for interaction prompts
5. **Main Camera**: Camera with follow functionality

### Key Features
- Human-robot interaction detection
- Proper component hierarchy
- Physics collision setup
- Rendering optimization ready

## Validation

Run the validation script to check if your Unity scenes have the required components:

```bash
python validate-scenes.py
```

The script checks for:
- Essential Unity scene components
- Robot model integration
- Human model integration
- Interaction components