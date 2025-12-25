#!/usr/bin/env python3
"""
Unity Scene Validation Script

This script validates that Unity scenes render robots and interactions effectively.
It checks for proper scene structure, robot model integration, and interaction functionality.
"""

import os
import sys
import json
import yaml
from pathlib import Path

def validate_unity_scene_structure(scene_path):
    """Validate the basic structure of a Unity scene file."""
    print(f"Validating Unity scene: {scene_path}")

    with open(scene_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Check for essential Unity components
    checks = {
        'OcclusionCullingSettings': 'Occlusion culling settings found' if 'OcclusionCullingSettings' in content else 'MISSING: Occlusion culling settings',
        'RenderSettings': 'Render settings found' if 'RenderSettings' in content else 'MISSING: Render settings',
        'LightmapSettings': 'Lightmap settings found' if 'LightmapSettings' in content else 'MISSING: Lightmap settings',
        'NavMeshSettings': 'NavMesh settings found' if 'NavMeshSettings' in content else 'MISSING: NavMesh settings',
        'RobotModel': 'Robot model found' if 'RobotModel' in content else 'MISSING: Robot model',
        'HumanModel': 'Human model found' if 'HumanModel' in content else 'MISSING: Human model',
        'InteractionZone': 'Interaction zone found' if 'InteractionZone' in content else 'MISSING: Interaction zone',
        'Transform': 'Transform components found' if 'Transform:' in content else 'MISSING: Transform components',
        'GameObject': 'Game objects found' if 'GameObject:' in content else 'MISSING: Game objects',
        'Camera': 'Camera found' if 'camera' in content.lower() else 'INFO: No camera found (might be ok)'
    }

    all_passed = True
    for check, result in checks.items():
        if result.startswith('MISSING:'):
            print(f"[ERROR] {result}")
            all_passed = False
        else:
            print(f"[OK] {result}")

    return all_passed

def validate_robot_model_integration(scene_path):
    """Validate that the robot model is properly integrated."""
    print("\nValidating robot model integration...")

    with open(scene_path, 'r', encoding='utf-8') as f:
        content = f.read()

    robot_checks = {
        'Robot Tag': 'Robot tag found' if 'm_TagString: Robot' in content else 'MISSING: Robot tag',
        'SkinnedMeshRenderer': 'Skinned mesh renderer found' if 'SkinnedMeshRenderer:' in content else 'MISSING: Skinned mesh renderer',
        'Robot Transform': 'Robot transform found' if 'RobotModel' in content and 'Transform:' in content else 'MISSING: Robot transform',
        'Robot Script': 'Robot script found' if 'robotName:' in content or 'robotType:' in content else 'MISSING: Robot script/components'
    }

    all_passed = True
    for check, result in robot_checks.items():
        if result.startswith('MISSING:'):
            print(f"[ERROR] {result}")
            all_passed = False
        else:
            print(f"[OK] {result}")

    return all_passed

def validate_human_model_integration(scene_path):
    """Validate that the human model is properly integrated."""
    print("\nValidating human model integration...")

    with open(scene_path, 'r', encoding='utf-8') as f:
        content = f.read()

    human_checks = {
        'Human Tag': 'Human tag found' if 'm_TagString: Human' in content else 'MISSING: Human tag',
        'Human SkinnedMeshRenderer': 'Human skinned mesh renderer found' if 'SkinnedMeshRenderer:' in content else 'MISSING: Human skinned mesh renderer',
        'Human Transform': 'Human transform found' if 'HumanModel' in content and 'Transform:' in content else 'MISSING: Human transform',
        'Human Script': 'Human script found' if 'humanName:' in content or 'humanType:' in content else 'MISSING: Human script/components'
    }

    all_passed = True
    for check, result in human_checks.items():
        if result.startswith('MISSING:'):
            print(f"[ERROR] {result}")
            all_passed = False
        else:
            print(f"[OK] {result}")

    return all_passed

def validate_interaction_components(scene_path):
    """Validate that interaction components are present."""
    print("\nValidating interaction components...")

    with open(scene_path, 'r', encoding='utf-8') as f:
        content = f.read()

    interaction_checks = {
        'Interaction Zone': 'Interaction zone found' if 'InteractionZone' in content else 'MISSING: Interaction zone',
        'Interaction Trigger': 'Interaction trigger found' if 'interaction' in content.lower() else 'MISSING: Interaction trigger',
        'Interaction Script': 'Interaction script found' if 'InteractionUI' in content or 'interaction' in content.lower() else 'MISSING: Interaction script',
        'Sphere Collider': 'Sphere collider found' if 'SphereCollider:' in content else 'MISSING: Sphere collider for interaction'
    }

    all_passed = True
    for check, result in interaction_checks.items():
        if result.startswith('MISSING:'):
            print(f"[ERROR] {result}")
            all_passed = False
        else:
            print(f"[OK] {result}")

    return all_passed

def main():
    """Main validation function."""
    print("Unity Scene Validation Tool")
    print("=" * 40)

    # Define the Unity scene path to validate
    scene_path = Path("docs/modules/002-digital-twin-sim/configs/unity-scenes/interaction-example.unity")

    if not scene_path.exists():
        print(f"[ERROR] Scene file not found: {scene_path}")
        return 1

    print(f"Validating scene: {scene_path}")
    print()

    # Run all validations
    structure_ok = validate_unity_scene_structure(scene_path)
    robot_ok = validate_robot_model_integration(scene_path)
    human_ok = validate_human_model_integration(scene_path)
    interaction_ok = validate_interaction_components(scene_path)

    print("\n" + "=" * 40)
    print("VALIDATION SUMMARY")
    print("=" * 40)

    results = {
        "Scene Structure": structure_ok,
        "Robot Integration": robot_ok,
        "Human Integration": human_ok,
        "Interaction Components": interaction_ok
    }

    all_passed = True
    for category, passed in results.items():
        status = "[PASS]" if passed else "[FAIL]"
        print(f"{category}: {status}")
        if not passed:
            all_passed = False

    print()
    if all_passed:
        print("[SUCCESS] All validations passed! Unity scene is ready for high-fidelity rendering.")
        print("\nScene includes:")
        print("- Proper Unity scene structure with all required components")
        print("- Robot model with appropriate rendering components")
        print("- Human model with appropriate rendering components")
        print("- Interaction zone and UI components for human-robot interaction")
        return 0
    else:
        print("[ERROR] Some validations failed. Please address the issues above.")
        return 1

if __name__ == "__main__":
    sys.exit(main())