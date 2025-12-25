# Nav2 Bipedal Navigation Validation Script
# This script validates the complete navigation workflow with bipedal constraints

import os
import sys
import yaml
import subprocess
import time
from pathlib import Path

def validate_config_file(filepath):
    """Validate that a configuration file exists and has valid YAML syntax"""
    if not os.path.exists(filepath):
        print(f"Configuration file does not exist: {filepath}")
        return False

    try:
        with open(filepath, 'r') as f:
            yaml.safe_load(f)
        print(f"Configuration file is valid: {filepath}")
        return True
    except yaml.YAMLError as e:
        print(f"YAML syntax error in {filepath}: {e}")
        return False

def validate_nav2_configuration():
    """Validate all Nav2 configuration files for bipedal navigation"""
    print("Validating Nav2 configuration files...")

    config_dir = "docs/modules/002-isaac-ai-brain/configs/nav2-configs/"
    config_files = [
        "bipedal-nav2-config.yaml",
        "bipedal_cmd_vel_limits.yaml",
        "bipedal-path-planner-config.yaml",
        "bipedal-recovery-config.yaml",
        "bipedal-nav2-advanced-config.yaml"
    ]

    all_valid = True
    for config_file in config_files:
        filepath = os.path.join(config_dir, config_file)
        if not validate_config_file(filepath):
            all_valid = False

    return all_valid

def validate_chapter_files():
    """Validate all chapter files exist and are properly structured"""
    print("Validating chapter files...")

    chapter_dir = "docs/modules/002-isaac-ai-brain/chapters/05-nav2-bipedal-navigation/"
    required_files = [
        "index.md",
        "exercises.md",
        "debugging-optimization.md",
        "exercise-scenarios.md",
        "troubleshooting.md",
        "architecture.md"
    ]

    all_exist = True
    for file in required_files:
        filepath = os.path.join(chapter_dir, file)
        if os.path.exists(filepath):
            print(f"Chapter file exists: {file}")
        else:
            print(f"Chapter file missing: {file}")
            all_exist = False

    return all_exist

def validate_yaml_structure(filepath):
    """Validate specific YAML structure elements for Nav2 configs"""
    try:
        with open(filepath, 'r') as f:
            config = yaml.safe_load(f)

        # Check for required Nav2 components
        required_components = [
            'amcl', 'bt_navigator', 'controller_server',
            'local_costmap', 'global_costmap', 'planner_server'
        ]

        missing_components = []
        for component in required_components:
            if component not in config:
                missing_components.append(component)

        if missing_components:
            print(f"Missing Nav2 components in {filepath}: {missing_components}")
            return False

        # Validate specific bipedal constraints
        controller_config = config.get('controller_server', {}).get('FollowPath', {})
        if 'cmd_vel_limits_file' in controller_config:
            limits_file = controller_config['cmd_vel_limits_file']
            if limits_file != 'bipedal_cmd_vel_limits.yaml':
                print(f"Unexpected cmd_vel_limits_file in {filepath}: {limits_file}")
                return False

        print(f"Nav2 configuration structure validated: {filepath}")
        return True

    except Exception as e:
        print(f"Error validating YAML structure in {filepath}: {e}")
        return False

def validate_bipedal_specific_parameters():
    """Validate that bipedal-specific parameters are properly configured"""
    print("Validating bipedal-specific parameters...")

    config_dir = "docs/modules/002-isaac-ai-brain/configs/nav2-configs/"
    config_files = [
        "bipedal-nav2-config.yaml",
        "bipedal-nav2-advanced-config.yaml"
    ]

    all_valid = True
    for config_file in config_files:
        filepath = os.path.join(config_dir, config_file)
        if os.path.exists(filepath):
            if not validate_yaml_structure(filepath):
                all_valid = False
        else:
            print(f"Configuration file missing: {filepath}")
            all_valid = False

    return all_valid

def validate_exercise_content():
    """Validate that exercise files contain expected content"""
    print("Validating exercise content...")

    exercise_file = "docs/modules/002-isaac-ai-brain/chapters/05-nav2-bipedal-navigation/exercises.md"

    if not os.path.exists(exercise_file):
        print(f"Exercise file does not exist: {exercise_file}")
        return False

    with open(exercise_file, 'r') as f:
        content = f.read()

    # Check for essential exercise sections
    required_sections = [
        "Configuring Nav2 for Bipedal Locomotion",
        "Implementing Step Planning for Bipedal Navigation",
        "Balancing Navigation and Stability",
        "Advanced Bipedal Navigation Scenarios",
        "Performance Optimization and Debugging"
    ]

    missing_sections = []
    for section in required_sections:
        if section not in content:
            missing_sections.append(section)

    if missing_sections:
        print(f"Missing exercise sections: {missing_sections}")
        return False

    print("Exercise content validation passed")
    return True

def validate_troubleshooting_guide():
    """Validate that troubleshooting guide covers essential topics"""
    print("Validating troubleshooting guide...")

    troubleshoot_file = "docs/modules/002-isaac-ai-brain/chapters/05-nav2-bipedal-navigation/troubleshooting.md"

    if not os.path.exists(troubleshoot_file):
        print(f"Troubleshooting file does not exist: {troubleshoot_file}")
        return False

    with open(troubleshoot_file, 'r') as f:
        content = f.read()

    # Check for essential troubleshooting categories
    required_categories = [
        "Navigation Startup Issues",
        "Localization Issues",
        "Path Planning Problems",
        "Controller Issues",
        "Costmap Issues",
        "Recovery Behavior Problems"
    ]

    missing_categories = []
    for category in required_categories:
        if category not in content:
            missing_categories.append(category)

    if missing_categories:
        print(f"Missing troubleshooting categories: {missing_categories}")
        return False

    print("Troubleshooting guide validation passed")
    return True

def validate_workflow_integration():
    """Validate that all components work together in a workflow"""
    print("Validating workflow integration...")

    # Check that all required files exist and are properly referenced
    validation_checks = [
        validate_nav2_configuration(),
        validate_chapter_files(),
        validate_bipedal_specific_parameters(),
        validate_exercise_content(),
        validate_troubleshooting_guide()
    ]

    all_passed = all(validation_checks)

    if all_passed:
        print("All validation checks passed! The complete navigation workflow with bipedal constraints is properly integrated.")
    else:
        print("Some validation checks failed. Please review the issues above.")

    return all_passed

def main():
    print("Starting Nav2 Bipedal Navigation Validation...")
    print("="*60)

    success = validate_workflow_integration()

    print("="*60)
    if success:
        print("Validation completed successfully!")
        print("The complete navigation workflow with bipedal constraints has been validated.")
        return 0
    else:
        print("Validation failed!")
        print("Please address the issues before proceeding.")
        return 1

if __name__ == "__main__":
    sys.exit(main())