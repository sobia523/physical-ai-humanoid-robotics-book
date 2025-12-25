#!/bin/bash
"""
Test validation script for ROS 2 fundamentals examples.
This script validates that the Python code examples have correct syntax and structure.
"""

import sys
import os
import subprocess

def validate_python_syntax(file_path):
    """Validate Python syntax for a file."""
    try:
        with open(file_path, 'r') as file:
            source = file.read()
        compile(source, file_path, 'exec')
        print(f"[OK] {file_path} - Syntax OK")
        return True
    except SyntaxError as e:
        print(f"[ERROR] {file_path} - Syntax Error: {e}")
        return False
    except Exception as e:
        print(f"[ERROR] {file_path} - Error: {e}")
        return False

def main():
    """Main validation function."""
    print("Validating ROS 2 Fundamentals code examples...")

    # Define the path to the examples (relative to this script's location)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    examples_dir = os.path.join(script_dir, "ros2_nervous_system_examples")

    # List of example files to validate
    example_files = [
        os.path.join(examples_dir, "hello_publisher.py"),
        os.path.join(examples_dir, "hello_subscriber.py"),
        os.path.join(examples_dir, "string_reversal_service.py")
    ]

    all_valid = True

    for file_path in example_files:
        if os.path.exists(file_path):
            if not validate_python_syntax(file_path):
                all_valid = False
        else:
            print(f"[ERROR] {file_path} - File does not exist")
            all_valid = False

    if all_valid:
        print("\n[SUCCESS] All code examples passed validation!")
        print("The examples have correct Python syntax and structure.")
        print("\nNote: Full ROS 2 execution testing requires a complete ROS 2 environment.")
        print("To fully test these examples, you would need to:")
        print("1. Install ROS 2 Humble Hawksbill")
        print("2. Build the workspace with: colcon build --packages-select ros2_nervous_system_examples")
        print("3. Source the setup: source install/setup.bash")
        print("4. Run the nodes with: ros2 run ros2_nervous_system_examples hello_publisher (and subscriber)")
        return 0
    else:
        print("\n[FAILURE] Some code examples failed validation!")
        return 1

if __name__ == "__main__":
    sys.exit(main())