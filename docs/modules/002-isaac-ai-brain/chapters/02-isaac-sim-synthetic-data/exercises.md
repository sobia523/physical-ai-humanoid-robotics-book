---
title: Chapter 2 Exercises - Isaac Sim & Synthetic Data
sidebar_position: 5
---

# Chapter 2 Exercises - Isaac Sim & Synthetic Data

## Exercise 1: Isaac Sim Environment Design

### Objective
Design and configure an Isaac Sim environment for training a humanoid robot navigation policy.

### Instructions
Create a complete Isaac Sim environment configuration for a humanoid robot to navigate through an indoor space with obstacles.

### Requirements
1. **Environment layout**: Design a 10m x 8m indoor space with walls, doors, and corridors
2. **Furniture and obstacles**: Include tables, chairs, and other common indoor objects
3. **Lighting setup**: Configure realistic indoor lighting with multiple light sources
4. **Robot configuration**: Specify the humanoid robot with appropriate sensors
5. **Physics properties**: Set realistic material properties and dynamics

### Deliverables
- USD file or Isaac Sim configuration file defining the environment
- Detailed documentation explaining your design choices
- A 300-word analysis of how your environment supports diverse training scenarios
- Screenshot or visualization of the environment (if possible)

### Example Configuration Structure
```yaml
environment:
  name: "indoor_navigation_training"
  dimensions: [10.0, 8.0, 3.0]
  floor_material: "realistic_wood"
  wall_material: "painted_concrete"

  objects:
    - name: "table_01"
      type: "static"
      position: [2.0, 1.0, 0.0]
      properties:
        friction: 0.6
        restitution: 0.1
    # Add more objects...

  lighting:
    - name: "main_light"
      type: "distant"
      position: [-5.0, -5.0, 5.0]
      intensity: 500.0
    # Add more lights...

  robot:
    name: "humanoid_navigator"
    sensors:
      - type: "camera"
        position: [0.0, 0.0, 1.5]
        resolution: [640, 480]
      # Add other sensors...
```

### Evaluation Criteria
- Completeness of environment design
- Realism of material and physics properties
- Appropriateness for humanoid navigation training
- Documentation quality and design rationale

## Exercise 2: Domain Randomization Pipeline

### Objective
Implement a domain randomization pipeline for object detection in Isaac Sim.

### Instructions
Create a domain randomization system that varies lighting, materials, and object properties to generate diverse training data for object detection.

### Requirements
1. **Lighting randomization**: Vary intensity, color temperature, and position of light sources
2. **Material randomization**: Randomize roughness, metallic, and albedo properties
3. **Object randomization**: Vary position, orientation, and appearance of objects
4. **Sensor randomization**: Add realistic noise and distortion to camera outputs
5. **Parameter ranges**: Define appropriate ranges based on real-world variations

### Deliverables
- Python script implementing the domain randomization pipeline
- Configuration file defining randomization parameters
- Sample synthetic images with different randomization settings
- 350-word report explaining your parameter choices and validation approach

### Implementation Example
```python
import omni
from omni.isaac.core.utils.stage import add_reference_to_stage
import numpy as np

class DomainRandomizer:
    def __init__(self):
        self.light_params = {
            "intensity_range": (200, 800),
            "color_temp_range": (3000, 7000),
            "position_range": (-5, 5)
        }
        self.material_params = {
            "roughness_range": (0.1, 0.9),
            "metallic_range": (0.0, 0.5),
            "albedo_range": (0.1, 1.0)
        }

    def randomize_scene(self):
        """Randomize all configurable elements in the scene"""
        self.randomize_lighting()
        self.randomize_materials()
        self.randomize_objects()
        self.randomize_sensors()
```

### Evaluation Criteria
- Completeness of randomization implementation
- Appropriateness of parameter ranges
- Quality of synthetic data generated
- Understanding of domain randomization principles

## Exercise 3: Synthetic Data Generation for Manipulation

### Objective
Create a synthetic dataset for humanoid robot manipulation tasks using Isaac Sim.

### Instructions
Generate a synthetic dataset for a humanoid robot performing object grasping and manipulation tasks.

### Requirements
1. **Object variety**: Include 10-20 different objects with various shapes, sizes, and textures
2. **Grasp scenarios**: Create diverse scenarios with different object poses and grasping approaches
3. **Sensor data**: Generate RGB, depth, and point cloud data for each scenario
4. **Ground truth labels**: Provide accurate labels for object poses, grasp points, and success/failure
5. **Realism validation**: Include validation that the synthetic data resembles real data

### Deliverables
- Complete synthetic dataset with at least 1000 samples
- Documentation of dataset structure and labeling schema
- Comparison of synthetic vs. real data characteristics
- 400-word analysis of how the dataset supports manipulation learning

### Dataset Structure Example
```
manipulation_dataset/
├── images/
│   ├── rgb/
│   └── depth/
├── labels/
│   ├── object_poses.json
│   ├── grasp_points.json
│   └── success_labels.json
├── metadata/
│   └── scene_configurations.json
└── readme.md
```

### Evaluation Criteria
- Quality and diversity of synthetic data
- Accuracy of ground truth labels
- Appropriateness for manipulation task learning
- Validation of realism claims

## Exercise 4: Reality Gap Analysis

### Objective
Analyze the reality gap between Isaac Sim and real-world performance for a specific task.

### Instructions
Design and conduct an experiment to measure the reality gap for a humanoid robot task of your choice.

### Requirements
1. **Task selection**: Choose a specific humanoid robot task (navigation, manipulation, etc.)
2. **Simulation setup**: Create an Isaac Sim environment that approximates the real-world scenario
3. **Real-world testing**: Plan how you would test the same task in reality (or use available data)
4. **Performance metrics**: Define metrics to measure the gap between simulation and reality
5. **Gap analysis**: Identify specific factors contributing to the performance gap

### Deliverables
- Detailed experimental setup description
- Performance metrics and measurement approach
- Analysis of gap factors with specific examples
- 500-word report on findings and recommendations
- Proposed strategies to reduce the identified gaps

### Example Metrics
- Success rate: Task completion in sim vs. reality
- Time to completion: Performance timing comparison
- Path efficiency: Navigation or motion efficiency
- Sensor accuracy: Perception accuracy comparison

### Evaluation Criteria
- Rigor of experimental design
- Appropriateness of chosen metrics
- Depth of gap analysis
- Feasibility of proposed solutions

## Exercise 5: Domain Adaptation Implementation

### Objective
Implement a domain adaptation technique to improve sim-to-real transfer.

### Instructions
Choose and implement a domain adaptation method to improve the performance of a model trained on synthetic data when applied to real data.

### Requirements
1. **Method selection**: Choose an appropriate domain adaptation technique (unsupervised, self-supervised, fine-tuning, etc.)
2. **Implementation**: Implement the selected method with clear documentation
3. **Validation**: Validate the effectiveness of adaptation on sample data
4. **Analysis**: Compare results with and without domain adaptation
5. **Justification**: Explain why you chose this particular approach

### Deliverables
- Domain adaptation implementation code
- Comparison results with and without adaptation
- 450-word analysis of effectiveness and limitations
- Recommendations for improvement

### Implementation Options
- Feature alignment methods
- Adversarial domain adaptation
- Self-supervised adaptation
- Progressive domain adaptation

### Evaluation Criteria
- Correctness of implementation
- Effectiveness of adaptation method
- Quality of validation and comparison
- Understanding of domain adaptation principles

## Exercise 6: Simulation-to-Reality Transfer Pipeline

### Objective
Design a complete pipeline for transferring a trained model from Isaac Sim to a physical humanoid robot.

### Instructions
Create a comprehensive workflow that covers the entire process from simulation training to real-world deployment.

### Requirements
1. **Simulation training**: Training phase with domain randomization
2. **Validation framework**: Methods to validate simulation quality
3. **Transfer strategy**: Approach to adapt the model for reality
4. **Real-world testing**: Plan for deploying and testing on a physical robot
5. **Monitoring system**: Approach to monitor and validate real-world performance

### Deliverables
- Complete pipeline architecture diagram
- Implementation plan with tools and technologies
- 600-word detailed workflow description
- Risk assessment and mitigation strategies
- Success metrics and validation approach

### Pipeline Components
- Data generation → Model training → Domain adaptation → Real-world deployment → Performance monitoring

### Evaluation Criteria
- Completeness of the pipeline design
- Technical feasibility of implementation
- Appropriateness of validation approaches
- Consideration of practical challenges

## Exercise 7: Hands-On Synthetic Data Generation Workshop

### Objective
Execute a complete synthetic data generation pipeline in Isaac Sim for humanoid robot perception tasks, from environment setup to dataset validation.

### Instructions
Configure and run a complete synthetic data generation pipeline using Isaac Sim with domain randomization, generating a dataset suitable for training a perception model for humanoid robots.

### Requirements
1. **Environment setup**: Use the configuration files created in T030 to set up a randomized environment
2. **Domain randomization**: Implement lighting, material, and object randomization following T031 guidelines
3. **Data collection**: Generate RGB, depth, and segmentation data with proper annotations
4. **Dataset validation**: Verify dataset quality and diversity metrics
5. **Performance monitoring**: Track data generation performance and identify bottlenecks

### Deliverables
- Complete synthetic dataset with at least 500 samples
- Configuration files used for the generation process
- Performance metrics and generation statistics
- 300-word reflection on the data generation process and challenges encountered
- Sample images demonstrating the diversity of generated data

### Implementation Steps
1. Set up Isaac Sim with the configuration from `synthetic-data-generation.yaml`
2. Configure domain randomization using parameters from `domain-randomization-config.yaml`
3. Run the synthetic data generation for a predetermined number of samples
4. Validate the generated dataset for quality and diversity
5. Analyze performance metrics and optimize as needed

### Sample Implementation Script
```python
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.sensor import Camera
import numpy as np
import cv2
import json
from pathlib import Path
import time

class IsaacSimDataGenerator:
    def __init__(self, config_path="synthetic-data-generation.yaml"):
        # Initialize world and configuration
        self.world = World(stage_units_in_meters=1.0)
        self.config = self.load_config(config_path)
        self.output_dir = Path(self.config["data_generation"]["output_directory"])
        self.output_dir.mkdir(exist_ok=True)

        # Setup directories for different data types
        (self.output_dir / "rgb").mkdir(exist_ok=True)
        (self.output_dir / "depth").mkdir(exist_ok=True)
        (self.output_dir / "segmentation").mkdir(exist_ok=True)
        (self.output_dir / "labels").mkdir(exist_ok=True)

        self.camera = None
        self.sample_counter = 0

    def load_config(self, config_path):
        """Load configuration from YAML file

        Args:
            config_path (str): Path to the YAML configuration file containing
                             simulation parameters, data generation settings,
                             and domain randomization parameters

        Returns:
            dict: Configuration dictionary with all simulation settings
        """
        # Import YAML library for configuration loading
        import yaml

        # Open and parse the configuration file
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)

        # Return the loaded configuration dictionary
        return config

    def setup_camera(self, prim_path="/World/Robot/RGB_Camera"):
        """Setup camera for data collection"""
        self.camera = Camera(
            prim_path=prim_path,
            position=[0, 0, 1.5],
            frequency=30,
            resolution=(640, 480)
        )
        self.world.scene.add(self.camera)

    def apply_domain_randomization(self):
        """Apply domain randomization to the scene

        This method systematically varies environmental parameters to create
        diverse training data that helps models generalize to real-world conditions.
        It randomizes lighting conditions, material properties, object positions,
        and sensor parameters to bridge the sim-to-real gap.
        """
        # Randomize lighting conditions (intensity, color, position)
        # This helps the model become invariant to different illumination scenarios
        self.randomize_lighting_conditions()

        # Randomize material properties (roughness, metallic, albedo)
        # This ensures the model focuses on shape and structure rather than textures
        self.randomize_material_properties()

        # Randomize object positions and orientations
        # This increases dataset diversity and robustness to object placement variations
        self.randomize_object_positions_orientations()

        # Randomize camera parameters (position, orientation, noise)
        # This simulates variations in sensor placement and quality
        self.randomize_camera_parameters()

    def randomize_lighting_conditions(self):
        """Randomize lighting parameters in the scene"""
        # Implementation for lighting randomization
        pass

    def randomize_material_properties(self):
        """Randomize material properties of objects in the scene"""
        # Implementation for material randomization
        pass

    def randomize_object_positions_orientations(self):
        """Randomize positions and orientations of objects in the scene"""
        # Implementation for object position/orientation randomization
        pass

    def randomize_camera_parameters(self):
        """Randomize camera parameters including position and noise"""
        # Implementation for camera parameter randomization
        pass

    def generate_sample(self):
        """Generate a single data sample

        This method performs the complete pipeline for generating one synthetic
        data sample including domain randomization, simulation settling, data
        capture, and data storage. Each sample includes RGB, depth, and
        segmentation data with corresponding annotations.

        Returns:
            bool: True if sample generation was successful, False otherwise
        """
        # Apply domain randomization to create diverse environmental conditions
        # This ensures each sample has different lighting, materials, and object positions
        self.apply_domain_randomization()

        # Step simulation multiple times to allow objects to settle into stable positions
        # This prevents unrealistic object configurations due to physics instabilities
        for _ in range(5):
            self.world.step(render=True)

        # Capture RGB image data from the configured camera sensor
        # This provides the visual input for training perception models
        rgb_image = self.camera.get_rgb()

        # Capture depth data for 3D understanding and spatial reasoning
        # Depth information is crucial for robotic manipulation and navigation tasks
        depth_image = self.camera.get_depth()

        # Capture semantic segmentation data for pixel-level object identification
        # This provides ground truth labels for training segmentation models
        segmentation = self.camera.get_semantic_segmentation()

        # Create unique sample identifier based on counter for proper dataset organization
        sample_id = f"sample_{self.sample_counter:06d}"

        # Save all captured data with proper annotations and metadata
        # This ensures the dataset is properly structured for training workflows
        self.save_sample(sample_id, rgb_image, depth_image, segmentation)

        # Increment sample counter for next sample's unique identification
        self.sample_counter += 1

        # Return success indicator
        return True

    def save_sample(self, sample_id, rgb, depth, segmentation):
        """Save a complete data sample with all modalities and annotations

        This method saves all generated data for a single sample including RGB image,
        depth map, segmentation mask, and associated metadata/labels. The data is
        organized in a structured directory format suitable for training workflows.

        Args:
            sample_id (str): Unique identifier for this sample
            rgb (numpy.ndarray): RGB image data (H, W, 3) in RGB format
            depth (numpy.ndarray): Depth data (H, W) in meters
            segmentation (numpy.ndarray): Semantic segmentation mask (H, W) with class IDs

        Returns:
            bool: True if save operation was successful, False otherwise
        """
        # Save RGB image in standard image format for visual processing
        # Convert from RGB to BGR format required by OpenCV while preserving color information
        rgb_path = self.output_dir / "rgb" / f"{sample_id}_rgb.png"
        cv2.imwrite(str(rgb_path), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))

        # Save depth image as grayscale with appropriate bit depth
        # Store depth values in meters for metric accuracy in 3D reconstruction
        depth_path = self.output_dir / "depth" / f"{sample_id}_depth.png"
        cv2.imwrite(str(depth_path), depth)

        # Save segmentation mask with integer class labels for pixel-wise annotation
        # This provides ground truth for semantic segmentation training tasks
        seg_path = self.output_dir / "segmentation" / f"{sample_id}_seg.png"
        cv2.imwrite(str(seg_path), segmentation)

        # Create comprehensive label dictionary with all relevant metadata
        # This enables full traceability and analysis of the generated data
        labels = {
            "sample_id": sample_id,  # Unique identifier for this sample
            "timestamp": time.time(),  # Unix timestamp for temporal tracking
            "camera_pose": self.camera.get_world_pose(),  # Camera position/orientation in world space
            "camera_intrinsics": self.camera.get_intrinsic_matrix(),  # Camera internal parameters
            "randomization_params": self.get_current_randomization_state(),  # Randomization parameters used
            "scene_objects": self.get_scene_object_poses(),  # Object positions in the scene
            "lighting_conditions": self.get_current_lighting_state()  # Current lighting configuration
        }

        # Save labels in JSON format for easy parsing and integration with training pipelines
        label_path = self.output_dir / "labels" / f"{sample_id}_labels.json"
        with open(label_path, 'w') as f:
            json.dump(labels, f, indent=2)

        return True

    def get_current_randomization_state(self):
        """Get the current state of randomization parameters

        Returns:
            dict: Dictionary containing the current randomization parameters used
        """
        # Implementation to return current randomization state
        return {}

    def get_scene_object_poses(self):
        """Get poses of all objects in the current scene

        Returns:
            dict: Dictionary containing object names and their world poses
        """
        # Implementation to return object poses
        return {}

    def get_current_lighting_state(self):
        """Get the current state of lighting in the scene

        Returns:
            dict: Dictionary containing lighting configuration and parameters
        """
        # Implementation to return lighting state
        return {}

    def generate_dataset(self, num_samples=500):
        """Generate complete synthetic dataset with progress tracking

        This method orchestrates the generation of a complete synthetic dataset
        by repeatedly calling generate_sample() for the specified number of samples.
        It includes performance tracking and progress reporting to monitor the
        data generation process.

        Args:
            num_samples (int): Number of samples to generate for the dataset

        Returns:
            dict: Statistics about the dataset generation process including
                  total time, samples per second, and any error information
        """
        # Record start time for performance measurement
        start_time = time.time()

        # Generate the specified number of samples with progress tracking
        for i in range(num_samples):
            # Generate a single data sample with all modalities
            success = self.generate_sample()

            # Report progress at regular intervals for monitoring
            if i % 50 == 0 and i > 0:
                elapsed_time = time.time() - start_time
                samples_per_sec = i / elapsed_time if elapsed_time > 0 else 0
                print(f"Generated {i}/{num_samples} samples "
                      f"({samples_per_sec:.2f} samples/sec)")

            # Handle any generation failures if they occur
            if not success:
                print(f"Warning: Failed to generate sample {i}")

        # Calculate and report final performance metrics
        total_time = time.time() - start_time
        samples_per_second = num_samples / total_time if total_time > 0 else 0

        print(f"Dataset generation completed in {total_time:.2f} seconds")
        print(f"Average time per sample: {total_time/num_samples:.2f} seconds")
        print(f"Generation rate: {samples_per_second:.2f} samples/second")

        # Return performance statistics for analysis
        return {
            "total_samples": num_samples,
            "total_time": total_time,
            "samples_per_second": samples_per_second,
            "average_time_per_sample": total_time / num_samples
        }

# Usage
if __name__ == "__main__":
    generator = IsaacSimDataGenerator()
    generator.setup_camera()
    generator.generate_dataset(num_samples=500)
```

### Evaluation Criteria
- Successful execution of the data generation pipeline
- Quality and diversity of generated dataset
- Proper implementation of domain randomization
- Performance optimization and efficiency
- Quality of documentation and reflection

## Submission Guidelines

- Submit all deliverables in a single organized package
- Include implementation code in well-documented Python files
- Provide detailed documentation for all configurations
- Use clear visualizations where appropriate
- Ensure all files are properly formatted and readable
- Include a summary document with your key insights and learning outcomes

## Grading Rubric

Each exercise will be graded on:

- **Technical Implementation (40%)**: Correctness and completeness of code/configuration
- **Design Quality (25%)**: Appropriateness and creativity of design choices
- **Analysis and Documentation (20%)**: Quality of analysis and documentation
- **Understanding (15%)**: Demonstration of understanding of concepts

## Resources and References

- NVIDIA Isaac Sim Documentation
- Domain Randomization Research Papers
- Synthetic Data Generation Best Practices
- Sim-to-Real Transfer Case Studies
- Isaac ROS Integration Examples
- Real-world Robotics Dataset Examples