---
title: Domain Randomization Techniques
sidebar_position: 3
---

# Domain Randomization Techniques

## Introduction

Domain randomization is a powerful technique in synthetic data generation that systematically varies environmental parameters to create robust AI models. By training on diverse synthetic data with randomized properties, AI models learn to focus on the essential features needed for their tasks while becoming invariant to non-essential variations. This chapter explores domain randomization techniques specifically for humanoid robotics applications using NVIDIA Isaac Sim.

## The Theory Behind Domain Randomization

### Problem Statement

The "reality gap" is the performance difference between AI models trained in simulation and their performance in the real world. This gap occurs because:

1. **Visual differences**: Lighting, textures, and rendering differences between simulation and reality
2. **Physical differences**: Variations in friction, compliance, and dynamic properties
3. **Sensor differences**: Noise characteristics and calibration differences
4. **Environmental differences**: Unmodeled aspects of real environments

### Solution Approach

Domain randomization addresses the reality gap by:

1. **Increasing dataset diversity**: Randomizing parameters to cover a wide range of possible conditions
2. **Forcing feature learning**: Making models focus on essential features rather than environmental artifacts
3. **Improving generalization**: Training models to be invariant to domain-specific characteristics
4. **Reducing overfitting**: Preventing models from learning simulation-specific patterns

## Types of Domain Randomization

### Visual Domain Randomization

Visual domain randomization focuses on randomizing visual aspects of the simulation:

#### Lighting Randomization
- **Intensity**: Randomize light source intensities to simulate different times of day
- **Color temperature**: Vary from warm (3000K) to cool (7000K) lighting conditions
- **Position and direction**: Change light positions to simulate different environmental conditions
- **Number of light sources**: Vary from single key light to complex multi-light setups

#### Material and Texture Randomization
- **Surface properties**: Randomize roughness, metallic, and specular properties
- **Texture variations**: Use diverse textures for similar object categories
- **Color variations**: Randomize colors within realistic ranges
- **Reflectance properties**: Vary mirror-like to matte surface characteristics

#### Camera Parameter Randomization
- **Intrinsic parameters**: Randomize focal length, principal point, and distortion
- **Extrinsic parameters**: Vary camera position and orientation
- **Noise characteristics**: Add realistic sensor noise patterns
- **Resolution variations**: Simulate different camera qualities

### Physical Domain Randomization

Physical domain randomization focuses on randomizing physical properties:

#### Dynamics Randomization
- **Mass variations**: Randomize object masses within realistic bounds
- **Friction coefficients**: Vary friction properties for different materials
- **Damping parameters**: Randomize joint damping and compliance
- **Inertia properties**: Modify object inertial characteristics

#### Contact Randomization
- **Contact stiffness**: Vary surface contact properties
- **Restitution coefficients**: Randomize bounciness of objects
- **Contact models**: Use different contact simulation approaches
- **Surface micro-geometry**: Add subtle surface variations

### Semantic Domain Randomization

Semantic domain randomization focuses on object and scene variations:

#### Object Randomization
- **Shape variations**: Use different shapes for similar object categories
- **Size variations**: Randomize object dimensions within realistic ranges
- **Position and orientation**: Vary object poses in the scene
- **Instance diversity**: Use multiple models for the same object category

#### Scene Randomization
- **Layout variations**: Change object arrangements and positions
- **Background diversity**: Use different background elements
- **Clutter levels**: Vary scene complexity and object density
- **Environmental conditions**: Simulate different weather and lighting scenarios

## Implementation Strategies

### Parameter Ranges and Distributions

Effective domain randomization requires careful selection of parameter ranges:

#### Continuous Parameters
For continuous parameters, use appropriate distributions:
- **Uniform distribution**: When all values in a range are equally likely
- **Normal distribution**: When values cluster around a mean
- **Log-uniform distribution**: For parameters that span several orders of magnitude

```python
import numpy as np

# Example parameter randomization strategies
class DomainRandomizationParameters:
    def __init__(self):
        # Lighting parameters
        self.light_intensity_range = (100, 1000)  # Lumens
        self.light_color_temperature_range = (3000, 7000)  # Kelvin
        self.light_position_range = (-5, 5)  # Meters

        # Material parameters
        self.roughness_range = (0.1, 0.9)
        self.metallic_range = (0.0, 0.5)
        self.albedo_range = (0.1, 1.0)

        # Physical parameters
        self.mass_variation = 0.2  # ±20% from base value
        self.friction_range = (0.3, 0.8)
        self.damping_range = (0.01, 0.1)

    def sample_light_intensity(self):
        """Sample light intensity with log distribution"""
        min_val, max_val = self.light_intensity_range
        return np.exp(np.random.uniform(np.log(min_val), np.log(max_val)))

    def sample_roughness(self):
        """Sample roughness with uniform distribution"""
        min_val, max_val = self.roughness_range
        return np.random.uniform(min_val, max_val)

    def sample_mass(self, base_mass):
        """Sample mass with normal distribution around base value"""
        variation = np.random.normal(0, self.mass_variation * base_mass)
        return max(0.01, base_mass + variation)  # Ensure positive mass
```

### Discrete Parameter Selection

For discrete parameters, use appropriate selection strategies:
- **Uniform selection**: When all options are equally valid
- **Weighted selection**: When some options are more common
- **Correlated selection**: When parameters depend on each other

### Correlated Randomization

Some parameters should be randomized together to maintain physical plausibility:

```python
class CorrelatedRandomization:
    def __init__(self):
        self.material_to_properties = {
            "wood": {"roughness": (0.6, 0.9), "metallic": (0.0, 0.1)},
            "metal": {"roughness": (0.1, 0.3), "metallic": (0.8, 1.0)},
            "plastic": {"roughness": (0.3, 0.7), "metallic": (0.0, 0.2)}
        }

    def randomize_material_properties(self, material_type):
        """Randomize material properties based on material type"""
        if material_type in self.material_to_properties:
            props = self.material_to_properties[material_type]
            roughness = np.random.uniform(props["roughness"][0], props["roughness"][1])
            metallic = np.random.uniform(props["metallic"][0], props["metallic"][1])
            return roughness, metallic
        else:
            # Default randomization
            return np.random.uniform(0.1, 0.9), np.random.uniform(0.0, 0.5)
```

## Step-by-Step Implementation Guide for Domain Randomization

### 1. Setting Up Domain Randomization in Isaac Sim

To implement domain randomization in Isaac Sim, follow these steps:

#### Step 1: Initialize the Domain Randomizer
First, create a domain randomizer class that will manage all randomization parameters:

```python
import omni
from omni.isaac.core import World
from pxr import Usd, UsdGeom, UsdShade, Gf, Sdf
import numpy as np
import random

class DomainRandomizer:
    def __init__(self, world: World):
        self.world = world
        self.stage = omni.usd.get_context().get_stage()

        # Initialize randomization parameters
        self.randomization_params = {
            'lighting': {
                'enabled': True,
                'intensity_range': (200, 1000),
                'color_temp_range': (3000, 7000),
                'position_jitter': (1.0, 1.0, 0.5)
            },
            'materials': {
                'enabled': True,
                'roughness_range': (0.1, 0.9),
                'metallic_range': (0.0, 0.5),
                'albedo_range': (0.1, 1.0)
            },
            'objects': {
                'enabled': True,
                'position_jitter': (0.2, 0.2, 0.0),
                'rotation_jitter': (0.0, 0.0, 15.0),
                'scale_range': (0.8, 1.2)
            }
        }
```

#### Step 2: Configure Randomization Parameters
Set up your randomization parameters based on your specific use case:

```python
def configure_randomization(self, config_file_path=None):
    """Configure domain randomization parameters"""
    if config_file_path:
        # Load parameters from configuration file
        import yaml
        with open(config_file_path, 'r') as f:
            config = yaml.safe_load(f)
        self.randomization_params.update(config)

    # Validate parameter ranges
    self._validate_parameters()

def _validate_parameters(self):
    """Validate that all parameters are within acceptable ranges"""
    for category, params in self.randomization_params.items():
        if not params.get('enabled', False):
            continue

        for param_name, param_value in params.items():
            if param_name.endswith('_range'):
                min_val, max_val = param_value
                if min_val > max_val:
                    raise ValueError(f"Invalid range for {param_name}: min > max")
```

### 2. Implementing Lighting Randomization

Lighting randomization is crucial for creating diverse visual conditions:

```python
def randomize_lighting(self):
    """Randomize all lights in the scene"""
    # Find all light primitives in the stage
    light_prims = []
    for prim in self.stage.Traverse():
        if prim.GetTypeName() in ["DistantLight", "SphereLight", "DiskLight", "DomeLight"]:
            light_prims.append(prim)

    for light_prim in light_prims:
        # Randomize intensity
        intensity_attr = light_prim.GetAttribute("inputs:intensity")
        if intensity_attr:
            min_intensity, max_intensity = self.randomization_params['lighting']['intensity_range']
            new_intensity = random.uniform(min_intensity, max_intensity)
            intensity_attr.Set(new_intensity)

        # Randomize color temperature
        color_attr = light_prim.GetAttribute("inputs:color")
        if color_attr:
            min_temp, max_temp = self.randomization_params['lighting']['color_temp_range']
            color_temp = random.uniform(min_temp, max_temp)
            rgb_color = self._color_temperature_to_rgb(color_temp)
            color_attr.Set(Gf.Vec3f(*rgb_color))

        # Randomize position (for non-distant lights)
        if light_prim.GetTypeName() != "DistantLight":
            xformable = UsdGeom.Xformable(light_prim)
            if xformable:
                pos_jitter = self.randomization_params['lighting']['position_jitter']
                jitter = [random.uniform(-j, j) for j in pos_jitter]
                # Apply position jitter to the light
                current_pos = self._get_current_position(light_prim)
                new_pos = [current_pos[i] + jitter[i] for i in range(3)]
                self._set_position(light_prim, new_pos)

def _color_temperature_to_rgb(self, color_temp):
    """Convert color temperature in Kelvin to RGB values"""
    temp = color_temp / 100
    red = min(255, max(0,
        255 if temp >= 66 else
        temp * 29.678 - 219.525 if temp <= 66 else
        329.698 * pow(temp - 60, -0.1332)
    )) / 255.0

    green = min(255, max(0,
        temp * 14.496 - 74.377 if temp <= 66 else
        138.518 * pow(temp - 55, -0.4702)
    )) / 255.0

    blue = min(255, max(0,
        255 if temp <= 20 else
        255 if temp >= 66 else
        temp * 17.896 - 141.786
    )) / 255.0

    return [red, green, blue]

def _get_current_position(self, prim):
    """Get current world position of a prim"""
    # Implementation to get current position
    return [0.0, 0.0, 0.0]  # Placeholder

def _set_position(self, prim, position):
    """Set world position of a prim"""
    # Implementation to set position
    pass
```

### 3. Implementing Material Randomization

Material randomization affects the visual appearance of objects:

```python
def randomize_materials(self):
    """Randomize materials in the scene"""
    # Find all material prims
    material_prims = []
    for prim in self.stage.Traverse():
        if prim.GetTypeName() == "Material":
            material_prims.append(prim)

    for material_prim in material_prims:
        # Get the material surface output
        material = UsdShade.Material(material_prim)
        surface_output = material.GetOutput("surface")

        if not surface_output:
            continue

        # Get the connected shader
        connected_shader = surface_output.GetConnectedSource()
        if not connected_shader.resolvedPrim:
            continue

        shader_prim = connected_shader.resolvedPrim
        shader = UsdShade.Shader(shader_prim)

        # Randomize roughness
        roughness_input = shader.GetInput("inputs:roughness")
        if roughness_input:
            min_rough, max_rough = self.randomization_params['materials']['roughness_range']
            new_roughness = random.uniform(min_rough, max_rough)
            roughness_input.Set(new_roughness)

        # Randomize metallic
        metallic_input = shader.GetInput("inputs:metallic")
        if metallic_input:
            min_metal, max_metal = self.randomization_params['materials']['metallic_range']
            new_metallic = random.uniform(min_metal, max_metal)
            metallic_input.Set(new_metallic)

        # Randomize albedo/base color
        albedo_input = shader.GetInput("inputs:diffuse_tint")
        if albedo_input:
            min_albedo, max_albedo = self.randomization_params['materials']['albedo_range']
            albedo_factor = random.uniform(min_albedo, max_albedo)
            # Create a random color with the albedo factor
            random_color = [
                random.uniform(0.2, 1.0) * albedo_factor,
                random.uniform(0.2, 1.0) * albedo_factor,
                random.uniform(0.2, 1.0) * albedo_factor
            ]
            albedo_input.Set(Gf.Vec3f(*random_color))

def randomize_object_properties(self):
    """Randomize object positions, rotations, and scales"""
    # Find all objects in the scene
    for prim in self.stage.Traverse():
        # Skip lights, cameras, and other special prims
        if any(skip_type in prim.GetTypeName() for skip_type in
               ["Light", "Camera", "Material", "Shader"]):
            continue

        # Check if this is a geometry prim
        if UsdGeom.Imageable(prim):
            # Randomize position
            if self.randomization_params['objects']['enabled']:
                self._randomize_object_position(prim)
                self._randomize_object_rotation(prim)
                self._randomize_object_scale(prim)

def _randomize_object_position(self, prim):
    """Randomize the position of an object"""
    xformable = UsdGeom.Xformable(prim)
    if xformable:
        pos_jitter = self.randomization_params['objects']['position_jitter']
        jitter = [random.uniform(-j, j) for j in pos_jitter]

        # Get current position
        current_pos = self._get_current_position(prim)
        new_pos = [current_pos[i] + jitter[i] for i in range(3)]

        # Apply new position
        self._set_position(prim, new_pos)

def _randomize_object_rotation(self, prim):
    """Randomize the rotation of an object"""
    xformable = UsdGeom.Xformable(prim)
    if xformable:
        rot_jitter = self.randomization_params['objects']['rotation_jitter']
        jitter = [random.uniform(-j, j) for j in rot_jitter]

        # Apply rotation jitter
        current_rot = self._get_current_rotation(prim)
        new_rot = [current_rot[i] + jitter[i] for i in range(3)]

        # Apply new rotation
        self._set_rotation(prim, new_rot)

def _randomize_object_scale(self, prim):
    """Randomize the scale of an object"""
    xformable = UsdGeom.Xformable(prim)
    if xformable:
        min_scale, max_scale = self.randomization_params['objects']['scale_range']
        scale_factor = random.uniform(min_scale, max_scale)

        # Apply scale factor
        current_scale = self._get_current_scale(prim)
        new_scale = [current_scale[i] * scale_factor for i in range(3)]

        # Apply new scale
        self._set_scale(prim, new_scale)
```

### 4. Implementing Physics Randomization

Physics randomization affects how objects behave in the simulation:

```python
def randomize_physics_properties(self):
    """Randomize physics properties of objects"""
    # Find all rigid bodies in the scene
    for prim in self.stage.Traverse():
        # Check for physics properties
        if prim.HasAttribute("physics:mass"):
            # Randomize mass
            mass_attr = prim.GetAttribute("physics:mass")
            if mass_attr:
                current_mass = mass_attr.Get()
                # Apply random variation (±20%)
                variation = random.uniform(-0.2, 0.2)
                new_mass = max(0.01, current_mass * (1 + variation))  # Ensure positive mass
                mass_attr.Set(new_mass)

        if prim.HasAttribute("physics:friction"):
            # Randomize friction
            friction_attr = prim.GetAttribute("physics:friction")
            if friction_attr:
                min_friction, max_friction = self.randomization_params['materials']['roughness_range']
                new_friction = random.uniform(min_friction, max_friction)
                friction_attr.Set(new_friction)

        if prim.HasAttribute("physics:restitution"):
            # Randomize restitution (bounciness)
            restitution_attr = prim.GetAttribute("physics:restitution")
            if restitution_attr:
                new_restitution = random.uniform(0.0, 0.3)  # Typical range for non-bouncy objects
                restitution_attr.Set(new_restitution)

def _get_current_rotation(self, prim):
    """Get current rotation of a prim"""
    # Implementation to get current rotation
    return [0.0, 0.0, 0.0]  # Placeholder

def _set_rotation(self, prim, rotation):
    """Set rotation of a prim"""
    # Implementation to set rotation
    pass

def _get_current_scale(self, prim):
    """Get current scale of a prim"""
    # Implementation to get current scale
    return [1.0, 1.0, 1.0]  # Placeholder

def _set_scale(self, prim, scale):
    """Set scale of a prim"""
    # Implementation to set scale
    pass
```

### 5. Integrating Domain Randomization into Your Training Pipeline

To integrate domain randomization into your training pipeline:

```python
def apply_domain_randomization(self):
    """Apply all domain randomization techniques"""
    if self.randomization_params['lighting']['enabled']:
        self.randomize_lighting()

    if self.randomization_params['materials']['enabled']:
        self.randomize_materials()

    if self.randomization_params['objects']['enabled']:
        self.randomize_object_properties()
        self.randomize_physics_properties()

    # Step the world to update the randomized scene
    self.world.step(render=True)

# Example usage in a training loop
def training_loop_with_domain_randomization():
    # Initialize Isaac Sim world
    world = World(stage_units_in_meters=1.0)

    # Initialize domain randomizer
    randomizer = DomainRandomizer(world)

    # Configure randomization parameters
    randomizer.configure_randomization()

    # Training loop
    for episode in range(num_episodes):
        # Apply domain randomization at the beginning of each episode
        randomizer.apply_domain_randomization()

        # Reset robot to random position/orientation
        reset_robot_to_random_position()

        # Run episode
        for step in range(episode_length):
            # Execute action
            action = get_action_from_policy()
            execute_action(action)

            # Step simulation
            world.step(render=True)

        # Evaluate performance
        evaluate_episode_performance()
```

### 6. Validation and Testing of Domain Randomization

Always validate that your domain randomization is working correctly:

```python
def validate_randomization(self, num_samples=100):
    """Validate that randomization is producing diverse samples"""
    # Collect statistics on randomized parameters
    intensity_samples = []
    roughness_samples = []
    position_samples = []

    for i in range(num_samples):
        # Apply randomization
        self.apply_domain_randomization()

        # Sample parameters from the scene
        intensity = self._sample_light_intensity()
        roughness = self._sample_material_roughness()
        position = self._sample_object_position()

        intensity_samples.append(intensity)
        roughness_samples.append(roughness)
        position_samples.append(position)

    # Calculate statistics
    intensity_std = np.std(intensity_samples)
    roughness_std = np.std(roughness_samples)
    position_std = np.std([np.linalg.norm(pos) for pos in position_samples])

    # Print validation results
    print(f"Intensity standard deviation: {intensity_std}")
    print(f"Roughness standard deviation: {roughness_std}")
    print(f"Position variation: {position_std}")

    # Validate that parameters are varying appropriately
    assert intensity_std > 0.1, "Light intensity is not being randomized"
    assert roughness_std > 0.1, "Material roughness is not being randomized"
    assert position_std > 0.01, "Object positions are not being randomized"

    print("Domain randomization validation passed!")

def _sample_light_intensity(self):
    """Sample current light intensity from the scene"""
    # Implementation to sample light intensity
    return random.uniform(200, 1000)  # Placeholder

def _sample_material_roughness(self):
    """Sample current material roughness from the scene"""
    # Implementation to sample material roughness
    return random.uniform(0.1, 0.9)  # Placeholder

def _sample_object_position(self):
    """Sample current object position from the scene"""
    # Implementation to sample object position
    return [random.uniform(-1, 1) for _ in range(3)]  # Placeholder
```

## Isaac Sim Domain Randomization Implementation

### USD Stage Randomization

Isaac Sim uses USD (Universal Scene Description) for scene representation, making it well-suited for domain randomization:

```python
import omni
from pxr import Usd, UsdGeom, UsdShade, Gf, Sdf
import numpy as np

class IsaacSimDomainRandomizer:
    def __init__(self, stage):
        self.stage = stage
        self.randomization_params = DomainRandomizationParameters()

    def randomize_lighting(self):
        """Randomize all lights in the scene"""
        # Get all light prims in the stage
        light_prims = []
        for prim in self.stage.Traverse():
            if prim.GetTypeName() in ["DistantLight", "SphereLight", "DiskLight", "DomeLight"]:
                light_prims.append(prim)

        for light_prim in light_prims:
            # Randomize intensity
            intensity_attr = light_prim.GetAttribute("inputs:intensity")
            if intensity_attr:
                new_intensity = self.randomization_params.sample_light_intensity()
                intensity_attr.Set(new_intensity)

            # Randomize color temperature
            color_attr = light_prim.GetAttribute("inputs:color")
            if color_attr:
                color_temp = np.random.uniform(
                    self.randomization_params.light_color_temperature_range[0],
                    self.randomization_params.light_color_temperature_range[1]
                )
                # Convert color temperature to RGB
                rgb_color = self.color_temperature_to_rgb(color_temp)
                color_attr.Set(Gf.Vec3f(*rgb_color))

    def randomize_materials(self):
        """Randomize materials in the scene"""
        # Find all material prims
        material_prims = []
        for prim in self.stage.Traverse():
            if prim.GetTypeName() == "Material":
                material_prims.append(prim)

        for material_prim in material_prims:
            # Get the PBR shader
            surface_output = UsdShade.Material(material_prim).GetOutput("surface")
            shader_prims = surface_output.GetConnectedSource().GetPrim()

            if shader_prims and shader_prims.GetTypeName() == "Shader":
                shader = UsdShade.Shader(shader_prims)

                # Randomize roughness
                roughness_input = shader.GetInput("inputs:roughness")
                if roughness_input:
                    new_roughness = self.randomization_params.sample_roughness()
                    roughness_input.Set(new_roughness)

                # Randomize metallic
                metallic_input = shader.GetInput("inputs:metallic")
                if metallic_input:
                    new_metallic = np.random.uniform(0.0, 0.5)
                    metallic_input.Set(new_metallic)

    def color_temperature_to_rgb(self, color_temp):
        """Convert color temperature in Kelvin to RGB values"""
        # Simplified algorithm for converting color temperature to RGB
        temp = color_temp / 100
        red = min(255, max(0,
            255 if temp >= 66 else
            temp * 29.678 - 219.525 if temp <= 66 else
            329.698 * pow(temp - 60, -0.1332)
        )) / 255.0

        green = min(255, max(0,
            temp * 14.496 - 74.377 if temp <= 66 else
            138.518 * pow(temp - 55, -0.4702)
        )) / 255.0

        blue = min(255, max(0,
            255 if temp <= 20 else
            255 if temp >= 66 else
            temp * 17.896 - 141.786
        )) / 255.0

        return [red, green, blue]

    def randomize_physics_properties(self):
        """Randomize physics properties of objects"""
        # Find all rigid bodies
        for prim in self.stage.Traverse():
            if prim.HasAttribute("physics:mass"):
                mass_attr = prim.GetAttribute("physics:mass")
                base_mass = mass_attr.Get()
                new_mass = self.randomization_params.sample_mass(base_mass)
                mass_attr.Set(new_mass)

            if prim.HasAttribute("physics:friction"):
                friction_attr = prim.GetAttribute("physics:friction")
                new_friction = np.random.uniform(
                    self.randomization_params.friction_range[0],
                    self.randomization_params.friction_range[1]
                )
                friction_attr.Set(new_friction)
```

### Isaac Sim Extensions for Domain Randomization

Isaac Sim provides specific extensions for domain randomization:

#### Isaac Sim Domain Randomization Extension
```python
# Example of using Isaac Sim's domain randomization extension
from omni.isaac.core.utils.extensions import enable_extension

def setup_domain_randomization_extension():
    """Enable and configure domain randomization extension"""
    enable_extension("omni.isaac.domain_randomization")

    # Configure domain randomization parameters
    import omni.replicator.core as rep

    # Set up randomization using Replicator
    with rep.new_layer():
        # Randomize lighting
        lights = rep.get.light()
        with lights.randomize.light(intensity=rep.distribution.uniform(100, 1000)):
            pass

        # Randomize materials
        materials = rep.get.material()
        with materials.randomize.diffuse_reflection_roughness(rep.distribution.uniform(0.1, 0.9)):
            pass

        # Randomize object positions
        objects = rep.get.prims(path_pattern="/World/Objects/*")
        with objects.randomize.position(
            position=rep.distribution.uniform((-1, 0, -1), (1, 2, 1)),
            rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360))
        ):
            pass
```

## Advanced Domain Randomization Techniques

### Curriculum Learning with Domain Randomization

Start with less randomized environments and gradually increase the randomness:

```python
class CurriculumDomainRandomizer:
    def __init__(self):
        self.curriculum_stages = [
            {"lighting_range": (500, 600), "material_range": (0.4, 0.6), "complexity": 1},
            {"lighting_range": (400, 700), "material_range": (0.3, 0.7), "complexity": 2},
            {"lighting_range": (300, 800), "material_range": (0.2, 0.8), "complexity": 3},
            {"lighting_range": (200, 1000), "material_range": (0.1, 0.9), "complexity": 4},
        ]
        self.current_stage = 0

    def advance_curriculum(self):
        """Advance to the next curriculum stage"""
        if self.current_stage < len(self.curriculum_stages) - 1:
            self.current_stage += 1
        return self.current_stage

    def get_current_params(self):
        """Get domain randomization parameters for current stage"""
        return self.curriculum_stages[self.current_stage]
```

### Adversarial Domain Randomization

Use adversarial techniques to identify and randomize the most critical parameters:

```python
class AdversarialDomainRandomizer:
    def __init__(self, policy_network):
        self.policy_network = policy_network
        self.randomization_strengths = {
            "lighting": 0.5,
            "materials": 0.5,
            "physics": 0.3
        }

    def compute_randomization_gradients(self, state):
        """Compute gradients to identify most impactful randomization parameters"""
        # This is a simplified representation
        # In practice, this would involve computing gradients
        # of the policy performance with respect to randomization parameters
        pass

    def adapt_randomization(self, performance_metrics):
        """Adapt randomization based on performance feedback"""
        # Increase randomization for parameters that don't hurt performance
        # Decrease randomization for parameters that hurt performance
        pass
```

## Humanoid Robotics Specific Considerations

### Locomotion-Specific Randomization

For humanoid robots, consider randomization that affects locomotion:

#### Terrain Randomization
- **Surface properties**: Friction and compliance for different ground types
- **Obstacle placement**: Randomly placed obstacles for navigation training
- **Step variations**: Different step heights and depths for bipedal training
- **Slope variations**: Randomly generated slopes and inclines

#### Balance-Specific Randomization
- **Center of mass variations**: Slight variations in robot CoM for robustness
- **Inertia randomization**: Variations in link inertias
- **Actuator dynamics**: Randomized motor response characteristics
- **Sensor noise**: Realistic IMU and joint sensor noise patterns

### Manipulation-Specific Randomization

For manipulation tasks:

#### Object Property Randomization
- **Grasp affordances**: Randomize object properties that affect graspability
- **Deformable objects**: Randomize properties of cloth, cables, etc.
- **Friction variations**: Different object-to-hand friction characteristics
- **Mass distribution**: Vary object mass distribution for manipulation

#### Hand/Effector Randomization
- **Hand size variations**: Different hand sizes for grasp generalization
- **Finger properties**: Vary finger length, width, and joint limits
- **Tactile sensor noise**: Add realistic tactile sensor noise
- **Actuation randomization**: Vary finger actuation characteristics

## Validation and Evaluation

### Measuring Domain Randomization Effectiveness

To validate domain randomization effectiveness:

#### Synthetic-to-Real Performance Gap
```python
def evaluate_domain_randomization(model, synthetic_test_set, real_test_set):
    """Evaluate the synthetic-to-real performance gap"""
    synthetic_accuracy = evaluate_model(model, synthetic_test_set)
    real_accuracy = evaluate_model(model, real_test_set)

    performance_gap = synthetic_accuracy - real_accuracy

    return {
        "synthetic_accuracy": synthetic_accuracy,
        "real_accuracy": real_accuracy,
        "performance_gap": performance_gap,
        "transfer_efficiency": real_accuracy / synthetic_accuracy
    }
```

#### Ablation Studies
- Test individual randomization components
- Measure the impact of each parameter on transfer performance
- Identify parameters that hurt or help performance

#### Diversity Metrics
- Measure the diversity of generated synthetic data
- Ensure randomization covers the target domain adequately
- Monitor for mode collapse in generated data

## Best Practices

### Parameter Selection
1. **Start conservative**: Begin with small randomization ranges and expand
2. **Focus on key parameters**: Randomize parameters most likely to differ between sim and reality
3. **Consider correlations**: Randomize related parameters together
4. **Validate plausibility**: Ensure randomized parameters remain physically plausible

### Training Strategies
1. **Gradual introduction**: Introduce randomization gradually during training
2. **Monitor performance**: Track performance on validation data to detect over-randomization
3. **Combine with real data**: Use domain randomization with real data when available
4. **Validate on hardware**: Test randomized models on physical robots regularly

### Computational Considerations
1. **Efficient randomization**: Randomize parameters efficiently to avoid slowing simulation
2. **Caching strategies**: Cache randomization results when appropriate
3. **Parallel generation**: Generate multiple randomized environments in parallel
4. **Memory management**: Manage memory usage during randomization

## Troubleshooting Common Issues

### Over-Randomization
Problem: Randomizing too much can make learning impossible.
Solution: Gradually increase randomization and monitor training progress.

### Mode Collapse
Problem: Randomization generates unrealistic or repetitive scenarios.
Solution: Validate generated data and adjust parameter ranges.

### Performance Degradation
Problem: Randomization hurts performance on target domain.
Solution: Identify and remove harmful randomization components.

### Computational Bottlenecks
Problem: Randomization slows down simulation significantly.
Solution: Optimize randomization code and use efficient algorithms.

## Summary

Domain randomization is a crucial technique for bridging the reality gap in humanoid robotics. By systematically varying environmental parameters, we can create AI models that are robust to real-world variations. The key is to balance diversity with plausibility, starting with conservative randomization and gradually expanding based on validation results.

Effective domain randomization in Isaac Sim requires understanding both the technical implementation and the specific needs of humanoid robotics applications. By carefully randomizing lighting, materials, physics properties, and scene layouts, we can create synthetic datasets that enable robust AI models for real-world deployment.

## Exercises

1. Implement a domain randomization pipeline for humanoid object detection in varying lighting conditions
2. Design a curriculum learning approach for domain randomization in humanoid navigation
3. Create a validation framework to measure the effectiveness of domain randomization for your specific application
4. Analyze which domain randomization parameters are most important for humanoid manipulation tasks