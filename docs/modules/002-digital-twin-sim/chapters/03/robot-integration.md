# Integrating Humanoid Robot Models into Unity Scenes

## Overview

This chapter provides comprehensive instructions for integrating humanoid robot models into Unity scenes for high-fidelity rendering and digital twin applications. The integration process involves importing robot models, configuring them for Unity's rendering pipeline, and setting up the necessary components for realistic visualization and interaction.

## Prerequisites

Before beginning the robot integration process, ensure you have:

- Unity 2022.3 LTS or later installed
- Robot model files (preferably in FBX, OBJ, or DAE format)
- Appropriate textures and materials (if available)
- Basic understanding of Unity interface and workflow
- ROS-TCP-Connector package (for ROS integration)

## Robot Model Preparation

### Model Format Conversion

Unity supports several 3D model formats, but for robotics applications, the following formats are recommended:

- **FBX**: Most versatile format with full animation and material support
- **OBJ**: Simple format suitable for static models
- **DAE (Collada)**: Good for models from CAD software

#### Converting from URDF to Unity-Compatible Format

If your robot model is defined in URDF format, you'll need to convert it to a Unity-compatible format:

1. **Export from CAD Software**: If available, export the CAD model directly to FBX
2. **Use URDF to DAE Tools**: Convert URDF to DAE using tools like `collada_urdf`
3. **Import to 3D Modeling Software**: Use Blender, Maya, or 3ds Max to convert formats

### Model Optimization for Unity

Before importing into Unity, optimize your robot model:

1. **Reduce Polygon Count**: Simplify geometry while maintaining visual quality
2. **Combine Meshes**: Combine multiple meshes where possible to reduce draw calls
3. **Optimize Textures**: Use appropriate texture sizes (2048x2048 max for most applications)
4. **Clean Up Geometry**: Remove unnecessary vertices, faces, and components

## Importing Robot Models into Unity

### Basic Import Process

1. **Create Unity Project**: Start a new Unity project or open an existing one
2. **Import Model**: Drag and drop your robot model file into the Assets folder
3. **Configure Import Settings**: Unity will automatically generate import settings

### Model Import Settings

When importing your robot model, configure these important settings:

#### Model Tab Settings
```
- Scale Factor: 1 (unless your model requires scaling)
- Mesh Compression: Off (for precise robot geometry)
- Read/Write Enabled: Checked (for runtime mesh manipulation)
- Optimize Mesh: Checked (for performance)
- Generate Colliders: Unchecked (unless needed for physics)
- Import BlendShapes: Checked (for animation support)
- Import Cameras: Unchecked (unless robot has cameras)
- Import Lights: Unchecked (unless robot has lights)
```

#### Rig Tab Settings
```
- Animation Type: Humanoid (for humanoid robots)
- Avatar Definition: Create From This Model
- Configure Avatar: Click to configure humanoid mapping
```

#### Animation Tab Settings
```
- Import Animation: Checked (if your model has animations)
- Animation Compression: Optimal (balance between quality and size)
- Animation Rotation Error: 0.5
- Animation Position Error: 0.5
- Animation Scale Error: 0.5
```

### Joint Mapping for Humanoid Robots

When using Humanoid animation type, Unity requires proper joint mapping:

1. **Configure Avatar**: Click "Configure..." button in Rig tab
2. **Map Joints**: Assign Unity's humanoid joint names to your model's bones:
   - Hips, Spine, Chest, Neck, Head
   - Left/Right UpperLeg, LowerLeg, Foot, Toes
   - Left/Right UpperArm, LowerArm, Hand
   - Left/Right Thumb, Index, Middle, Ring, Little fingers

3. **Validate Mapping**: Use the "Muscle & Settings" view to verify proper mapping

## Setting Up Robot Components

### Creating Robot Prefab

1. **Create Empty GameObject**: Right-click in Hierarchy → Create Empty
2. **Name the GameObject**: "HumanoidRobot" or appropriate name
3. **Add Robot Model**: Drag your imported model as a child of the root GameObject
4. **Configure Transform**: Position and rotate the model as needed
5. **Create Prefab**: Drag the root GameObject to Assets folder to create a prefab

### Material and Shader Configuration

#### Applying PBR Materials

For realistic rendering, configure your robot materials with PBR properties:

```csharp
// Example script to configure robot materials
using UnityEngine;

public class RobotMaterialSetup : MonoBehaviour
{
    [Header("Material Properties")]
    public float metallicValue = 0.7f;
    public float smoothnessValue = 0.5f;
    public Color baseColor = Color.gray;

    [Header("Material References")]
    public Material[] robotMaterials;

    void Start()
    {
        ConfigureRobotMaterials();
    }

    void ConfigureRobotMaterials()
    {
        foreach (Material mat in robotMaterials)
        {
            if (mat != null)
            {
                // Configure PBR properties
                mat.SetFloat("_Metallic", metallicValue);
                mat.SetFloat("_Smoothness", smoothnessValue);
                mat.SetColor("_Color", baseColor);

                // Set appropriate shader
                mat.shader = Shader.Find("Universal Render Pipeline/Lit");
            }
        }
    }
}
```

#### Custom Robot Shader

Create a custom shader for specific robot rendering needs:

```hlsl
// Robot specific surface shader
Shader "Robotics/RobotSurface"
{
    Properties
    {
        _Color ("Tint", Color) = (1,1,1,1)
        _MainTex ("Albedo", 2D) = "white" {}
        _Metallic ("Metallic", Range(0,1)) = 0.0
        _Smoothness ("Smoothness", Range(0,1)) = 0.5
        _EmissionColor ("Emission", Color) = (0,0,0,1)
        _EmissionMap ("Emission Map", 2D) = "white" {}
        _RimColor ("Rim Color", Color) = (1,1,1,1)
        _RimPower ("Rim Power", Range(0.1, 10.0)) = 3.0
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" "LightMode"="UniversalForward" }
        LOD 200

        UsePass "Universal Render Pipeline/Lit/ShadowCaster"

        Pass
        {
            Name "ForwardLit"
            Tags { "LightMode" = "UniversalForward" }

            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma multi_compile _ _MAIN_LIGHT_SHADOWS
            #pragma multi_compile _ _MAIN_LIGHT_SHADOWS_CASCADE
            #pragma multi_compile _ _SHADOWS_SOFT

            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Lighting.hlsl"

            struct Attributes
            {
                float4 positionOS : POSITION;
                float3 normalOS : NORMAL;
                float4 tangentOS : TANGENT;
                float2 uv : TEXCOORD0;
            };

            struct Varyings
            {
                float4 positionCS : SV_POSITION;
                float3 positionWS : TEXCOORD0;
                float3 normalWS : TEXCOORD1;
                float4 tangentWS : TEXCOORD2;
                float2 uv : TEXCOORD3;
                half4 shadowCoord : TEXCOORD4;
            };

            TEXTURE2D(_MainTex);     SAMPLER(sampler_MainTex);
            float4 _MainTex_ST;
            float4 _Color;
            float _Metallic;
            float _Smoothness;
            float4 _EmissionColor;
            TEXTURE2D(_EmissionMap); SAMPLER(sampler_EmissionMap);
            float4 _RimColor;
            float _RimPower;

            Varyings vert(Attributes input)
            {
                Varyings output;
                VertexPositionInputs vertexInput = GetVertexPositionInputs(input.positionOS.xyz);
                VertexNormalInputs normalInput = GetVertexNormalInputs(input.normalOS, input.tangentOS);

                output.positionCS = vertexInput.positionCS;
                output.positionWS = vertexInput.positionWS;
                output.normalWS = NormalizeNormalInput(normalInput.normalWS);
                output.tangentWS = float4(normalInput.tangentWS.xyz, normalInput.tangentWS.w * unity_WorldTransformParams.w);
                output.uv = TRANSFORM_TEX(input.uv, _MainTex);
                output.shadowCoord = TransformWorldToShadowCoord(output.positionWS);

                return output;
            }

            half4 frag(Varyings input) : SV_Target
            {
                float2 uv = input.uv;
                half4 albedoAlpha = SAMPLE_TEXTURE2D(_MainTex, sampler_MainTex, uv) * _Color;
                half3 albedo = albedoAlpha.rgb;
                half alpha = albedoAlpha.a;

                half3 normalWS = NormalizePerPixelNormal(input.normalWS);

                Light mainLight = GetMainLight(input.shadowCoord);

                half3 diffuse = LightingLambert(mainLight.color, mainLight.direction, normalWS);
                half3 specular = LightingSpecular(mainLight.color, mainLight.direction,
                                                normalize(input.positionWS - _WorldSpaceCameraPos),
                                                normalWS, 1.0 - _Smoothness, alpha);

                // Rim lighting for robot highlights
                half3 viewDir = normalize(_WorldSpaceCameraPos - input.positionWS);
                half rim = 1.0 - saturate(dot(normalWS, viewDir));
                half3 rimLight = _RimColor.rgb * pow(rim, _RimPower);

                // Emission
                half3 emission = SAMPLE_TEXTURE2D(_EmissionMap, sampler_EmissionMap, uv).rgb * _EmissionColor.rgb;

                half3 finalColor = (diffuse + specular) * albedo + emission + rimLight;

                return half4(finalColor, alpha);
            }
            ENDHLSL
        }
    }
    Fallback "Universal Render Pipeline/Lit"
}
```

### Robot Animation Setup

#### Creating Animation Controller

1. **Create Animator Controller**: Right-click in Assets → Create → Animator Controller
2. **Name Controller**: "HumanoidRobotController"
3. **Assign to Robot**: Select your robot prefab and assign the controller to its Animator component

#### Animation States Setup

Create appropriate animation states for your humanoid robot:

```csharp
// Animation controller setup script
using UnityEngine;
using UnityEngine.Animations;

public class RobotAnimationSetup : MonoBehaviour
{
    [Header("Animation States")]
    public RuntimeAnimatorController robotController;
    public AnimationClip idleAnimation;
    public AnimationClip walkingAnimation;
    public AnimationClip standingUpAnimation;
    public AnimationClip sittingDownAnimation;

    private Animator animator;

    void Start()
    {
        animator = GetComponent<Animator>();
        if (animator == null)
        {
            animator = gameObject.AddComponent<Animator>();
        }

        animator.runtimeAnimatorController = robotController;

        // Initialize to idle state
        animator.Play("Idle");
    }

    public void SetRobotState(string state)
    {
        if (animator != null)
        {
            animator.Play(state);
        }
    }
}
```

### Physics Configuration for Visual Integration

#### Configuring Colliders (Visual Only)

If you need colliders for visual effects but not physics simulation:

```csharp
// Visual-only colliders for robot components
using UnityEngine;

public class RobotVisualColliders : MonoBehaviour
{
    [Header("Collider Configuration")]
    public bool useVisualColliders = true;
    public float colliderScaleFactor = 1.0f;

    void Start()
    {
        if (useVisualColliders)
        {
            SetupVisualColliders();
        }
    }

    void SetupVisualColliders()
    {
        // Add colliders to robot components for visual effects
        // (not for physics simulation)
        Transform[] robotComponents = GetComponentsInChildren<Transform>();

        foreach (Transform component in robotComponents)
        {
            if (component != transform) // Skip root
            {
                // Add a collider with isTrigger = true for visual effects only
                Collider col = component.gameObject.AddComponent<BoxCollider>();
                col.isTrigger = true;

                // Scale based on mesh bounds
                Renderer renderer = component.GetComponent<Renderer>();
                if (renderer != null)
                {
                    Bounds bounds = renderer.bounds;
                    col.bounds = new Bounds(bounds.center, bounds.size * colliderScaleFactor);
                }
            }
        }
    }
}
```

## Scene Integration

### Creating Robot Scene

1. **Create New Scene**: File → New Scene
2. **Configure Lighting**: Set up appropriate lighting for your robot
3. **Add Robot Prefab**: Drag your robot prefab into the scene
4. **Position Robot**: Place the robot appropriately in the scene

### Environment Setup

#### Lighting Configuration

```csharp
// Scene lighting setup for robot visualization
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;

public class RobotSceneLighting : MonoBehaviour
{
    [Header("Lighting Configuration")]
    public Light mainLight;
    public Color environmentColor = Color.grey;
    public float ambientIntensity = 0.2f;

    [Header("Reflection Probes")]
    public ReflectionProbe[] reflectionProbes;

    void Start()
    {
        SetupSceneLighting();
        SetupReflectionProbes();
    }

    void SetupSceneLighting()
    {
        // Configure main directional light (simulating sun)
        if (mainLight == null)
        {
            // Create light if not provided
            GameObject lightObj = new GameObject("Main Light");
            mainLight = lightObj.AddComponent<Light>();
            mainLight.type = LightType.Directional;
        }

        mainLight.color = environmentColor;
        mainLight.intensity = 1.0f;
        mainLight.transform.rotation = Quaternion.Euler(50, -30, 0);
        mainLight.shadows = LightShadows.Soft;

        // Set ambient lighting
        RenderSettings.ambientLight = environmentColor * ambientIntensity;
        RenderSettings.ambientIntensity = ambientIntensity;
    }

    void SetupReflectionProbes()
    {
        foreach (ReflectionProbe probe in reflectionProbes)
        {
            // Configure and bake reflection probe
            probe.mode = UnityEngine.Rendering.ReflectionProbeMode.Realtime;
            probe.importance = 1;
            probe.refreshMode = UnityEngine.Rendering.ReflectionProbeRefreshMode.OnAwake;
        }
    }
}
```

#### Camera Setup for Robot Visualization

```csharp
// Robot visualization camera setup
using UnityEngine;

public class RobotVisualizationCamera : MonoBehaviour
{
    [Header("Camera Configuration")]
    public Transform robotTarget;
    public float followDistance = 5.0f;
    public float followHeight = 2.0f;
    public float smoothSpeed = 0.125f;

    [Header("Camera Settings")]
    public float fieldOfView = 60f;
    public float minDistance = 0.1f;
    public float maxDistance = 100f;

    private Vector3 offset;

    void Start()
    {
        if (robotTarget != null)
        {
            offset = transform.position - robotTarget.position;
        }

        // Configure camera settings
        Camera cam = GetComponent<Camera>();
        if (cam != null)
        {
            cam.fieldOfView = fieldOfView;
            cam.nearClipPlane = minDistance;
            cam.farClipPlane = maxDistance;
        }
    }

    void LateUpdate()
    {
        if (robotTarget != null)
        {
            Vector3 desiredPosition = robotTarget.position + offset;
            Vector3 smoothedPosition = Vector3.Lerp(transform.position, desiredPosition, smoothSpeed);
            transform.position = smoothedPosition;

            // Look at robot
            transform.LookAt(robotTarget);
        }
    }
}
```

## Integration Validation

### Visual Validation Checklist

1. **Model Integrity**: Verify all robot components are present and correctly positioned
2. **Material Application**: Check that materials are applied correctly with proper PBR properties
3. **Joint Configuration**: Ensure joints are properly mapped for animation
4. **Lighting Response**: Verify materials respond correctly to scene lighting
5. **Animation Functionality**: Test that animations play correctly
6. **Performance**: Monitor frame rate and rendering performance

### Testing Integration

#### Basic Functionality Test

```csharp
// Integration test script
using UnityEngine;
using System.Collections;

public class RobotIntegrationTest : MonoBehaviour
{
    public GameObject robotModel;
    public Material[] robotMaterials;
    public Animator robotAnimator;

    [Header("Test Results")]
    public bool modelLoaded = false;
    public bool materialsApplied = false;
    public bool animationWorking = false;
    public bool performanceAcceptable = true;

    void Start()
    {
        StartCoroutine(RunIntegrationTests());
    }

    IEnumerator RunIntegrationTests()
    {
        // Test 1: Model loaded
        yield return new WaitForSeconds(0.1f);
        modelLoaded = robotModel != null && robotModel.activeInHierarchy;
        Debug.Log($"Model loaded: {modelLoaded}");

        // Test 2: Materials applied
        yield return new WaitForSeconds(0.1f);
        materialsApplied = ValidateMaterials();
        Debug.Log($"Materials applied: {materialsApplied}");

        // Test 3: Animation working
        yield return new WaitForSeconds(0.1f);
        if (robotAnimator != null)
        {
            robotAnimator.Play("Idle", 0, 0f);
            animationWorking = true; // Animation system is responsive
        }
        Debug.Log($"Animation working: {animationWorking}");

        // Test 4: Performance check
        yield return new WaitForSeconds(1.0f);
        float frameTime = Time.unscaledDeltaTime;
        performanceAcceptable = (1.0f / frameTime) >= 30f; // Target 30 FPS
        Debug.Log($"Performance acceptable: {performanceAcceptable} ({1.0f/frameTime:F2} FPS)");

        // Final result
        bool allTestsPassed = modelLoaded && materialsApplied && animationWorking && performanceAcceptable;
        Debug.Log($"Integration test result: {(allTestsPassed ? "PASSED" : "FAILED")}");
    }

    bool ValidateMaterials()
    {
        if (robotMaterials == null || robotMaterials.Length == 0)
            return false;

        foreach (Material mat in robotMaterials)
        {
            if (mat == null)
                return false;
        }

        return true;
    }
}
```

## Advanced Integration Features

### Robot State Visualization

```csharp
// Advanced robot state visualization
using UnityEngine;

public class RobotStateVisualizer : MonoBehaviour
{
    [Header("State Indicators")]
    public Light[] statusLights;
    public Renderer[] componentRenderers;
    public Material activeMaterial;
    public Material inactiveMaterial;

    [Header("State Data")]
    public bool[] componentStates;
    public float[] componentValues; // For continuous values like joint angles

    void Update()
    {
        UpdateStatusLights();
        UpdateComponentMaterials();
        UpdateVisualFeedback();
    }

    void UpdateStatusLights()
    {
        for (int i = 0; i < statusLights.Length && i < componentStates.Length; i++)
        {
            statusLights[i].enabled = componentStates[i];
            // Color based on state (green for active, red for inactive)
            statusLights[i].color = componentStates[i] ? Color.green : Color.red;
        }
    }

    void UpdateComponentMaterials()
    {
        for (int i = 0; i < componentRenderers.Length && i < componentStates.Length; i++)
        {
            if (componentRenderers[i] != null && componentRenderers[i].material != null)
            {
                componentRenderers[i].material = componentStates[i] ? activeMaterial : inactiveMaterial;
            }
        }
    }

    void UpdateVisualFeedback()
    {
        // Update visual elements based on continuous values
        for (int i = 0; i < componentValues.Length; i++)
        {
            // Example: Update emission intensity based on component value
            if (i < componentRenderers.Length && componentRenderers[i] != null)
            {
                Material mat = componentRenderers[i].material;
                if (mat != null)
                {
                    mat.SetFloat("_EmissionScaleUI", componentValues[i]);
                }
            }
        }
    }
}
```

### LOD (Level of Detail) System

```csharp
// LOD system for robot models
using UnityEngine;

[RequireComponent(typeof(LODGroup))]
public class RobotLODController : MonoBehaviour
{
    [Header("LOD Configuration")]
    public float[] lodDistances = {10f, 30f, 60f};
    public GameObject[] lodMeshes;

    private LODGroup lodGroup;
    private LOD[] lods;

    void Start()
    {
        SetupLODSystem();
    }

    void SetupLODSystem()
    {
        lodGroup = GetComponent<LODGroup>();

        // Create LOD array
        lods = new LOD[lodDistances.Length];

        for (int i = 0; i < lodDistances.Length; i++)
        {
            // Create renderer array for this LOD level
            Renderer[] renderers = new Renderer[0];

            if (i < lodMeshes.Length && lodMeshes[i] != null)
            {
                renderers = lodMeshes[i].GetComponentsInChildren<Renderer>();
            }

            lods[i] = new LOD(lodDistances[i] / 100f, renderers); // Unity uses 0-1 scale
        }

        lodGroup.SetLODs(lods);
        lodGroup.RecalculateBounds();
    }
}
```

## Troubleshooting Common Integration Issues

### Model Import Issues

**Problem**: Robot model appears distorted or scaled incorrectly
**Solution**: Check import scale settings and ensure model units match Unity's meter-based system

**Problem**: Joints not mapping correctly for animation
**Solution**: Verify joint naming conventions and use Unity's Avatar configuration tool

### Material Issues

**Problem**: Materials appear flat or with incorrect lighting
**Solution**: Ensure materials use PBR shaders and have proper Metallic/Smoothness maps

**Problem**: Textures not appearing correctly
**Solution**: Check texture import settings and ensure proper UV mapping

### Performance Issues

**Problem**: Low frame rates with complex robot models
**Solution**: Implement LOD system, reduce polygon count, optimize materials

**Problem**: High memory usage
**Solution**: Compress textures, reduce material variants, use texture atlasing

## Best Practices

### Model Organization
- Keep robot components organized in a clear hierarchy
- Use consistent naming conventions
- Separate static and dynamic components

### Material Management
- Use material variants instead of duplicate materials
- Implement texture atlasing for better performance
- Use instancing where possible

### Performance Optimization
- Implement culling for off-screen robots
- Use occlusion culling in complex environments
- Optimize draw calls through batching

## Summary

Integrating humanoid robot models into Unity requires careful attention to model preparation, material configuration, and scene setup. By following the procedures outlined in this chapter, you can successfully import and configure robot models for high-fidelity visualization. The key steps include proper model import settings, material configuration with PBR properties, joint mapping for animation, and performance optimization through LOD systems and efficient rendering techniques.

The next chapter will cover specific rendering settings and techniques for achieving high-fidelity visualization in Unity robotics applications.