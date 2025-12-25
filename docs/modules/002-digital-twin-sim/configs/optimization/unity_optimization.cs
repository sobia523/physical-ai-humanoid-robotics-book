// Unity Performance Optimization Configuration
// This script demonstrates optimization strategies for Unity visualization
// in digital twin systems to maintain high frame rates and smooth performance

using UnityEngine;
using System.Collections.Generic;

[CreateAssetMenu(fileName = "UnityOptimizationConfig", menuName = "DigitalTwin/Optimization/Unity Config")]
public class UnityOptimizationConfig : ScriptableObject
{
    [Header("Rendering Optimization")]
    public bool useLOD = true;                    // Enable Level of Detail
    public int lodCount = 3;                     // Number of LOD levels
    public float lodTransitionSpeed = 1.0f;      // Speed of LOD transitions
    public bool enableOcclusionCulling = true;   // Use occlusion culling
    public bool enableFrustumCulling = true;     // Enable frustum culling (default)

    [Header("Quality Settings")]
    public int targetFrameRate = 60;             // Target frame rate
    public int maximumLODLevel = 1;              // Maximum LOD level to use
    public float lodBias = 0.5f;                 // LOD bias (0.5 = 50% quality)
    public ShadowQuality shadowQuality = ShadowQuality.Low;
    public TextureQuality textureQuality = TextureQuality.Medium;
    public AnisotropicFiltering anisotropicFiltering = AnisotropicFiltering.Enable;

    [Header("Shader Optimization")]
    public bool useMobileShaders = false;        // Use optimized shaders
    public bool enableDynamicBatching = true;    // Enable dynamic batching
    public bool enableStaticBatching = true;     // Enable static batching
    public int maximumLODForShadows = 1;         // LOD level for shadow rendering

    [Header("Lighting Optimization")]
    public bool useLightProbes = true;           // Use light probes instead of real-time lighting
    public bool useReflectionProbes = false;     // Disable reflection probes if not needed
    public bool enableRealtimeGI = false;        // Disable real-time global illumination
    public int mixedLightingMode = 1;            // Baked lighting mode

    [Header("Physics Optimization")]
    public int fixedTimestep = 15;               // Physics update rate (1/60 = 0.0167)
    public int maxSubSteps = 1;                  // Maximum physics substeps
    public bool enablePhysics = true;            // Enable physics simulation

    [Header("Audio Optimization")]
    public bool enableAudio = false;             // Disable audio in simulation if not needed
    public AudioSpeakerMode speakerMode = AudioSpeakerMode.Mono;

    [Header("Resource Management")]
    public bool enableObjectPooling = true;      // Use object pooling
    public int poolSize = 100;                   // Default pool size
    public bool enableAssetBundling = false;     // Use asset bundles if needed
    public bool enableAddressables = true;       // Use Addressable Asset System

    [Header("Network Optimization")]
    public float networkUpdateRate = 10.0f;      // Network update rate in Hz
    public int networkBufferSize = 1024;         // Network buffer size
    public bool enableCompression = true;        // Enable data compression
    public float networkTimeout = 5.0f;          // Network timeout in seconds
}

// Optimization Manager for Unity Digital Twin
public class UnityOptimizationManager : MonoBehaviour
{
    [SerializeField] private UnityOptimizationConfig config;
    [SerializeField] private float currentFrameRate;
    [SerializeField] private float averageFrameRate;

    private List<float> frameRateHistory = new List<float>();
    private const int FRAME_RATE_HISTORY_SIZE = 60; // 1 second at 60fps

    void Start()
    {
        ApplyOptimizationSettings();
        InitializePerformanceMonitoring();
    }

    void Update()
    {
        UpdatePerformanceMetrics();
        MonitorPerformance();
    }

    private void ApplyOptimizationSettings()
    {
        if (config == null) return;

        // Apply quality settings
        Application.targetFrameRate = config.targetFrameRate;
        QualitySettings.maximumLODLevel = config.maximumLODLevel;
        QualitySettings.lodBias = config.lodBias;
        QualitySettings.shadows = config.shadowQuality;
        QualitySettings.textureQuality = config.textureQuality;
        QualitySettings.anisotropicFiltering = config.anisotropicFiltering;

        // Apply physics settings
        Time.fixedDeltaTime = 1.0f / config.fixedTimestep;
        Time.maximumDeltaTime = 1.0f / 10.0f; // Maximum time per frame
        Physics.defaultSolverIterations = 6; // Reduce solver iterations
        Physics.defaultSolverVelocityIterations = 1; // Reduce velocity iterations

        // Apply batching settings
        Graphics.activeTier = GraphicsTier.Tier1; // Use mid-tier graphics
    }

    private void InitializePerformanceMonitoring()
    {
        frameRateHistory.Clear();
        for (int i = 0; i < FRAME_RATE_HISTORY_SIZE; i++)
        {
            frameRateHistory.Add(config.targetFrameRate);
        }
    }

    private void UpdatePerformanceMetrics()
    {
        currentFrameRate = 1.0f / Time.unscaledDeltaTime;

        // Update frame rate history
        frameRateHistory.RemoveAt(0);
        frameRateHistory.Add(currentFrameRate);

        // Calculate average frame rate
        float sum = 0;
        foreach (float rate in frameRateHistory)
        {
            sum += rate;
        }
        averageFrameRate = sum / frameRateHistory.Count;
    }

    private void MonitorPerformance()
    {
        // Adjust settings based on performance
        if (averageFrameRate < config.targetFrameRate * 0.8f)
        {
            // Performance is low, reduce quality
            ReduceQualitySettings();
        }
        else if (averageFrameRate > config.targetFrameRate * 0.95f)
        {
            // Performance is good, can increase quality
            IncreaseQualitySettings();
        }
    }

    private void ReduceQualitySettings()
    {
        if (config == null) return;

        // Temporarily reduce quality settings
        QualitySettings.maximumLODLevel = Mathf.Max(0, config.maximumLODLevel - 1);
        QualitySettings.lodBias = Mathf.Max(0.25f, config.lodBias * 0.8f);

        // Reduce shadow distance
        if (Light.main != null)
        {
            Light.main.shadowDistance = Mathf.Max(50f, Light.main.shadowDistance * 0.8f);
        }
    }

    private void IncreaseQualitySettings()
    {
        if (config == null) return;

        // Gradually increase quality settings
        QualitySettings.maximumLODLevel = Mathf.Min(2, config.maximumLODLevel + 1);
        QualitySettings.lodBias = Mathf.Min(1.0f, config.lodBias * 1.1f);

        // Increase shadow distance
        if (Light.main != null)
        {
            Light.main.shadowDistance = Mathf.Min(150f, Light.main.shadowDistance * 1.1f);
        }
    }

    // Method to handle ROS-TCP connection optimization
    public void OptimizeROSConnection(float currentLatency)
    {
        if (config == null) return;

        // Adjust update rate based on network latency
        float adjustedRate = Mathf.Clamp(
            config.networkUpdateRate * (1.0f - (currentLatency / 1000.0f)),
            config.networkUpdateRate * 0.5f,
            config.networkUpdateRate
        );

        // Use adjustedRate for ROS message publishing frequency
    }

    // Method to optimize visualization based on simulation complexity
    public void OptimizeVisualization(int objectCount)
    {
        if (config == null) return;

        // Adjust visualization quality based on number of objects
        if (objectCount > 100)
        {
            // Reduce quality for many objects
            QualitySettings.maximumLODLevel = 0;
            QualitySettings.lodBias = 0.5f;
        }
        else if (objectCount > 50)
        {
            // Medium quality for moderate number of objects
            QualitySettings.maximumLODLevel = 1;
            QualitySettings.lodBias = 0.75f;
        }
        else
        {
            // High quality for few objects
            QualitySettings.maximumLODLevel = config.maximumLODLevel;
            QualitySettings.lodBias = config.lodBias;
        }
    }
}