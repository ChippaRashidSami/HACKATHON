---
sidebar_position: 2
title: "High-Fidelity Interaction in Unity"
---

# High-Fidelity Interaction in Unity

## Learning Objectives

After completing this chapter, you will be able to:
- Understand Unity's rendering pipeline and how it complements physics simulation
- Set up Unity for high-fidelity visualization of humanoid robots
- Create interactive environments for humanoid robot testing
- Implement Unity ↔ ROS 2 communication for robot control
- Understand the role of high-fidelity rendering in digital twin applications

## Introduction

While Gazebo provides excellent physics simulation capabilities, Unity offers high-fidelity rendering and interactive environments that are essential for creating realistic digital twins of robotic systems. The combination of Gazebo's accurate physics and Unity's visual realism provides a comprehensive simulation environment for humanoid robots.

In this chapter, we'll explore how to use Unity for creating high-fidelity visualizations and interactive environments for humanoid robots, with integration to ROS 2 for complete simulation capabilities.

## Unity in Robotics Context

Unity is a powerful game engine that's increasingly being adopted in robotics for:

- **High-Fidelity Visualization**: Photorealistic rendering capabilities
- **Interactive Environments**: User-controlled environments for testing human-robot interaction
- **VR/AR Integration**: Immersive interfaces for robot teleoperation
- **Digital Twin Applications**: Accurate visual representation of physical systems

### Unity vs Gazebo

While Gazebo focuses on physics accuracy and sensor simulation, Unity focuses on:
- Visual realism (shaders, lighting, materials)
- User interaction (mouse, keyboard, VR controllers)
- Complex scene management
- Asset integration and animation

## Setting Up Unity for Robotics

### Unity Robotics Package

The Unity Robotics package provides essential tools for robotics simulation:

1. **ROS-TCP-Connector**: Communication between Unity and ROS
2. **URDF-Importer**: Importing robot models from URDF files
3. **Robot-Builder**: Tools for creating robot models in Unity

### Installation Process

1. Create a new Unity 3D project
2. Install the Unity Robotics Hub
3. Import the ROS-TCP-Connector and URDF-Importer packages
4. Set up the ROS connection

## Unity Rendering Pipeline

Unity's rendering pipeline is critical to understand for high-fidelity visualization:

### Universal Render Pipeline (URP)

For efficient real-time rendering:

```csharp
// Example: Setting up URP in a Unity script
using UnityEngine;
using UnityEngine.Rendering.Universal;

public class RobotRenderSetup : MonoBehaviour
{
    void Start()
    {
        // Configure rendering settings for high-quality robot visualization
        var rendererData = GetComponent<UniversalRendererData>();
        // Set up quality settings appropriate for robot visualization
    }
}
```

### Lighting and Materials

For photorealistic robot visualization:

```csharp
// Example: Setting up materials for robot parts
public class RobotMaterialSetup : MonoBehaviour
{
    public Material metalMat;
    public Material rubberMat;
    public Material glassMat;

    void Start()
    {
        // Apply appropriate materials to robot parts
        Renderer[] renderers = GetComponentsInChildren<Renderer>();
        
        foreach (Renderer r in renderers)
        {
            if (r.name.Contains("joint") || r.name.Contains("frame"))
            {
                r.material = metalMat;
            }
            else if (r.name.Contains("foot") || r.name.Contains("hand"))
            {
                r.material = rubberMat;
            }
            else if (r.name.Contains("camera") || r.name.Contains("lidar"))
            {
                r.material = glassMat;
            }
        }
    }
}
```

## Importing Robot Models from URDF

The URDF-Importer package allows us to bring ROS robot models into Unity:

### Import Process

1. Copy your URDF file to Unity's `Assets` folder
2. Create a new GameObject and add the URDF-Importer component
3. Configure joint limits and visual settings
4. The robot will be imported with proper kinematics

### Custom URDF Processing

```csharp
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.URDF;
using UnityEngine.Rendering;

public class CustomURDFProcessor : MonoBehaviour
{
    public string urdfPath;
    
    void Start()
    {
        // Load and process URDF with custom settings
        StartCoroutine(LoadURDF(urdfPath));
    }

    IEnumerator LoadURDF(string path)
    {
        var urdfRobot = URDFLoader.LoadFromPath(path);
        
        // Customize imported robot
        ConfigureRobotMaterials(urdfRobot);
        SetupJointControllers(urdfRobot);
        
        yield return null;
    }

    void ConfigureRobotMaterials(GameObject robot)
    {
        // Apply specific materials for high-fidelity visualization
        var links = robot.GetComponentsInChildren<Link>();
        foreach (var link in links)
        {
            // Configure each link's visual properties
            ConfigureLinkVisuals(link);
        }
    }

    void ConfigureLinkVisuals(Link link)
    {
        // Set up PBR materials for realistic appearance
        var renderer = link.GetComponent<Renderer>();
        if (renderer != null)
        {
            renderer.material.SetColor("_BaseColor", Color.gray);
            renderer.material.SetFloat("_Metallic", 0.8f);
            renderer.material.SetFloat("_Smoothness", 0.6f);
        }
    }

    void SetupJointControllers(GameObject robot)
    {
        // Add joint controllers for Unity-side control
        var joints = robot.GetComponentsInChildren<Joint>();
        foreach (var joint in joints)
        {
            // Add Unity joint controllers
            joint.gameObject.AddComponent<UnityJointController>();
        }
    }
}
```

## Creating Interactive Environments

Unity excels at creating interactive environments for humanoid robot testing:

### Environment Setup

```csharp
using UnityEngine;

public class InteractiveEnvironment : MonoBehaviour
{
    public GameObject humanoidRobot;
    public Transform spawnPoint;
    public List<Obstacle> obstacles;
    public HumanoidInputController inputController;

    void Start()
    {
        // Spawn robot at designated location
        Instantiate(humanoidRobot, spawnPoint.position, spawnPoint.rotation);
        
        // Set up interactive elements
        SetupInteractiveElements();
    }

    void SetupInteractiveElements()
    {
        // Add interactive objects that the robot can manipulate
        foreach (var obstacle in obstacles)
        {
            var interactiveObj = obstacle.gameObject.AddComponent<InteractiveObject>();
            interactiveObj.onInteract += HandleInteraction;
        }
    }

    void HandleInteraction(GameObject obj)
    {
        // Handle interaction between robot and environment
        Debug.Log($"Robot interacted with {obj.name}");
    }
}
```

### Physics Materials

For realistic interaction physics:

```csharp
using UnityEngine;

[CreateAssetMenu(fileName = "PhysicsMaterial", menuName = "Robotics/Physics Material")]
public class RobotPhysicsMaterial : ScriptableObject
{
    public PhysicMaterial material;
    
    [Header("Robot-Specific Properties")]
    public float footFriction = 0.8f;
    public float handGrip = 0.9f;
    public float collisionResponse = 0.2f;
    
    public PhysicMaterial CreateForRobotPart(string partName)
    {
        var newMaterial = new PhysicMaterial(partName);
        newMaterial.staticFriction = GetStaticFriction(partName);
        newMaterial.dynamicFriction = GetDynamicFriction(partName);
        newMaterial.bounciness = GetBounciness(partName);
        return newMaterial;
    }

    float GetStaticFriction(string partName)
    {
        if (partName.Contains("foot")) return footFriction;
        if (partName.Contains("hand")) return handGrip;
        return 0.5f;
    }

    float GetDynamicFriction(string partName)
    {
        return GetStaticFriction(partName) * 0.8f;
    }

    float GetBounciness(string partName)
    {
        if (partName.Contains("head")) return 0.1f;
        return 0.05f;
    }
}
```

## Unity ↔ ROS 2 Communication

The communication between Unity and ROS 2 enables:
- Robot state visualization in Unity
- Command execution from Unity
- Sensor data visualization
- Shared simulation state

### ROS-TCP-Connector Implementation

```csharp
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;

public class UnityROSConnector : MonoBehaviour
{
    ROSConnection ros;
    string rosIP = "127.0.0.1";
    int rosPort = 10000;

    // Robot joint positions
    public float[] jointPositions = new float[20]; // Example: 20 DOF humanoid
    
    void Start()
    {
        // Get ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.Initialize(rosIP, rosPort);
        
        // Start listening to ROS topics
        ros.Subscribe<JointStateMsg>("/joint_states", JointStateCallback);
        
        // Start coroutine to publish commands
        StartCoroutine(PublishJointCommands());
    }

    void JointStateCallback(JointStateMsg msg)
    {
        // Update joint positions based on ROS message
        for (int i = 0; i < msg.position.Length && i < jointPositions.Length; i++)
        {
            jointPositions[i] = (float)msg.position[i];
        }
        
        // Update Unity robot model
        UpdateRobotModel();
    }

    void UpdateRobotModel()
    {
        // Apply joint positions to Unity humanoid model
        var robotParts = GetComponentsInChildren<JointController>();
        for (int i = 0; i < robotParts.Length && i < jointPositions.Length; i++)
        {
            robotParts[i].SetJointPosition(jointPositions[i]);
        }
    }

    IEnumerator PublishJointCommands()
    {
        var jointCmdMsg = new JointStateMsg();
        jointCmdMsg.name = new string[] { 
            "left_hip_joint", "left_knee_joint", "left_ankle_joint",
            "right_hip_joint", "right_knee_joint", "right_ankle_joint",
            // Add more joint names as needed
        };

        while (ros != null)
        {
            // Prepare command message
            jointCmdMsg.position = new double[jointCmdMsg.name.Length];
            
            // Fill with desired positions (this could come from AI, input, etc.)
            for (int i = 0; i < jointCmdMsg.position.Length; i++)
            {
                jointCmdMsg.position[i] = GetDesiredPosition(i); // Implement this method
            }

            // Publish command
            ros.Publish("/joint_commands", jointCmdMsg);

            yield return new WaitForSeconds(0.1f); // 10Hz update rate
        }
    }

    double GetDesiredPosition(int jointIndex)
    {
        // This method would determine desired joint positions
        // Could come from AI controller, motion planner, etc.
        return 0.0; // Placeholder
    }
}
```

## High-Fidelity Rendering Techniques

### Realistic Materials

For photorealistic robot visualization:

```csharp
using UnityEngine;

public class RobotMaterialManager : MonoBehaviour
{
    [Header("Material Settings")]
    public float metalness = 0.9f;
    public float smoothness = 0.7f;
    public Color baseColor = Color.gray;
    
    [Header("Emission Settings")]
    public bool hasEmission = false;
    public Color emissionColor = Color.red;
    public float emissionIntensity = 1.0f;

    void Start()
    {
        ApplyMaterialsToRobot();
    }

    void ApplyMaterialsToRobot()
    {
        var renderers = GetComponentsInChildren<Renderer>();
        
        foreach (var renderer in renderers)
        {
            var material = renderer.material;

            // Configure PBR properties
            material.SetColor("_BaseColor", baseColor);
            material.SetFloat("_Metallic", metalness);
            material.SetFloat("_Smoothness", smoothness);

            // Configure emission if needed
            if (hasEmission)
            {
                material.EnableKeyword("_EMISSION");
                material.SetColor("_EmissionColor", emissionColor * emissionIntensity);
            }
        }
    }
}
```

### Lighting Setup

```csharp
using UnityEngine;

public class RobotLightingSetup : MonoBehaviour
{
    public Light mainLight;
    public float intensity = 1.0f;
    public Color color = Color.white;
    public float shadowStrength = 0.8f;

    void Start()
    {
        ConfigureLighting();
    }

    void ConfigureLighting()
    {
        if (mainLight != null)
        {
            mainLight.intensity = intensity;
            mainLight.color = color;
            mainLight.shadows = LightShadows.Soft;
            mainLight.shadowStrength = shadowStrength;
            
            // Additional lighting setup for optimal robot visualization
            mainLight.cookieSize = 10f;
        }
        
        // Add ambient lighting
        RenderSettings.ambientLight = new Color(0.2f, 0.2f, 0.2f, 1);
        RenderSettings.ambientIntensity = 0.5f;
    }
}
```

## VR Integration for Human-Robot Interaction

Unity's VR capabilities enable immersive human-robot interaction:

```csharp
using UnityEngine;
using UnityEngine.XR;

public class VRHumanoidController : MonoBehaviour
{
    public GameObject humanoidRobot;
    public Transform vrHeadset;
    public Transform leftController;
    public Transform rightController;
    
    void Update()
    {
        if (IsVRActive())
        {
            HandleVRControls();
        }
    }

    bool IsVRActive()
    {
        return XRSettings.isDeviceActive;
    }

    void HandleVRControls()
    {
        // Use VR controllers to manipulate robot
        if (leftController != null)
        {
            // Map left controller to left arm of robot
            MoveRobotArm("left", leftController.position, leftController.rotation);
        }
        
        if (rightController != null)
        {
            // Map right controller to right arm of robot
            MoveRobotArm("right", rightController.position, rightController.rotation);
        }
    }

    void MoveRobotArm(string arm, Vector3 position, Quaternion rotation)
    {
        // Calculate inverse kinematics to move robot's arm
        // This would implement the actual arm movement logic
    }
}
```

## Diagram: Unity-ROS 2 Integration Architecture

```
              Unity-ROS 2 Integration Architecture
                            
    [Unity Scene] ←→ [ROS-TCP-Connector] ←→ [ROS 2 Network]
         |                   |                      |
    [Robot Model]      [Message]           [ROS Nodes]
    [Environment]      [Serialization]     [Controllers]
    [Visuals]          [Protocol]          [Sensors]
         |                   |                      |
    [User Input] ←→ [Data Sync] ←→ [Robot States]
    [VR Devices]     [Commands]      [Sensor Data]
```

## Practical Implementation: Unity Humanoid Environment

Here's a complete example of setting up an interactive humanoid environment:

```csharp
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;

public class HumanoidEnvironment : MonoBehaviour
{
    [Header("Robot Configuration")]
    public GameObject humanoidPrefab;
    public Transform spawnPoint;
    
    [Header("Environment Configuration")]
    public List<GameObject> interactiveObjects;
    public Vector3 environmentBounds = new Vector3(10, 10, 10);
    
    [Header("ROS Configuration")]
    public string rosIP = "127.0.0.1";
    public int rosPort = 10000;
    
    private GameObject instantiatedRobot;
    private ROSConnection ros;
    private List<Rigidbody> robotParts = new List<Rigidbody>();

    void Start()
    {
        InitializeEnvironment();
        InitializeROSConnection();
        SpawnRobot();
        SetupInteractiveEnvironment();
    }

    void InitializeEnvironment()
    {
        // Set up physics properties for the environment
        Physics.defaultSolverIterations = 10;
        Physics.defaultSolverVelocityIterations = 8;
    }

    void InitializeROSConnection()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Initialize(rosIP, rosPort);
        
        // Subscribe to relevant topics
        ros.Subscribe<JointStateMsg>("/joint_states", OnJointStatesReceived);
        ros.Subscribe<OdometryMsg>("/odom", OnOdometryReceived);
    }

    void SpawnRobot()
    {
        instantiatedRobot = Instantiate(humanoidPrefab, spawnPoint.position, spawnPoint.rotation);
        
        // Collect all rigidbodies for physics control
        var rigidbodies = instantiatedRobot.GetComponentsInChildren<Rigidbody>();
        robotParts.AddRange(rigidbodies);
    }

    void OnJointStatesReceived(JointStateMsg msg)
    {
        // Update Unity robot model based on ROS joint states
        UpdateRobotModel(msg);
    }

    void OnOdometryReceived(OdometryMsg msg)
    {
        // Update robot position based on odometry
        var position = new Vector3(
            (float)msg.pose.pose.position.x,
            (float)msg.pose.pose.position.y,
            (float)msg.pose.pose.position.z
        );
        
        instantiatedRobot.transform.position = position;
    }

    void SetupInteractiveEnvironment()
    {
        // Add interactive elements to the scene
        foreach (var obj in interactiveObjects)
        {
            var interObj = obj.AddComponent<InteractiveObject>();
            interObj.onInteract += OnObjectInteracted;
        }
    }

    void OnObjectInteracted(GameObject obj)
    {
        Debug.Log($"Robot interacted with: {obj.name}");
        
        // Send ROS message about the interaction
        var msg = new RosMessageTypes.Std.StringMsg();
        msg.data = $"Robot interacted with {obj.name}";
        ros.Publish("/interaction_log", msg);
    }

    void UpdateRobotModel(JointStateMsg jointState)
    {
        // Apply joint positions to Unity model
        // This implementation depends on your specific robot structure
    }
}
```

## Performance Optimization

For smooth operation with complex humanoid models:

```csharp
using UnityEngine;

public class RobotPerformanceOptimizer : MonoBehaviour
{
    [Header("LOD Settings")]
    public float lodDistance = 10f;
    public int lodLevel = 0;
    
    [Header("Physics Settings")]
    public float physicsUpdateRate = 60f;
    public int solverIterations = 8;
    
    void Start()
    {
        OptimizeForRobotSimulation();
    }

    void OptimizeForRobotSimulation()
    {
        // Optimize physics settings
        Time.fixedDeltaTime = 1f / physicsUpdateRate;
        Physics.defaultSolverIterations = solverIterations;
        
        // Reduce rendering load for distant robots
        SetupLODForRobot();
    }

    void SetupLODForRobot()
    {
        // Set up Level of Detail for robot model
        var lods = new LOD[3];
        var rendererGroups = GetComponentsInChildren<Renderer>();
        
        // Create LODs with decreasing detail
        for (int i = 0; i < lods.Length; i++)
        {
            float screenPercentage = 0.5f / (i + 1);
            lods[i] = new LOD(screenPercentage, rendererGroups);
        }
        
        var lodGroup = gameObject.AddComponent<LODGroup>();
        lodGroup.SetLODs(lods);
        lodGroup.RecalculateBounds();
    }
}
```

## Summary

Unity provides high-fidelity visualization capabilities that complement the physics simulation of Gazebo. By integrating Unity with ROS 2, we can create comprehensive digital twin environments for humanoid robots with realistic rendering and interactive capabilities.

Key concepts include:
- Importing robot models from URDF files
- Setting up Unity ↔ ROS 2 communication
- Creating interactive environments for testing
- Optimizing rendering for real-time performance
- Implementing VR integration for immersive interaction

## Exercises

### Exercise 1: Import Your Robot
Use the URDF-Importer to import your humanoid robot model into Unity and configure appropriate materials.

### Exercise 2: Create Interactive Environment
Design a Unity scene with interactive objects that your humanoid robot can manipulate.

### Exercise 3: ROS Integration
Implement Unity ↔ ROS 2 communication to visualize robot state and send commands from Unity.

## Next Steps

In the next chapter, we'll explore how to simulate various sensors (LiDAR, Depth Cameras, IMUs) in both Gazebo and Unity environments, and how to visualize this sensor data in RViz for comprehensive perception system development.