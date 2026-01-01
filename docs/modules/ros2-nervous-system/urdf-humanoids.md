---
sidebar_position: 4
---

# Humanoid Structure using URDF: Links, Joints, and Frames

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the structure of URDF (Unified Robot Description Format) files
- Identify links, joints, and frames in a humanoid robot description
- Explain why URDF is required for humanoid robots
- Create basic URDF elements for humanoid robot components
- Understand how AI agents interact with URDF-defined robot structures

## Introduction

URDF (Unified Robot Description Format) is an XML-based format used in ROS to describe robot models. For humanoid robots, URDF is essential as it defines the physical structure, kinematic properties, and visual appearance of the robot. This chapter explains how URDF enables AI agents to understand and interact with humanoid robot structures.

## Understanding URDF Structure

URDF is an XML format that describes a robot as a collection of links connected by joints. The structure forms a tree (or multiple trees for robots with multiple end-effectors), with each link having properties like mass, visual appearance, and collision properties.

### Basic URDF Elements

- **Links**: Rigid bodies with physical properties
- **Joints**: Connections between links that allow motion
- **Materials**: Visual properties for rendering
- **Gazebo tags**: Simulation-specific properties

## Links: The Building Blocks of Robots

**Links** represent rigid bodies in the robot. Each link has physical properties and can have visual and collision representations.

### Link Structure

```xml
<link name="link_name">
  <inertial>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <mass value="1"/>
    <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
  </inertial>
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <cylinder length="1" radius="0.1"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <cylinder length="1" radius="0.1"/>
    </geometry>
  </collision>
</link>
```

### Key Link Properties:
- **inertial**: Mass, center of mass, and inertia tensor
- **visual**: How the link appears in visualization
- **collision**: How the link interacts in collision detection

## Joints: Connecting the Robot

**Joints** define the connection between links and specify the allowed motion. Different joint types enable different kinds of movement.

### Joint Types

1. **Revolute**: Rotational joint with limits
2. **Continuous**: Rotational joint without limits
3. **Prismatic**: Linear sliding joint with limits
4. **Fixed**: No motion allowed (rigid connection)
5. **Floating**: 6 DOF motion (rarely used)
6. **Planar**: Motion on a plane

### Joint Structure

```xml
<joint name="joint_name" type="revolute">
  <parent link="parent_link_name"/>
  <child link="child_link_name"/>
  <origin xyz="0 0 1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
</joint>
```

### Key Joint Properties:
- **parent/child**: Links that the joint connects
- **origin**: Position and orientation of the joint
- **axis**: Axis of rotation or translation
- **limit**: Motion constraints (for revolute/prismatic joints)

## Frames: Coordinate Systems in URDF

**Frames** in URDF represent coordinate systems attached to links. They are essential for:
- Spatial relationships between robot parts
- Sensor mounting positions
- End-effector poses
- Interaction with AI agents

### TF (Transform) Frames

URDF automatically creates TF frames for each link, allowing the robot's state to be represented in a tree of coordinate transformations. This is crucial for AI agents that need to understand spatial relationships.

## Complete Humanoid Robot Example

Here's a simplified example of a humanoid robot's leg structure in URDF:

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
  </link>

  <!-- Hip joint and thigh -->
  <joint name="hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="thigh_link"/>
    <origin xyz="0 0 -0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="thigh_link">
    <visual>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>

  <!-- Knee joint and shin -->
  <joint name="knee_joint" type="revolute">
    <parent link="thigh_link"/>
    <child link="shin_link"/>
    <origin xyz="0 0 -0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.35" effort="100" velocity="1"/>
  </joint>

  <link name="shin_link">
    <visual>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
  </link>

  <!-- Ankle joint and foot -->
  <joint name="ankle_joint" type="revolute">
    <parent link="shin_link"/>
    <child link="foot_link"/>
    <origin xyz="0 0 -0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.78" upper="0.78" effort="100" velocity="1"/>
  </joint>

  <link name="foot_link">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
  </link>
</robot>
```

## Why URDF is Required for Humanoids

URDF is essential for humanoid robots for several reasons:

### 1. Kinematic Modeling
- AI agents need to understand the robot's structure to plan movements
- Forward and inverse kinematics calculations require accurate joint relationships
- Understanding reachability and workspace constraints

### 2. Collision Avoidance
- AI agents must avoid self-collisions and environmental collisions
- URDF provides collision geometry for each link
- Planning algorithms use this information to find safe paths

### 3. Sensor Integration
- Sensors are mounted relative to specific links
- URDF defines the transformation between sensor frames and robot base
- AI agents use this to interpret sensor data in the robot's context

### 4. Control Architecture
- Joint position, velocity, and effort limits are defined in URDF
- AI agents must respect these physical constraints
- Understanding the robot's degrees of freedom

## How AI Agents Interact with URDF

### 1. Motion Planning
AI agents use URDF information to:
- Plan collision-free trajectories
- Understand joint limits and constraints
- Calculate inverse kinematics for end-effector positioning

### 2. State Estimation
AI agents interpret sensor data using:
- Kinematic chain defined by URDF
- Joint angle measurements
- IMU data in the context of robot frames

### 3. Control Commands
AI agents generate commands considering:
- Joint types and ranges
- Physical limits defined in URDF
- Desired end-effector poses in robot frames

## URDF Tools and Visualization

Several ROS tools help work with URDF:

- **robot_state_publisher**: Publishes joint states as TF transforms
- **joint_state_publisher**: Publishes joint state messages
- **rviz**: Visualizes URDF models
- **urdf_parser**: Parses and validates URDF files

## Common URDF Patterns for Humanoids

### 1. Serial Chain Structure
- Limbs are typically modeled as serial chains of joints
- Each chain has a base and an end-effector
- Easy to calculate forward/inverse kinematics

### 2. Fixed Base vs. Floating Base
- Fixed base: Robot is anchored (e.g., industrial manipulator)
- Floating base: Robot can move in space (humanoid robots)
- Floating base requires additional considerations for balance

### 3. Transmission Elements
- Define how actuators connect to joints
- Include gear ratios, mechanical properties
- Important for accurate control simulation

## Summary

URDF is fundamental to humanoid robotics as it provides the structural description that AI agents need to interact with the physical robot. Key concepts:

- **Links** represent rigid bodies with physical properties
- **Joints** define connections and allowed motions between links
- **Frames** provide coordinate systems for spatial reasoning
- URDF enables AI agents to understand robot kinematics, avoid collisions, and plan safe movements
- Proper URDF modeling is essential for effective AI-robot integration

Understanding URDF is crucial for AI developers working with humanoid robots, as it forms the bridge between abstract AI algorithms and the physical robot's structure and capabilities.