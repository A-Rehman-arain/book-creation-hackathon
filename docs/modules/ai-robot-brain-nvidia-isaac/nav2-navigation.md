---
sidebar_position: 6
title: Nav2 for humanoid navigation and path planning
---

# Nav2 for humanoid navigation and path planning

Navigation2 (Nav2) is the latest navigation stack for ROS 2, designed for mobile robots with advanced capabilities for path planning, path execution, and obstacle avoidance. For humanoid robots, Nav2 requires specific configuration and adaptation to handle the unique challenges of bipedal or multi-legged locomotion.

## Learning Objectives

After completing this chapter, you will understand:
- The core concepts of Navigation2 (Nav2) and its architecture
- How to configure Nav2 for humanoid robot navigation
- The path planning algorithms available in Nav2
- How to adapt Nav2 for humanoid-specific navigation requirements

## Introduction to Navigation2 (Nav2)

Nav2 is a complete redesign of the ROS navigation stack, built from the ground up for ROS 2. It provides:

- **Behavior Trees**: Flexible and configurable navigation behaviors
- **Advanced Path Planning**: State-of-the-art path planning algorithms
- **Recovery Behaviors**: Robust handling of navigation failures
- **Plugin Architecture**: Extensible components for customization

### Key Improvements Over Navigation1

1. **ROS 2 Native**: Built specifically for ROS 2 architecture
2. **Behavior Trees**: More flexible and configurable navigation logic
3. **Improved Safety**: Better obstacle avoidance and recovery mechanisms
4. **Modern Algorithms**: Latest path planning and control algorithms

## Nav2 Architecture

### Core Components

Nav2 consists of several core components that work together:

#### Global Planner
- Computes the overall path from start to goal
- Uses costmaps to avoid known obstacles
- Supports various path planning algorithms (A*, Dijkstra, NavFn, etc.)

#### Local Planner
- Executes the path while avoiding dynamic obstacles
- Controls robot motion in real-time
- Implements trajectory generation and control

#### Costmap 2D
- Maintains maps of obstacles and free space
- Combines static map data with sensor information
- Provides cost information for planning algorithms

#### Controller Server
- Manages the execution of navigation tasks
- Coordinates between global and local planners
- Handles recovery behaviors when needed

### Behavior Tree Framework

Nav2 uses behavior trees to define navigation behaviors:

```xml
<!-- Example Nav2 behavior tree -->
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <PipelineSequence name="NavigateWithReplanning">
      <RateController hz="1.0">
        <RecoveryNode number_of_retries="6" name="ClearGlobalCostmapRecovery">
          <ClearCostmapService service_name="clear_costmap_global"/>
          <Sequence name="LocalPlanUntilPathInvalid">
            <GoalUpdated/>
            <RecoveryNode number_of_retries="1" name="ComputePathToPose">
              <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
            </RecoveryNode>
            <RecoveryNode number_of_retries="1" name="FollowPath">
              <FollowPath path="{path}" controller_id="FollowPath"/>
            </RecoveryNode>
          </Sequence>
        </RecoveryNode>
      </RateController>
    </PipelineSequence>
  </BehaviorTree>
</root>
```

## Path Planning Algorithms

### Global Path Planners

#### A* (A-star)
- Optimal path planning algorithm
- Guarantees shortest path in static environments
- Good for humanoid navigation with proper costmap configuration

#### Dijkstra
- Similar to A* but without heuristic
- More computationally expensive but still optimal
- Useful when heuristic function is difficult to define

#### NavFn
- Fast marching method for path planning
- Good performance for large environments
- May not always find optimal path

### Local Path Planners

#### DWB (Dynamic Window Approach)
- Real-time trajectory generation
- Considers robot dynamics and constraints
- Well-suited for humanoid robots with complex kinematics

#### TEB (Timed Elastic Band)
- Optimizes trajectories considering time and constraints
- Handles robot footprint and kinematic constraints
- Good for precise humanoid navigation

## Humanoid-Specific Navigation Configuration

### Costmap Configuration for Humanoid Robots

Humanoid robots require special consideration in costmap configuration:

```yaml
# Example costmap configuration for humanoid robot
local_costmap:
  global_frame: odom
  robot_base_frame: base_link
  update_frequency: 10.0
  publish_frequency: 10.0
  width: 10
  height: 10
  resolution: 0.05
  robot_radius: 0.4  # Adjust based on humanoid dimensions
  plugins:
    - {name: static_layer, type: "nav2_costmap_2d::StaticLayer"}
    - {name: obstacle_layer, type: "nav2_costmap_2d::ObstacleLayer"}
    - {name: inflation_layer, type: "nav2_costmap_2d::InflationLayer"}

global_costmap:
  global_frame: map
  robot_base_frame: base_link
  update_frequency: 1.0
  static_map: true
  robot_radius: 0.4  # Same as local costmap
  plugins:
    - {name: static_layer, type: "nav2_costmap_2d::StaticLayer"}
    - {name: obstacle_layer, type: "nav2_costmap_2d::ObstacleLayer"}
    - {name: inflation_layer, type: "nav2_costmap_2d::InflationLayer"}
```

### Controller Configuration for Humanoid Robots

Humanoid robots require controllers that account for their unique locomotion:

```yaml
# Example controller configuration for humanoid robot
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # DWB Controller
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: true
      min_vel_x: 0.0
      min_vel_y: -0.1
      max_vel_x: 0.5
      max_vel_y: 0.1
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.5
      min_speed_theta: 0.0
      acc_lim_x: 2.5
      acc_lim_y: 2.5
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_y: -2.5
      decel_lim_theta: -3.2
      vx_samples: 20
      vy_samples: 5
      vtheta_samples: 20
      sim_time: 1.7
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.2
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.15
      stateful: true
      restore_defaults: false
```

## Humanoid Navigation Challenges

### Balance and Stability

Humanoid robots face unique challenges in navigation:

#### Center of Mass Management
- Path planning must consider balance constraints
- Avoid sharp turns that could destabilize the robot
- Plan paths that maintain stable locomotion

#### Footstep Planning
- Integrate footstep planning with navigation
- Consider terrain for safe foot placement
- Plan for dynamic obstacles in 3D space

### Kinematic Constraints

#### Degrees of Freedom
- Account for complex joint limitations
- Plan paths that respect joint angle constraints
- Consider whole-body motion planning

#### Gait Adaptation
- Adjust navigation parameters based on gait type
- Handle transitions between different locomotion modes
- Adapt to terrain variations

## Practical Example: Nav2 Configuration for Humanoid Robot

Here's a complete example of Nav2 configuration tailored for humanoid navigation:

```yaml
# Complete Nav2 configuration for humanoid robot
bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: true
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    default_nav_through_poses_bt_xml: "navigate_w_replanning_and_recovery.xml"
    default_nav_to_pose_bt_xml: "navigate_w_replanning_and_recovery.xml"
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_compute_path_through_poses_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_is_stuck_condition_bt_node
      - nav2_goal_reached_condition_bt_node
      - nav2_goal_updated_condition_bt_node
      - nav2_initial_pose_received_condition_bt_node
      - nav2_reinitialize_global_localization_service_bt_node
      - nav2_rate_controller_bt_node
      - nav2_distance_controller_bt_node
      - nav2_speed_controller_bt_node
      - nav2_truncate_path_action_bt_node
      - nav2_truncate_path_local_action_bt_node
      - nav2_goal_updater_node_bt_node
      - nav2_recovery_node_bt_node
      - nav2_pipeline_sequence_bt_node
      - nav2_round_robin_node_bt_node
      - nav2_transform_available_condition_bt_node
      - nav2_time_expired_condition_bt_node
      - nav2_path_expiring_timer_condition
      - nav2_distance_traveled_condition_bt_node
      - nav2_single_trigger_bt_node
      - nav2_is_battery_low_condition_bt_node
      - nav2_navigate_through_poses_action_bt_node
      - nav2_navigate_to_pose_action_bt_node
      - nav2_remove_passed_goals_action_bt_node
      - nav2_planner_selector_bt_node
      - nav2_controller_selector_bt_node
      - nav2_goal_checker_selector_bt_node
      - nav2_controller_cancel_bt_node
      - nav2_path_longer_on_approach_bt_node
      - nav2_wait_cancel_bt_node

controller_server:
  ros__parameters:
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Humanoid-specific controller configuration
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      speed_limit_topic: ""
      subsampling_enabled: false
      linear_granularity: 0.025
      angular_granularity: 0.025
      transform_tolerance: 0.2
      xy_goal_tolerance: 0.3  # Larger tolerance for humanoid balance
      yaw_goal_tolerance: 0.2
      stateful: true
      max_linear_jerk: 2.0
      max_angular_jerk: 3.0
      oscillation_distance: 0.05
      oscillation_angle: 0.2
      prune_distance: 1.0
      debug_trajectory_details: false
      min_vel_x: 0.05  # Minimum forward speed for stability
      min_vel_y: -0.1
      max_vel_x: 0.4   # Conservative speed for humanoid stability
      max_vel_y: 0.1
      max_vel_theta: 0.5
      acc_lim_x: 1.0   # Conservative acceleration for balance
      acc_lim_y: 1.0
      acc_lim_theta: 1.5
      decel_lim_x: -1.0
      decel_lim_y: -1.0
      decel_lim_theta: -1.5

local_costmap:
  ros__parameters:
    update_frequency: 10.0
    publish_frequency: 10.0
    global_frame: odom
    robot_base_frame: base_link
    rolling_window: true
    width: 10
    height: 10
    resolution: 0.05
    robot_radius: 0.5  # Conservative radius for humanoid
    plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
    inflation_layer:
      inflation_radius: 0.8  # Larger inflation for humanoid safety
      cost_scaling_factor: 3.0
    obstacle_layer:
      enabled: true
      observation_sources: scan
      scan:
        topic: /scan
        max_obstacle_height: 2.0  # Consider 3D obstacles for humanoid
        clearing: true
        marking: true
        data_type: "LaserScan"
        raytrace_max_range: 3.0
        raytrace_min_range: 0.0
        obstacle_max_range: 2.5
        obstacle_min_range: 0.0

global_costmap:
  ros__parameters:
    update_frequency: 1.0
    global_frame: map
    robot_base_frame: base_link
    robot_radius: 0.5
    resolution: 0.05
    plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
    inflation_layer:
      inflation_radius: 1.0
      cost_scaling_factor: 3.0
    obstacle_layer:
      enabled: true
      observation_sources: scan
      scan:
        topic: /scan
        max_obstacle_height: 2.0
        clearing: true
        marking: true
        data_type: "LaserScan"
        raytrace_max_range: 3.0
        raytrace_min_range: 0.0
        obstacle_max_range: 2.5
        obstacle_min_range: 0.0
```

## Integration with Isaac Ecosystem

### Isaac ROS to Nav2 Pipeline

The Isaac ROS perception pipeline can be integrated with Nav2 for complete navigation:

```yaml
# Example integration configuration
integration_pipeline:
  perception_node:
    package: "isaac_ros_visual_slam"
    executable: "visual_slam_node"
    parameters:
      - enable_rectification: true
      - enable_imu_fusion: true
      - map_frame: "map"
      - odom_frame: "odom"

  navigation_node:
    package: "nav2_bringup"
    executable: "nav2_launch.py"
    parameters:
      - use_sim_time: true
      - autostart: true
```

### Sensor Fusion for Navigation

Combining Isaac ROS perception with Nav2 navigation:

1. **Visual SLAM**: Provides accurate localization for navigation
2. **Object Detection**: Identifies dynamic obstacles for Nav2
3. **Depth Estimation**: Provides 3D information for navigation safety

## Performance Optimization for Humanoid Navigation

### Parameter Tuning

#### Costmap Optimization
- Adjust resolution based on humanoid size and environment
- Configure inflation radius for safety margins
- Optimize update frequencies for real-time performance

#### Controller Optimization
- Tune velocity limits for humanoid stability
- Adjust acceleration limits for balance preservation
- Optimize goal tolerances for humanoid precision

### Computational Efficiency

#### Multi-threading
- Utilize multiple CPU cores for navigation computations
- Separate perception and navigation threads
- Optimize data flow between components

#### Memory Management
- Efficient costmap updates for humanoid navigation
- Optimize path planning memory usage
- Manage trajectory memory for humanoid motion

## Best Practices for Humanoid Navigation

### Safety Considerations
- Implement conservative velocity limits for stability
- Use larger safety margins in costmaps
- Plan paths that maintain humanoid balance

### Configuration Management
- Test navigation parameters in simulation first
- Gradually increase performance parameters
- Monitor humanoid stability during navigation

### Debugging and Monitoring
- Use visualization tools to monitor navigation
- Log humanoid-specific metrics during navigation
- Implement safety checks for balance maintenance

## Summary

Nav2 provides a comprehensive navigation solution that can be adapted for humanoid robots with proper configuration. By understanding the unique challenges of humanoid navigation and configuring Nav2 appropriately, robotics engineers can create robust navigation systems that maintain humanoid balance and stability while achieving navigation goals. The integration with Isaac ROS perception capabilities creates a complete AI-powered navigation solution for humanoid robots.