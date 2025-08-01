# Perceptor Robot

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)
[![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%205-red.svg)](https://www.raspberrypi.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

A comprehensive ROS2 robotics platform for autonomous navigation, SLAM mapping, and sensor fusion using an iRobot Create 2 base with integrated LiDAR sensor. Features include advanced autonomous navigation with keepout zones, variable speed limits, visual waypoint navigation, real-time collision monitoring, and multi-sensor fusion using Extended Kalman Filter (EKF). Designed for educational robotics, research applications, and maker projects with full simulation support and real hardware deployment capabilities.

## 🚀 Quick Start

```bash
# Clone and build workspace
cd ~/Roomba/slam_dev_ws
colcon build --packages-select perceptor
source install/setup.bash

# Launch robot with teleoperation
ros2 launch perceptor launch_robot.launch.py

# Launch sensors separately (modular approach)
ros2 launch perceptor sensors.launch.py

# Visualization (on host computer with display)
ros2 run rviz2 rviz2 -d src/perceptor/config/main.rviz
```

## 📋 Table of Contents

- [🚀 Quick Start](#-quick-start)

### **1. [Basic Robot Bringup and Teleoperation](#1-basic-robot-bringup-and-teleoperation)**
  - [Hardware Components](#hardware-components)
  - [Software Components](#software-components)
  - [Launch File Usage](#launch-file-usage)

### **2. [LiDAR Integration](#2-lidar-integration)**
  - [Hardware Components](#hardware-components-1)
  - [Software Components](#software-components-1)
  - [Advanced Features](#advanced-features)

### **3. [SLAM Mapping and Localization](#3-slam-mapping-and-localization)**
  - [SLAM Algorithm Features](#slam-algorithm-features)
  - [Mapping Capabilities](#mapping-capabilities)
  - [Localization Systems](#localization-systems)

### **4. [Autonomous Navigation](#4-autonomous-navigation)**
  - [Core Navigation Stack](#core-navigation-stack)
  - [**Navigating with Keepout Zones**](#navigating-with-keepout-zones)
  - [**Navigating with Speed Limits**](#navigating-with-speed-limits)
  - [**Waypoint Navigation**](#waypoint-navigation)
  - [**Collision Monitor**](#collision-monitor)
  - [Advanced Navigation Features](#advanced-navigation-features)

### **5. [Sensor Fusion (Odometry + IMU)](#5-sensor-fusion-odometry--imu)**
  - [Multi-sensor Integration](#multi-sensor-integration)
  - [Hardware Components](#hardware-components-2)
  - [EKF Configuration](#ekf-configuration)

### **6. [Installation and Setup](#installation-and-setup)**
  - [Prerequisites](#prerequisites)
  - [Complete Installation Guide](#complete-installation-guide)
  - [Package Dependencies Summary](#package-dependencies-summary)

### **7. [Troubleshooting](#troubleshooting)**
  - [Common Hardware Issues](#common-hardware-issues)
  - [Software Debugging](#software-debugging)

### **Additional Resources**
  - [Acknowledgments](#acknowledgments)
  - [Contributing](#contributing)
  - [License](#license)

---

## 1. Basic Robot Bringup and Teleoperation

### Hardware Components

**Robot Platform:**
- **iRobot Create 2**: Differential drive mobile base (0.235m wheel separation, 0.036m wheel radius)
- **Raspberry Pi 5**: Main compute unit running ROS2 Jazzy (headless operation)
- **Nintendo Pro Controller**: Bluetooth gamepad for manual teleoperation

**Physical Specifications:**
- Base diameter: 348.5mm
- Maximum speed: 0.5 m/s linear, 4.25 rad/s angular
- Battery: Create 2 internal battery (14.4V, ~3000mAh)
- Communication: USB serial connection to Pi 5

### Software Components

**Core ROS2 Packages:**
- `create_robot`: iRobot Create 2 driver and description
- `joy`: Joystick/gamepad hardware interface
- `teleop_twist_joy`: Joystick-to-velocity command conversion
- `twist_mux`: Command arbitration and priority management

**Control Architecture:**
```
Nintendo Pro Controller → joy_node → teleop_node → twist_mux → create_driver → Robot Hardware
```

### Multimedia Placeholders
- 📹 **[Demo video of robot driving]** - Basic teleoperation demonstration
- 📊 **[RQT graph showing control flow]** - Node and topic visualization
- 🌳 **[TF tree diagram]** - Coordinate frame relationships

### Package Dependencies

**Build Requirements:**
```bash
# ROS2 packages
sudo apt install ros-jazzy-create-robot ros-jazzy-joy ros-jazzy-teleop-twist-joy ros-jazzy-twist-mux

# System dependencies
sudo apt install bluetooth bluez-tools
```

**Installation Commands:**
```bash
# Install dependencies
cd ~/Roomba/slam_dev_ws
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
colcon build --packages-select perceptor create_robot create_driver create_msgs create_description
source install/setup.bash
```

### Launch File Usage

**Primary Launch File: `launch_robot.launch.py`**
```bash
# Basic robot with teleoperation (default)
ros2 launch perceptor launch_robot.launch.py

# Basic teleoperation without sensors
ros2 launch perceptor launch_robot.launch.py enable_camera:=false enable_lidar:=false

# Disable joystick for autonomous-only operation
ros2 launch perceptor launch_robot.launch.py enable_joystick:=false

# Enable/disable individual sensors
ros2 launch perceptor launch_robot.launch.py enable_lidar:=true enable_camera:=false
```

**Basic Teleoperation Without Sensors:**
This command launches only the robot base with Nintendo Pro Controller support for manual driving without any sensors active. This is useful for initial testing and basic robot movement verification.

**Launch Arguments:**
- `enable_joystick`: Enable Nintendo Pro Controller (default: true)
- `enable_lidar`: Enable LiDAR sensor (default: false, launched separately)
- `enable_camera`: Enable camera sensor (default: false, launched separately)

**Gamepad Control Mapping:**
- **Right Bumper (RB)**: Deadman switch (must hold to move)
- **Left Stick**: Linear velocity (forward/backward)
- **Right Stick**: Angular velocity (rotation)
- **Left Bumper (LB)**: Turbo mode (increased speed scaling)

---

## 2. LiDAR Integration

### Hardware Components

**LiDAR Sensor:**
- **Model**: RPLiDAR A1/A2 (360-degree laser scanner)
- **Range**: 0.15m to 12.0m measurement distance
- **Resolution**: 360 samples per rotation (1° angular resolution)
- **Update Rate**: 10Hz scan frequency
- **Interface**: USB serial connection (/dev/ttyUSB0)
- **Power**: 5V via USB, ~400mA current draw

**Physical Mounting:**
- **Position**: Geometric center of robot, 200mm above base_link
- **Coordinate Frame**: `laser` frame with 180° Z-axis rotation for proper orientation
- **Mounting Hardware**: Custom 3D-printed bracket (STL files in `/hardware` directory)

### Software Components

**Core ROS2 Packages:**
- `rplidar_ros`: Official Slamtec RPLiDAR driver
- `sensor_msgs`: LaserScan message definitions
- `tf2`: Coordinate frame transformations

**Data Flow:**
```
RPLiDAR Hardware → rplidar_ros → /scan topic → SLAM/Navigation algorithms
                                     ↓
                              TF: base_link → laser
```

### Multimedia Placeholders
- 📹 **[LiDAR scan visualization in RViz]** - Real-time laser scan data display
- 🌳 **[TF tree with laser frame]** - Coordinate frame relationships including laser

### Package Dependencies

**Build Requirements:**
```bash
# RPLiDAR driver
sudo apt install ros-jazzy-rplidar-ros

# TF and sensor message support
sudo apt install ros-jazzy-tf2-ros ros-jazzy-sensor-msgs
```

**Hardware Setup:**
```bash
# Add user to dialout group for serial access
sudo usermod -a -G dialout $USER
# Logout and login required

# Check device connection
ls -la /dev/ttyUSB*
# Should show: /dev/ttyUSB0 (or similar)
```

### Launch File Usage

**Primary Launch File: `rplidar.launch.py`**
```bash
# Launch LiDAR sensor separately
ros2 launch perceptor rplidar.launch.py

# Robot operation with LiDAR enabled
ros2 launch perceptor launch_robot.launch.py enable_camera:=false

# Verify scan data
ros2 topic echo /scan --once
ros2 topic hz /scan  # Should show ~10Hz
```

**Robot Operation with LiDAR:**
This enables the robot base with LiDAR sensor active for laser scan data collection while keeping the camera disabled. This configuration is ideal for SLAM mapping and navigation tasks.

**Configuration Parameters:**
- `serial_port`: Device path (default: /dev/ttyUSB0)
- `frame_id`: TF frame name (default: laser)
- `angle_compensate`: Motor speed compensation (default: true)
- `scan_mode`: Scanning mode (default: Standard)

---

## 3. SLAM Mapping and Localization

### Software Components

**SLAM Algorithm:**
- **Package**: `slam_toolbox` (pose graph-based SLAM)
- **Algorithm**: Karto SLAM with loop closure detection
- **Map Representation**: Occupancy grid (nav_msgs/OccupancyGrid)
- **Localization**: Particle filter-based pose estimation

**Data Processing Pipeline:**
```
LiDAR (/scan) + Odometry (/odom) → slam_toolbox → Map (/map) + Pose (/tf: map→odom)
```

**Key Features:**
- Real-time mapping during robot operation
- Loop closure detection for map consistency
- Pose graph optimization for drift correction
- Map serialization for persistent storage

### Multimedia Placeholders
- 📹 **[Real-time mapping demo video]** - SLAM in action during exploration
- 🗺️ **[Generated map visualization]** - Example maps created in different environments

### Package Dependencies

**Build Requirements:**
```bash
# SLAM Toolbox
sudo apt install ros-jazzy-slam-toolbox

# Navigation messages and TF
sudo apt install ros-jazzy-nav-msgs ros-jazzy-tf2-geometry-msgs
```

**Configuration Files:**
```bash
# SLAM configuration
src/perceptor/config/slam_config.yaml

# Map saving location
maps/generated_map.yaml
maps/generated_map.pgm
```

### Launch File Usage

**SLAM Launch Commands:**
```bash
# Start SLAM mapping with custom configuration
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/pi/Roomba/slam_dev_ws/src/perceptor/config/mapper_params_online_async.yaml
```

**Map Generation Process:**
1. **Start Robot**: Launch robot base and LiDAR sensor
2. **Initialize SLAM**: Start slam_toolbox node with custom configuration
3. **Load SLAM Toolbox Panel**: Open the SLAM Toolbox panel in RViz
4. **Explore Environment**: Drive robot around to build the map
5. **Monitor Progress**: Watch map building in RViz interface
6. **Save Map**: Use the SLAM Toolbox panel to save the generated map

**Important Note:** The map files will be saved on the machine where the SLAM command was executed. Use the SLAM Toolbox panel in RViz for the most reliable map saving process.

**SLAM Parameters:**
- `map_frame`: Map coordinate frame (default: map)
- `odom_frame`: Odometry frame (default: odom)
- `base_frame`: Robot base frame (default: base_link)
- `scan_topic`: LiDAR data topic (default: /scan)
- `resolution`: Map resolution in meters/pixel (default: 0.05)

---

## 4. Autonomous Navigation

### Software Components

**Navigation Stack:**
- **Package**: `nav2` (Navigation2 stack for ROS2)
- **Localization**: AMCL (Adaptive Monte Carlo Localization)
- **Path Planning**: NavFn planner with Dijkstra's algorithm
- **Control**: DWB (Dynamic Window Approach) local planner
- **Obstacle Avoidance**: Costmap-based dynamic obstacle detection

**Navigation Pipeline:**
```
Goal → Global Planner → Local Planner → cmd_vel → Robot
  ↑         ↑              ↑
Map    Costmaps    LiDAR + Odometry
```

**Key Capabilities:**
- Autonomous path planning to goal positions
- Dynamic obstacle avoidance during navigation
- Recovery behaviors for stuck situations
- Multi-layered costmap system for safety

### Multimedia Placeholders
- 📹 **[Navigation demo video]** - Autonomous navigation to goal positions
- 🌳 **[Nav2 behavior tree visualization]** - Decision-making process visualization

### Package Dependencies

**Build Requirements:**
```bash
# Navigation2 stack
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup

# AMCL localization
sudo apt install ros-jazzy-nav2-amcl

# Map server for pre-built maps
sudo apt install ros-jazzy-nav2-map-server
```

**Configuration Files:**
```bash
# Navigation parameters
src/perceptor/config/nav2_params.yaml

# Costmap configuration
src/perceptor/config/costmap_params.yaml

# AMCL localization settings
src/perceptor/config/amcl_params.yaml
```

### Launch File Usage

**Standard 3-Terminal Navigation Launch Sequence:**
```bash
# Terminal 1: Robot base and LiDAR
ros2 launch perceptor launch_robot.launch.py enable_camera:=false

# Terminal 2: AMCL localization (requires existing map)
ros2 launch perceptor localization_launch.py \
  map:=/home/pi/Roomba/slam_dev_ws/src/perceptor/maps/home.yaml \
  params_file:=/home/pi/Roomba/slam_dev_ws/src/perceptor/config/nav2_params.yaml

# Terminal 3: Navigation stack (path planning and controllers)
ros2 launch perceptor navigation_launch.py \
  params_file:=/home/pi/Roomba/slam_dev_ws/src/perceptor/config/nav2_params.yaml

# Set navigation goal via RViz
# Use "2D Goal Pose" tool in RViz interface
```

**Enhanced Navigation with Keepout Zones:**
For navigation with keepout zone support, add a 4th terminal:
```bash
# Terminal 4: Keepout zone extension (optional)
ros2 launch perceptor keepout_extension.launch.py \
  keepout_mask:=/home/pi/Roomba/slam_dev_ws/src/perceptor/maps/keepout_mask.yaml
```

**AMCL Localization Setup:**
After launching AMCL localization (Terminal 2), you must set the initial robot pose estimate in RViz using the '2D Pose Estimate' tool. This helps the particle filter converge to the correct robot position.

**Navigation Stack Integration:**
The navigation launch (Terminal 3) provides path planning, local control, and behavior coordination. It integrates seamlessly with the localization system to enable autonomous navigation.

**Path Planning Workflow:**
1. **Launch Robot Base**: Start robot hardware and sensor drivers (Terminal 1)
2. **Initialize Localization**: Load map and start AMCL (Terminal 2)
3. **Start Navigation**: Launch path planning and control systems (Terminal 3)
4. **Set Initial Pose**: Use RViz '2D Pose Estimate' tool to initialize robot position
5. **Set Navigation Goal**: Specify target position via RViz '2D Goal Pose' tool
6. **Execute Plan**: Nav2 generates and follows optimal path to goal
7. **Monitor Progress**: Track navigation status, obstacles, and goal completion

**Navigation Parameters:**
- `robot_radius`: Robot footprint for collision checking
- `max_vel_x`: Maximum linear velocity (m/s)
- `max_vel_theta`: Maximum angular velocity (rad/s)
- `goal_tolerance`: Acceptable distance to goal (m)
- `obstacle_range`: LiDAR range for obstacle detection (m)

### Navigating with Keepout Zones

**Concept:**
Keepout zones allow you to define no-go areas in your environment where the robot should never navigate. These zones are useful for protecting sensitive equipment, avoiding hazardous areas, or respecting restricted spaces.

**Implementation:**
The keepout zone filter integrates with Nav2's costmap system to mark specific areas as completely impassable, with infinite cost values that prevent path planning through these regions.

**Package Dependencies:**
```bash
# Keepout zone support (included in nav2-bringup)
sudo apt install ros-jazzy-nav2-costmap-2d
```

**Configuration Parameters:**
```yaml
# costmap_params.yaml - Keepout Zone Filter
keepout_filter:
  plugin: "nav2_costmap_2d::KeepoutFilter"
  enabled: True
  filter_info_topic: "/costmap_filter_info"

# Example keepout zone definition
keepout_zones:
  - polygon: [[2.0, 1.0], [3.0, 1.0], [3.0, 2.0], [2.0, 2.0]]  # Square zone
  - polygon: [[5.0, 3.0], [6.0, 4.0], [5.5, 5.0]]              # Triangle zone
```

**Modular 4-Terminal Launch Sequence:**
```bash
# Terminal 1: Robot base and LiDAR
ros2 launch perceptor launch_robot.launch.py enable_camera:=false

# Terminal 2: AMCL localization with main navigation map
ros2 launch perceptor localization_launch.py \
  map:=/home/pi/Roomba/slam_dev_ws/src/perceptor/maps/home.yaml \
  params_file:=/home/pi/Roomba/slam_dev_ws/src/perceptor/config/nav2_params.yaml

# Terminal 3: Navigation stack (path planning and controllers)
ros2 launch perceptor navigation_launch.py \
  params_file:=/home/pi/Roomba/slam_dev_ws/src/perceptor/config/nav2_params.yaml

# Terminal 4: Keepout zone extension (modular add-on)
ros2 launch perceptor keepout_extension.launch.py \
  keepout_mask:=/home/pi/Roomba/slam_dev_ws/src/perceptor/maps/keepout_mask.yaml
```

**Architecture Benefits:**
- **Modular Design**: Each component can be started/stopped independently
- **Easy Debugging**: Issues can be isolated to specific terminals
- **Flexible Usage**: Keepout zones can be added/removed without restarting navigation
- **Standard Nav2 Pattern**: Follows established localization + navigation separation

**Verification Commands:**
```bash
# Check keepout mask is loaded
ros2 topic echo /keepout_filter_mask --once

# Verify costmap integration
ros2 topic echo /global_costmap/costmap --once

# Monitor navigation behavior
ros2 topic echo /plan --once
```

**Multimedia Placeholders:**
- 📊 **[Configuration example screenshots]** - RViz visualization of keepout zones
- 📹 **[Keepout zone navigation demo]** - Robot avoiding restricted areas

**Reference:** [Nav2 Keepout Zones Tutorial](https://docs.nav2.org/tutorials/docs/navigation2_with_keepout_filter.html)

### Navigating with Speed Limits

**Concept:**
Speed restriction zones enable variable velocity limits across different areas of your map. This feature allows safer navigation in crowded areas, near fragile equipment, or in zones requiring careful movement.

**Implementation:**
The speed filter modifies the robot's maximum velocities based on map annotations, creating dynamic speed zones that adapt to environmental requirements.

**Package Dependencies:**
```bash
# Speed limit support (included in nav2-bringup)
sudo apt install ros-jazzy-nav2-costmap-2d ros-jazzy-nav2-controller
```

**Configuration Parameters:**
```yaml
# costmap_params.yaml - Speed Filter
speed_filter:
  plugin: "nav2_costmap_2d::SpeedFilter"
  enabled: True
  filter_info_topic: "/speed_filter_info"
  speed_limit_topic: "/speed_limit"

# Example speed zones
speed_zones:
  - area: [[0.0, 0.0], [5.0, 0.0], [5.0, 5.0], [0.0, 5.0]]  # Slow zone
    max_speed: 0.2  # 0.2 m/s maximum
  - area: [[10.0, 10.0], [15.0, 15.0]]  # Fast zone
    max_speed: 0.8  # 0.8 m/s maximum
```

**Modular 5-Terminal Launch Sequence (Speed Limits Only):**
```bash
# Terminal 1: Robot base and LiDAR
ros2 launch perceptor launch_robot.launch.py enable_camera:=false

# Terminal 2: AMCL localization with main navigation map
ros2 launch perceptor localization_launch.py \
  map:=/home/pi/Roomba/slam_dev_ws/src/perceptor/maps/home.yaml \
  params_file:=/home/pi/Roomba/slam_dev_ws/src/perceptor/config/nav2_params.yaml

# Terminal 3: Navigation stack (path planning and controllers)
ros2 launch perceptor navigation_launch.py \
  params_file:=/home/pi/Roomba/slam_dev_ws/src/perceptor/config/nav2_params.yaml

# Terminal 4: Speed limit extension (modular add-on)
ros2 launch perceptor speed_extension.launch.py \
  speed_mask:=/home/pi/Roomba/slam_dev_ws/src/perceptor/maps/speed_mask.yaml
```

**Combined Navigation Filters (Speed + Keepout Zones):**
```bash
# Terminal 4: Both speed limits and keepout zones
ros2 launch perceptor navigation_filters.launch.py \
  speed_mask:=/home/pi/Roomba/slam_dev_ws/src/perceptor/maps/speed_mask.yaml \
  keepout_mask:=/home/pi/Roomba/slam_dev_ws/src/perceptor/maps/keepout_mask.yaml
```

**Speed Zone Mask Configuration:**
Speed zones are defined using grayscale values in the mask image:
- **White (255)**: Full speed (1.0x multiplier)
- **Light Gray (192)**: 75% speed (0.75x multiplier)
- **Medium Gray (128)**: 50% speed (0.5x multiplier)
- **Dark Gray (64)**: 25% speed (0.25x multiplier)
- **Black (0)**: Very slow (0.1x multiplier)

**Verification Commands:**
```bash
# Check speed mask is loaded
ros2 topic echo /speed_filter_mask --once

# Monitor current speed limits
ros2 topic echo /speed_limit

# Verify costmap integration
ros2 topic echo /global_costmap/costmap --once

# Check speed filter info
ros2 topic echo /speed_filter_info --once
```

**Technical Note - Speed Filter Behavior:**
The speed filter operates by:
- **Proportional Scaling**: Speed multipliers are applied based on mask grayscale values
- **Real-time Updates**: Speed limits change dynamically as robot moves through zones
- **Controller Integration**: Works with DWB local planner to enforce speed restrictions
- **Smooth Transitions**: Gradual speed changes between zones prevent abrupt stops

**Integration with Existing Systems:**
- **Compatible with Keepout Zones**: Both filters can operate simultaneously
- **Costmap Integration**: Speed zones appear as colored overlays in global costmap
- **Parameter Inheritance**: Uses existing nav2_params.yaml with added speed filter configuration
- **Lifecycle Management**: Independent lifecycle control for easy enable/disable

**Expected Behavior:**
- Robot automatically slows down when entering darker zones
- Speed increases when moving to lighter zones
- Path planning considers speed restrictions for optimal routes
- Local controller respects speed limits during execution

**Troubleshooting:**
```bash
# Check if speed filter is active in costmap
ros2 param get /global_costmap/global_costmap plugins

# Verify speed filter configuration
ros2 param get /global_costmap/global_costmap speed_filter.enabled

# Monitor speed filter topic subscriptions
ros2 topic info /speed_filter_info --verbose
```

**Multimedia Placeholders:**
- 📊 **[Speed zone configuration screenshots]** - Map with color-coded speed areas
- 📹 **[Variable speed navigation demo]** - Robot adapting speed to different zones

**Reference:** [Nav2 Speed Filter Tutorial](https://docs.nav2.org/tutorials/docs/navigation2_with_speed_filter.html)

### Waypoint Navigation

**Concept:**
Waypoint navigation enables the robot to follow a sequence of predefined points, creating complex patrol routes, inspection paths, or multi-destination missions. The waypoint follower manages the sequence and handles failures gracefully.

**Implementation:**
The waypoint follower functionality is **enabled** through the Perceptor robot's nav2_params.yaml configuration. No additional launch files or command-line tools are needed - waypoint navigation is controlled entirely through RViz's intuitive graphical interface.

**Key Advantage:**
RViz's Nav2 plugin provides a visual, point-and-click interface for setting waypoints directly on the map, making waypoint navigation accessible without programming knowledge.

### Setting Waypoints with RViz Interface

**Step-by-Step Waypoint Navigation:**

1. **Launch Navigation Stack** (standard 3-terminal setup):
```bash
# Terminal 1: Robot base and LiDAR
ros2 launch perceptor launch_robot.launch.py enable_camera:=false

# Terminal 2: AMCL localization with main navigation map
ros2 launch perceptor localization_launch.py \
  map:=/home/pi/Roomba/slam_dev_ws/src/perceptor/maps/home.yaml \
  params_file:=/home/pi/Roomba/slam_dev_ws/src/perceptor/config/nav2_params.yaml

# Terminal 3: Navigation stack (path planning and controllers)
ros2 launch perceptor navigation_launch.py \
  params_file:=/home/pi/Roomba/slam_dev_ws/src/perceptor/config/nav2_params.yaml
```

2. **Open RViz** and ensure Nav2 plugin is loaded:
   ```bash
   rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz
   ```

3. **Set Initial Robot Pose**:
   - Click **"2D Pose Estimate"** tool in RViz toolbar
   - Click and drag on map to set robot's starting position and orientation
   - Verify AMCL particle cloud converges around robot

4. **Add Waypoints Visually**:
   - Click **"Waypoint Mode"** button in Nav2 panel (or use "Nav2 Goal" tool)
   - Click on map locations where you want the robot to visit
   - Each click adds a waypoint to the sequence
   - Waypoints appear as numbered markers on the map

5. **Start Waypoint Following**:
   - Click **"Start Waypoint Following"** button in Nav2 panel
   - Robot will navigate to each waypoint in sequence
   - Progress is shown in RViz with path visualization and status updates

**Integration with Navigation Features:**
- **Keepout Zone Awareness**: Waypoints automatically avoid restricted areas
- **Speed Zone Compliance**: Robot adjusts speed appropriately during waypoint navigation
- **Dynamic Obstacle Avoidance**: Real-time path adjustments around moving obstacles
- **Recovery Behaviors**: Automatic retry and recovery on navigation failures

**Configuration Parameters** (already enabled in nav2_params.yaml):
```yaml
waypoint_follower:
  ros__parameters:
    loop_rate: 20.0
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: True
      waypoint_pause_duration: 2000  # 2 seconds pause at each waypoint
```

**Advanced Waypoint Features:**
- **Looped Patrol Routes**: Set `stop_on_failure: false` for continuous patrol
- **Custom Pause Durations**: Adjust wait time at each waypoint as needed
- **Failure Recovery**: Automatic retry and alternative path planning
- **Multi-Goal Coordination**: Seamless integration with single-goal navigation

**Troubleshooting Waypoint Navigation:**
```bash
# Check waypoint follower status
ros2 lifecycle get /waypoint_follower

# Monitor waypoint execution
ros2 topic echo /waypoint_follower/transition_event

# View current waypoint action status
ros2 action list | grep waypoint
```

**Multimedia Placeholders:**
- � **[RViz waypoint interface demo]** - Visual waypoint setting and execution
- 📊 **[Multi-waypoint patrol screenshots]** - Complex route planning examples

**Reference:** [Nav2 Waypoint Follower Tutorial](https://docs.nav2.org/tutorials/docs/navigation2_with_waypoint_follower.html)

### Collision Monitor

**Concept:**
The collision monitor provides real-time safety monitoring by continuously checking for potential collisions using sensor data. It can trigger emergency stops, speed reductions, or approach warnings based on configurable safety zones.

**Implementation:**
The collision_monitor node creates multiple polygon-based detection zones around the robot, each with different safety behaviors. It integrates with LiDAR data to provide immediate collision avoidance responses.

**Package Dependencies:**
```bash
# Collision monitoring support
sudo apt install ros-jazzy-nav2-collision-monitor ros-jazzy-nav2-lifecycle-manager
```

**Configuration Parameters:**
```yaml
# collision_monitor_params.yaml
collision_monitor:
  base_frame_id: "base_link"
  odom_frame_id: "odom"
  cmd_vel_in_topic: "cmd_vel_smoothed"
  cmd_vel_out_topic: "cmd_vel"
  state_topic: "collision_monitor_state"

  # Polygon definitions
  polygons: ["PolygonStop", "PolygonSlow", "PolygonApproach"]

  PolygonStop:
    type: "polygon"
    points: [0.3, 0.3, 0.3, -0.3, -0.2, -0.3, -0.2, 0.3]  # Close safety zone
    action_type: "stop"  # Emergency stop
    max_points: 3  # Minimum points to trigger

  PolygonSlow:
    type: "polygon"
    points: [0.6, 0.5, 0.6, -0.5, -0.4, -0.5, -0.4, 0.5]  # Medium safety zone
    action_type: "slowdown"
    slowdown_ratio: 0.3  # Reduce speed to 30%

  PolygonApproach:
    type: "polygon"
    points: [1.0, 0.8, 1.0, -0.8, -0.6, -0.8, -0.6, 0.8]  # Warning zone
    action_type: "approach"  # Warning only
```

**Launch Commands:**
```bash
# Start collision monitor with navigation
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false \
  params_file:=src/perceptor/config/collision_monitor_params.yaml

# Monitor collision states
ros2 topic echo /collision_monitor_state

# Check collision monitor status
ros2 lifecycle get /collision_monitor
```

**Safety Configuration Examples:**
```yaml
# Conservative settings (slow robot)
conservative_safety:
  stop_zone: 0.4m radius    # Large stop zone
  slow_zone: 0.8m radius    # Large slow zone
  slowdown_ratio: 0.2       # Very slow (20% speed)

# Aggressive settings (fast robot)
aggressive_safety:
  stop_zone: 0.2m radius    # Small stop zone
  slow_zone: 0.5m radius    # Medium slow zone
  slowdown_ratio: 0.6       # Moderate slow (60% speed)
```

**Multimedia Placeholders:**
- 📊 **[Collision zone visualization screenshots]** - RViz display of safety polygons
- 📹 **[Emergency stop demo video]** - Collision monitor preventing crashes

**Reference:** [Nav2 Collision Monitor Tutorial](https://docs.nav2.org/tutorials/docs/navigation2_with_collision_monitor.html)

---

## 5. Sensor Fusion (Odometry + IMU)

### Hardware Components

**IMU Sensor:**
- **Model**: MPU6050 (6-axis accelerometer + gyroscope)
- **Interface**: I2C connection to Raspberry Pi 5
- **Sampling Rate**: 100Hz for high-frequency motion data
- **Mounting**: Rigidly attached to robot base for accurate measurements

**Sensor Integration:**
- **Wheel Odometry**: Create 2 built-in encoders (primary)
- **IMU Data**: Angular velocity and linear acceleration (secondary)
- **Fusion Algorithm**: Extended Kalman Filter (EKF) for optimal state estimation

### Software Components

**Sensor Fusion Stack:**
- **Package**: `robot_localization` (industry-standard EKF implementation)
- **Filter Type**: Extended Kalman Filter with 15-state model
- **Input Sources**: Wheel odometry (/odom) + IMU data (/imu/data)
- **Output**: Fused odometry (/odometry/filtered) with improved accuracy

**Data Flow:**
```
Wheel Encoders → /odom ↘
                        → EKF → /odometry/filtered → SLAM/Navigation
IMU Sensor → /imu/data ↗
```

### Multimedia Placeholders
- 📊 **[Sensor fusion accuracy comparison]** - Before/after EKF performance metrics
- 📈 **[EKF state estimation visualization]** - Real-time filter performance plots

### Package Dependencies

**Build Requirements:**
```bash
# Robot localization package
sudo apt install ros-jazzy-robot-localization

# IMU driver and messages
sudo apt install ros-jazzy-imu-tools ros-jazzy-sensor-msgs-py

# I2C tools for hardware setup
sudo apt install i2c-tools python3-smbus
```

**Hardware Configuration:**
```bash
# Enable I2C on Raspberry Pi
sudo raspi-config
# Interface Options → I2C → Enable

# Check I2C device detection
sudo i2cdetect -y 1
# Should show MPU6050 at address 0x68
```

### EKF Configuration

**Launch File Setup:**
```bash
# Start sensor fusion
ros2 launch robot_localization ekf.launch.py

# Integrated launch with robot
ros2 launch perceptor launch_robot.launch.py enable_imu:=true
```

**EKF Parameters (ekf_config.yaml):**
```yaml
frequency: 30  # Filter update rate (Hz)
sensor_timeout: 0.1  # Maximum sensor delay (s)

# Odometry configuration
odom0: /odom
odom0_config: [true,  true,  false,  # x, y, z
               false, false, true,   # roll, pitch, yaw
               true,  true,  false,  # vx, vy, vz
               false, false, true,   # vroll, vpitch, vyaw
               false, false, false]  # ax, ay, az

# IMU configuration
imu0: /imu/data
imu0_config: [false, false, false,  # x, y, z
              false, false, true,   # roll, pitch, yaw
              false, false, false,  # vx, vy, vz
              false, false, true,   # vroll, vpitch, vyaw
              true,  true,  false]  # ax, ay, az
```

**Benefits of Sensor Fusion:**
- **Improved Accuracy**: Reduced odometry drift over time
- **Better Orientation**: More stable yaw estimation
- **Faster Response**: Higher update rate than wheel encoders alone
- **Robustness**: Continued operation if one sensor fails

---

## Installation and Setup

### Prerequisites

**System Requirements:**
- **Operating System**: Ubuntu 22.04 LTS or Raspberry Pi OS (64-bit)
- **ROS2 Distribution**: Jazzy Jalopy
- **Hardware**: iRobot Create 2, RPLiDAR A1/A2, Nintendo Pro Controller
- **Compute**: Raspberry Pi 5 (8GB recommended) or x86_64 PC

### Complete Installation Guide

**1. ROS2 Jazzy Installation:**
```bash
# Add ROS2 repository
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS2 Jazzy
sudo apt update
sudo apt install ros-jazzy-desktop  # Full installation
source /opt/ros/jazzy/setup.bash
```

**2. Workspace Setup:**
```bash
# Create workspace
mkdir -p ~/Roomba/slam_dev_ws/src
cd ~/Roomba/slam_dev_ws/src

# Clone required repositories
git clone https://github.com/AutonomyLab/create_robot.git
git clone https://github.com/Slamtec/rplidar_ros.git
git clone <this-repository-url> perceptor

# Install all dependencies
cd ~/Roomba/slam_dev_ws
rosdep install --from-paths src --ignore-src -r -y
```

**3. Build Workspace:**
```bash
# Build all packages
cd ~/Roomba/slam_dev_ws
colcon build --symlink-install

# Source workspace
source install/setup.bash
echo "source ~/Roomba/slam_dev_ws/install/setup.bash" >> ~/.bashrc
```

**4. Hardware Setup:**
```bash
# Serial port permissions
sudo usermod -a -G dialout $USER

# Bluetooth setup (for gamepad)
sudo apt install bluetooth bluez-tools
sudo systemctl enable bluetooth

# I2C setup (for IMU)
sudo raspi-config  # Enable I2C interface
sudo apt install i2c-tools python3-smbus

# Reboot required after permission changes
sudo reboot
```

### Package Dependencies Summary

**Core Dependencies:**
```bash
# Robot base and control
ros-jazzy-create-robot ros-jazzy-joy ros-jazzy-teleop-twist-joy ros-jazzy-twist-mux

# LiDAR and sensors
ros-jazzy-rplidar-ros ros-jazzy-sensor-msgs

# SLAM and navigation
ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-nav2-bringup

# Sensor fusion
ros-jazzy-robot-localization ros-jazzy-imu-tools

# Visualization and tools
ros-jazzy-rviz2 ros-jazzy-tf2-tools ros-jazzy-rqt-graph
```

---

## Troubleshooting

### Common Hardware Issues

**Nintendo Pro Controller Connection:**
```bash
# Check Bluetooth status
sudo systemctl status bluetooth
sudo systemctl restart bluetooth

# Manual pairing process
sudo bluetoothctl
scan on
# Hold Share + Home buttons on controller
pair XX:XX:XX:XX:XX:XX
trust XX:XX:XX:XX:XX:XX
connect XX:XX:XX:XX:XX:XX
exit

# Verify controller detection
ls /dev/input/js*  # Should show js0
ros2 topic echo /joy --once  # Test joystick data
```

**LiDAR Connection Problems:**
```bash
# Check USB serial connection
ls -la /dev/ttyUSB*  # Should show ttyUSB0

# Test LiDAR directly
ros2 launch rplidar_ros rplidar_a1_launch.py
ros2 topic echo /scan --once  # Verify scan data

# Permission issues
sudo usermod -a -G dialout $USER
# Logout and login required
```

**Create Robot Communication:**
```bash
# Check Create robot connection
ros2 topic echo /odom --once  # Should show odometry data
ros2 topic list | grep create  # Show Create-related topics

# Serial port issues
sudo chmod 666 /dev/ttyUSB0  # Temporary fix
# Permanent: add user to dialout group (see above)
```

### Software Debugging

**TF Tree Issues:**
```bash
# Visualize coordinate frames
ros2 run tf2_tools view_frames
evince frames.pdf  # View generated TF tree

# Check specific transforms
ros2 run tf2_ros tf2_echo base_link laser
ros2 run tf2_ros tf2_echo map odom
```

**Node and Topic Verification:**
```bash
# List all active nodes
ros2 node list

# Check topic connections
ros2 topic list
ros2 topic info /scan
ros2 topic hz /scan  # Verify message frequency

# Monitor system performance
ros2 run rqt_graph rqt_graph  # Visualize node graph
```

**SLAM and Navigation Issues:**
```bash
# Check SLAM status
ros2 topic echo /map --once  # Verify map generation
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: test_map}}"

# Navigation debugging
ros2 topic echo /cmd_vel  # Monitor velocity commands
ros2 topic echo /amcl_pose  # Check localization
```

---

## Acknowledgments

This project builds upon the excellent foundation provided by **Articulated Robotics** and their comprehensive ROS2 robotics tutorials and repositories. I extend my heartfelt gratitude to Josh Newans and the Articulated Robotics community for their invaluable contributions to robotics education and open-source development.

**Special Thanks:**
- **Articulated Robotics**: Original repository structure and ROS2 best practices
- **Create Robot Community**: iRobot Create 2 driver development and maintenance
- **SLAM Toolbox Team**: Robust SLAM implementation for ROS2
- **Navigation2 Team**: Comprehensive autonomous navigation stack
- **ROS2 Community**: Continuous development of the robotics ecosystem

**Educational Resources:**
- [Articulated Robotics YouTube Channel](https://www.youtube.com/c/ArticulatedRobotics)
- [ROS2 Documentation](https://docs.ros.org/en/jazzy/)
- [Navigation2 Tutorials](https://navigation.ros.org/)

---

## Contributing

I welcome contributions from the robotics community! Please see my [Contributing Guidelines](CONTRIBUTING.md) for details on:

- Code style and standards
- Testing procedures
- Documentation requirements
- Pull request process

**Areas for Contribution:**
- Hardware integration guides
- Performance optimizations
- Additional sensor support
- Educational tutorials
- Bug fixes and improvements

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

<div align="center">

### ⭐ Star this repository if you found it helpful! ⭐

**Built with ❤️ for the robotics community**

**Made with Arduino | Powered by Mathematics | Open Source Love**

---

*Perceptor Robot - Advancing autonomous robotics through open collaboration*

</div>
