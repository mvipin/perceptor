# Perceptor Robot

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)
[![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%205-red.svg)](https://www.raspberrypi.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

A comprehensive ROS2 robotics platform for autonomous navigation, SLAM mapping, and sensor fusion using an iRobot Create 2 base with integrated LiDAR sensor. Designed for educational robotics, research applications, and maker projects with full simulation support and real hardware deployment capabilities.

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

1. [Basic Robot Bringup and Teleoperation](#1-basic-robot-bringup-and-teleoperation)
2. [LiDAR Integration](#2-lidar-integration)
3. [SLAM Mapping and Localization](#3-slam-mapping-and-localization)
4. [Autonomous Navigation](#4-autonomous-navigation)
5. [Sensor Fusion (Odometry + IMU)](#5-sensor-fusion-odometry--imu)
6. [Installation and Setup](#installation-and-setup)
7. [Troubleshooting](#troubleshooting)

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

# Disable joystick for autonomous-only operation
ros2 launch perceptor launch_robot.launch.py enable_joystick:=false

# Enable/disable individual sensors
ros2 launch perceptor launch_robot.launch.py enable_lidar:=true enable_camera:=false
```

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
# Launch LiDAR sensor
ros2 launch perceptor rplidar.launch.py

# Verify scan data
ros2 topic echo /scan --once
ros2 topic hz /scan  # Should show ~10Hz
```

**Configuration Parameters:**
- `serial_port`: Device path (default: /dev/ttyUSB0)
- `frame_id`: TF frame name (default: laser)
- `angle_compensate`: Motor speed compensation (default: true)
- `scan_mode`: Scanning mode (default: Standard)

**Coordinate Frame Setup:**
```xml
<!-- URDF configuration in lidar.xacro -->
<joint name="laser_joint" type="fixed">
    <parent link="base_link"/>
    <child link="laser"/>
    <origin xyz="0 0 0.2" rpy="0 0 ${pi}"/>  <!-- 180° rotation for proper orientation -->
</joint>
```

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
# Start SLAM mapping (requires robot and LiDAR running)
ros2 launch slam_toolbox online_async_launch.py

# Alternative: integrated SLAM launch
ros2 launch perceptor slam_mapping.launch.py

# Save generated map
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

**Map Generation Process:**
1. **Start Robot**: Launch robot base and LiDAR sensor
2. **Initialize SLAM**: Start slam_toolbox node
3. **Explore Environment**: Drive robot to map area
4. **Monitor Progress**: Watch map building in RViz
5. **Save Map**: Export final map for navigation use

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

**Navigation Launch Commands:**
```bash
# Start navigation (requires existing map)
ros2 launch nav2_bringup navigation_launch.py map:=~/maps/my_map.yaml

# Alternative: integrated navigation launch
ros2 launch perceptor navigation.launch.py

# Set navigation goal via RViz
# Use "2D Goal Pose" tool in RViz interface
```

**Path Planning Setup:**
1. **Load Map**: Provide pre-generated map file
2. **Initialize Localization**: AMCL estimates robot pose
3. **Set Goal**: Specify target position via RViz or code
4. **Execute Plan**: Nav2 generates and follows path
5. **Monitor Progress**: Track navigation status and obstacles

**Navigation Parameters:**
- `robot_radius`: Robot footprint for collision checking
- `max_vel_x`: Maximum linear velocity (m/s)
- `max_vel_theta`: Maximum angular velocity (rad/s)
- `goal_tolerance`: Acceptable distance to goal (m)
- `obstacle_range`: LiDAR range for obstacle detection (m)

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

## 🔧 Troubleshooting

### Common Issues

#### Controller Not Detected
```bash
# Check Bluetooth status
sudo systemctl status bluetooth
sudo systemctl restart bluetooth

# Check device permissions
sudo usermod -a -G input $USER
```

#### LiDAR Connection Issues
```bash
# Check device connection
ls -la /dev/ttyUSB*

# Test with official launch
ros2 launch rplidar_ros rplidar_a1_launch.py
```

#### Robot Not Moving
```bash
# Check Create robot connection
ros2 topic echo /odom --once

# Verify joystick input
ros2 topic echo /joy --once

# Monitor command flow
ros2 topic echo /cmd_vel --once
```

### System Verification
```bash
# Check all topics
ros2 topic list

# Verify nodes
ros2 node list

# Check TF tree
ros2 run tf2_tools view_frames
```

## 📁 Package Structure

```
perceptor/
├── config/           # Configuration files
│   ├── pro_controller.yaml    # Nintendo Pro Controller config
│   ├── twist_mux.yaml        # Command multiplexer config
│   └── main.rviz            # RViz visualization config
├── launch/           # Launch files
│   ├── launch_robot.launch.py   # Main robot launch (integrated)
│   ├── launch_sim.launch.py     # Simulation launch
│   ├── joystick.launch.py       # Joystick control
│   └── sensors.launch.py        # Sensor systems
├── description/      # Robot description (URDF/Xacro)
└── worlds/          # Gazebo simulation worlds
```

---

## Acknowledgments

This project builds upon the excellent foundation provided by **Articulated Robotics** and their comprehensive ROS2 robotics tutorials and repositories. I extend our heartfelt gratitude to Josh Newans and the Articulated Robotics community for their invaluable contributions to robotics education and open-source development.

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

I welcome contributions from the robotics community! Please see our [Contributing Guidelines](CONTRIBUTING.md) for details on:

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
