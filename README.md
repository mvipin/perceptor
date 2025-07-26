# Perceptor Robot

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)
[![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%205-red.svg)](https://www.raspberrypi.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

A comprehensive ROS2 package for autonomous navigation and perception using iRobot Create 2 base with integrated LiDAR and camera sensors. Supports both simulation and real robot deployment with Nintendo Pro Controller teleoperation.

## 🚀 Quick Start

```bash
# Clone and build
cd ~/Roomba/slam_dev_ws
colcon build --packages-select perceptor
source install/setup.bash

# Launch complete robot system (integrated)
ros2 launch perceptor launch_robot.launch.py

# Visualization (on host computer)
ros2 run rviz2 rviz2 -d src/perceptor/config/main.rviz
```

## 🎮 Manual Control

**Nintendo Pro Controller** (Bluetooth):
- **Hold Y button** + **move left stick** to control robot
- **Release Y button** to immediately stop (safety feature)
- Two speed modes: slow (0.2 m/s) and fast (0.4 m/s)

## 🤖 Hardware Configuration

### Robot Platform
- **Base**: iRobot Create 2 (differential drive, 0.235m wheel separation)
- **LiDAR**: RPLidar A1M8 (360°, 12m range, 10Hz, 115200 baud)
- **Camera**: USB camera (640x480, 62.4° FOV)
- **Controller**: Nintendo Switch Pro Controller (Bluetooth)

### Sensor Mounting
```
base_link (Create robot center)
├── laser (0, 0, 0.2)           # LiDAR: 200mm above center
└── camera_link (0.075, 0, 0.1) # Camera: 75mm forward, 100mm up
```

## 🏗️ System Architecture

### Control Flow
```
Nintendo Pro Controller → joy_node → joy_teleop → twist_mux → create_driver → Robot
                                                      ↑
                                    Navigation/Autonomous ← nav2/slam_toolbox
```

### Command Priority (twist_mux)
1. **Joystick**: Priority 100 (manual override)
2. **Ball Tracker**: Priority 20 (autonomous following)  
3. **Navigation**: Priority 10 (path planning)

### Operational Modes
- **Real Robot**: `use_sim_time:=false` (default) - Physical Create robot
- **Simulation**: `use_sim_time:=true` - Gazebo simulation with ros2_control

## 📦 Installation

### Prerequisites
- **ROS2 Jazzy** on Ubuntu 22.04 or Raspberry Pi OS
- **Hardware**: iRobot Create 2, RPLidar A1M8, USB camera
- **Dependencies**: `create_robot`, `rplidar_ros`, `joy`, `twist_mux`

### Build Instructions
```bash
# Clone workspace
cd ~/Roomba/slam_dev_ws/src
git clone <repository-url> perceptor

# Install dependencies
cd ~/Roomba/slam_dev_ws
rosdep install --from-paths src --ignore-src -r -y

# Build package
colcon build --packages-select perceptor
source install/setup.bash
```

### Hardware Setup

#### Nintendo Pro Controller Pairing
```bash
# Enable Bluetooth and pair controller
sudo bluetoothctl
scan on
# Hold Share + Home buttons on controller
pair XX:XX:XX:XX:XX:XX
trust XX:XX:XX:XX:XX:XX
connect XX:XX:XX:XX:XX:XX
exit
```

#### LiDAR & Create Robot Connection
- **LiDAR**: Connect RPLidar A1M8 to `/dev/ttyUSB0`
- **Create Robot**: Connect via USB-to-serial adapter
- **Permissions**: `sudo usermod -a -G dialout $USER` (logout/login required)

## 🚀 Usage

### Integrated System Launch
```bash
# Complete robot system (default: all components enabled)
ros2 launch perceptor launch_robot.launch.py

# Custom configuration examples
ros2 launch perceptor launch_robot.launch.py enable_lidar:=false
ros2 launch perceptor launch_robot.launch.py enable_camera:=false enable_joystick:=false
ros2 launch perceptor launch_robot.launch.py enable_lidar:=true enable_camera:=true enable_joystick:=true

# Visualization (host computer)
ros2 run rviz2 rviz2 -d src/perceptor/config/main.rviz
```

### Launch Parameters
- **enable_joystick**: `true/false` (default: true) - Nintendo Pro Controller
- **enable_lidar**: `true/false` (default: true) - RPLidar A1M8 sensor
- **enable_camera**: `true/false` (default: true) - USB camera

### Individual Components (Alternative)
```bash
# Robot base only
ros2 launch perceptor launch_robot.launch.py enable_lidar:=false enable_camera:=false

# Sensors separately (if needed)
ros2 launch perceptor sensors.launch.py

# Simulation mode
ros2 launch perceptor launch_sim.launch.py
```

### Manual Control & Fast Mode Testing

**Basic Control**:
1. **Pair Nintendo Pro Controller** via Bluetooth
2. **Hold Y button** (deadman switch) for safety
3. **Move left stick** while holding Y:
   - **Up/Down**: Forward/backward
   - **Left/Right**: Rotation

**Speed Modes**:
- **Slow Mode**: 0.2 m/s linear, 1.2 rad/s angular (default)
- **Fast Mode**: 0.4 m/s linear, 2.4 rad/s angular

**Fast Mode Safety Precautions**:
- Ensure clear, open area (minimum 3m radius)
- Test slow mode first to verify responsiveness
- Keep emergency stop ready (release Y button)
- Start with gentle stick movements

**Note**: Both speed modes are automatically available through the joy_teleop configuration.

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

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- Built on the [create_robot](https://github.com/AutonomyLab/create_robot) package
- Uses [rplidar_ros](https://github.com/Slamtec/rplidar_ros) for LiDAR integration
- Inspired by the ROS2 navigation and SLAM community
