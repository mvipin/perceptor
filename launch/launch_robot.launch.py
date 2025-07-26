"""
Real Robot Launch Configuration - HEADLESS PI 5 MODE

This launch file configures and starts the complete system for running the Perceptor
physical robot on a Raspberry Pi 5 in headless mode (no graphics/RViz).

HEADLESS PI 5 MODE CHARACTERISTICS:
- Platform: Raspberry Pi 5 (headless operation, no display)
- Robot Base: Physical Create 2 robot base with create_driver interface
- Control System: create_driver topics (cmd_vel, odom, joint_states from create_driver)
- Sensors: Physical LiDAR and Camera (when launched separately)
- Hardware: Real Create robot motors, encoders, and sensors
- Graphics: None (RViz and visualization run on remote host)
- Performance: Optimized for Pi 5 ARM64 architecture

This configuration is specifically designed for headless operation on Raspberry Pi 5,
with minimal resource usage and no graphics components. Visualization tools like RViz
should be run separately on a host computer with display capabilities.

Key components launched:
1. Robot State Publisher (RSP): Publishes Create robot's kinematic structure
2. create_driver: Interfaces with physical Create 2 robot hardware
3. Twist Mux: Arbitrates between multiple velocity command sources
4. Nintendo Pro Controller: Bluetooth joystick control (optional)
5. RPLidar A1M8: 360-degree laser scanner (optional)
6. USB Camera: Computer vision sensor (optional)

Launch Parameters:
- enable_joystick: true/false (default: true)
- enable_lidar: true/false (default: true)
- enable_camera: true/false (default: true)

Visualization (run on host computer):
- RViz: ros2 run rviz2 rviz2 -d src/perceptor/config/main.rviz
- rqt tools: ros2 run rqt_graph rqt_graph

Usage:
    # Complete robot system (default)
    ros2 launch perceptor launch_robot.launch.py

    # Robot base only (no sensors/joystick)
    ros2 launch perceptor launch_robot.launch.py enable_joystick:=false enable_lidar:=false enable_camera:=false

    # Custom configuration
    ros2 launch perceptor launch_robot.launch.py enable_lidar:=true enable_camera:=false

    # Visualization on host computer
    ros2 run rviz2 rviz2 -d src/perceptor/config/main.rviz
"""

import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.conditions import IfCondition

from launch_ros.actions import Node



def generate_launch_description():
    """
    Generate the launch description for the real robot system.

    Returns:
        LaunchDescription: Complete launch configuration for physical robot operation
    """

    # Launch Arguments
    enable_joystick_arg = DeclareLaunchArgument(
        'enable_joystick',
        default_value='true',
        description='Enable joystick/gamepad control via Bluetooth'
    )

    enable_lidar_arg = DeclareLaunchArgument(
        'enable_lidar',
        default_value='true',
        description='Enable RPLidar A1M8 laser scanner'
    )

    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='true',
        description='Enable USB camera for computer vision'
    )

    # Package name configuration
    # This must match the actual package name for proper resource location
    package_name = 'perceptor'

    # Robot State Publisher (RSP) Launch
    # Includes the robot state publisher configuration with real robot settings
    # Functionality: Publishes Create robot's kinematic structure and transforms
    # Dependencies: Requires robot URDF/xacro files in the description folder
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]),
        launch_arguments={
            # Real robot mode: Uses system time instead of simulation time
            'use_sim_time': 'false',
            # Disable ros2_control: Use create_driver topics instead
            'use_ros2_control': 'false'
        }.items()
    )

    # Create Robot Driver Node
    # Direct node launch instead of including XML launch file
    # Functionality: Provides cmd_vel, odom, joint_states from Create robot
    # Dependencies: Requires physical Create robot and create_driver package
    create_driver_config = os.path.join(
        get_package_share_directory('create_bringup'),
        'config',
        'default.yaml'
    )

    create_driver = Node(
        package='create_driver',
        executable='create_driver',
        name='create_driver',
        output='screen',
        parameters=[
            create_driver_config,
            {'robot_model': 'CREATE_2'}
        ]
    )

    # Joystick Control (Optional)
    # Bluetooth gamepad support for manual robot control
    # Can be disabled with: enable_joystick:=false
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'joystick.launch.py'
        )]),
        launch_arguments={'use_sim_time': 'false'}.items(),
        condition=IfCondition(LaunchConfiguration('enable_joystick'))
    )

    # LiDAR Sensor (Optional)
    # RPLidar A1M8 360-degree laser scanner using official rplidar_ros launch
    # Can be disabled with: enable_lidar:=false
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_a1_launch.py'
        )]),
        condition=IfCondition(LaunchConfiguration('enable_lidar'))
    )

    # Camera Sensor (Optional)
    # USB camera for computer vision and ball tracking
    # Can be disabled with: enable_camera:=false
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'camera.launch.py'
        )]),
        condition=IfCondition(LaunchConfiguration('enable_camera'))
    )

    # Twist Multiplexer Configuration
    # Path to configuration file defining command source priorities and timeouts
    twist_mux_params = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'twist_mux.yaml'
    )

    # Twist Mux Node - Command arbitration system
    # Manages multiple velocity command sources (joystick, navigation, ball tracker)
    # Functionality: Prioritizes and switches between different control inputs
    # Dependencies: Requires twist_mux.yaml configuration file
    twist_mux = Node(
        package="twist_mux",              # Twist multiplexer package
        executable="twist_mux",           # Main arbitration executable
        parameters=[twist_mux_params],    # Priority and timeout configuration
        remappings=[
            # Output to create_driver cmd_vel topic (real robot mode)
            ('/cmd_vel_out', '/cmd_vel')
        ]
    )



    # Launch Description Assembly
    # Integrated robot system with optional sensor components
    return LaunchDescription([
        # Launch arguments
        enable_joystick_arg,           # Joystick enable/disable option
        enable_lidar_arg,              # LiDAR enable/disable option
        enable_camera_arg,             # Camera enable/disable option

        # Core robot components
        rsp,                           # Robot state publisher (Create robot model)
        create_driver,                 # Create robot hardware driver
        twist_mux,                     # Command arbitration (immediate start)

        # Optional components (controlled by launch arguments)
        joystick,                      # Bluetooth gamepad control (conditional)
        lidar,                         # RPLidar A1M8 laser scanner (conditional)
        camera,                        # USB camera (conditional)

        # Note: Visualization tools run on host computer:
        # - RViz: ros2 run rviz2 rviz2 -d src/perceptor/config/main.rviz
        # - rqt_graph: ros2 run rqt_graph rqt_graph
    ])
