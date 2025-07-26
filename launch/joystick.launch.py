"""
Nintendo Pro Controller Launch Configuration

This launch file configures the Nintendo Switch Pro Controller for manual teleoperation
of the Perceptor robot. It provides safe, responsive control with proper deadman button
safety features and dual speed modes.

CONTROLLER CONFIGURATION:
- Hardware: Nintendo Switch Pro Controller (Bluetooth)
- Deadman Button: Y button (must be held for movement)
- Control Stick: Left analog stick (up/down = linear, left/right = angular)
- Speed Modes: Slow (0.2 m/s) and Fast (0.4 m/s) automatically available
- Safety: Robot stops immediately when Y button is released

SYSTEM ARCHITECTURE:
Nintendo Pro Controller → joy_node → joy_teleop → /cmd_vel_joy → twist_mux → create_driver

The system consists of two main components:
1. Joy node: Reads raw joystick input from Nintendo Pro Controller
2. Joy teleop node: Converts joystick input to velocity commands with safety features

This configuration uses the consolidated pro_controller.yaml configuration file
and publishes to /cmd_vel_joy to avoid feedback loops with twist_mux.

Nodes launched:
- joy_node: Hardware interface for Nintendo Pro Controller (auto-detection)
- joy_teleop: Joystick-to-velocity command converter with deadman button safety

Safety Features:
- Deadman button prevents accidental movement
- Command timeout (0.5s) prevents runaway behavior
- Topic isolation prevents feedback loops
- Priority override allows manual control over autonomous systems

Usage:
    ros2 launch perceptor joystick.launch.py [use_sim_time:=true/false]
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Generate the launch description for the joystick control system.

    Returns:
        LaunchDescription: Complete launch configuration for joystick-based robot control
    """

    # Launch configuration for simulation time synchronization
    # Controls whether nodes use simulation time or system time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Create our own joy_teleop setup with proper topic remapping
    # This avoids the feedback loop issue by publishing to /cmd_vel_joy instead of /cmd_vel

    # Path to the Pro Controller configuration (consolidated single config file)
    pro_controller_config = os.path.join(
        get_package_share_directory('perceptor'),
        'config',
        'pro_controller.yaml'
    )

    # Joy Node - Hardware interface for joystick/gamepad input
    # Note: Auto-detection works better than specifying /dev/input/js0 for Pro Controller
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[
            {'deadzone': 0.2},
            {'autorepeat_rate': 20.0},
            {'use_sim_time': use_sim_time}
        ],
    )

    # Joy Teleop Node - Convert joystick input to velocity commands
    # IMPORTANT: Remap /cmd_vel to /cmd_vel_joy to avoid feedback loop with twist_mux
    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[
            pro_controller_config,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/cmd_vel', '/cmd_vel_joy')  # Publish to cmd_vel_joy for twist_mux
        ]
    )

    # Note: twist_stamper removed for real robot mode
    # create_driver accepts geometry_msgs/Twist directly on /cmd_vel topic
    # No need for TwistStamped conversion when using create_driver

    return LaunchDescription([
        # Launch argument declaration
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true, system time if false'
        ),

        joy_node,           # Hardware joystick interface
        joy_teleop_node,    # Joystick-to-velocity converter (publishes to /cmd_vel_joy)
    ])