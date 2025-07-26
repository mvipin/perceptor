"""
Ball Tracker Launch Configuration

This launch file configures and starts the ball tracking system for the perceptor robot.
It launches the ball_tracker package with appropriate parameters for either simulation or
real robot operation, enabling the robot to detect, track, and follow a colored ball.

The launch file automatically selects the correct parameter configuration based on the
sim_mode argument, using different HSV color thresholds and tracking parameters optimized
for simulation vs real-world camera conditions.

Command-line equivalent:
    For simulation mode:
        ros2 launch ball_tracker ball_tracker.launch.py \
            params_file:=<path_to_sim_params> \
            image_topic:=/camera/image_raw \
            cmd_vel_topic:=/cmd_vel_tracker \
            enable_3d_tracker:=true

    For real robot mode:
        ros2 launch ball_tracker ball_tracker.launch.py \
            params_file:=<path_to_robot_params> \
            image_topic:=/camera/image_raw \
            cmd_vel_topic:=/cmd_vel_tracker \
            enable_3d_tracker:=true

Usage:
    ros2 launch perceptor ball_tracker.launch.py [sim_mode:=true/false]
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    # Package name for this robot configuration
    my_package_name='perceptor'

    # Launch configuration for simulation mode selection
    # Controls whether to use simulation or real robot parameters
    sim_mode = LaunchConfiguration('sim_mode')

    # Declare the sim_mode launch argument
    # Default: false (real robot mode)
    # Valid values: 'true' for simulation, 'false' for real robot
    # Impact: Selects appropriate HSV color thresholds and tracking parameters
    sim_mode_dec = DeclareLaunchArgument(
        'sim_mode',
        default_value='false',
        description='Use simulation parameters if true, real robot parameters if false'
    )

    # Parameter file paths for different operating modes
    # Simulation parameters: Optimized for Gazebo lighting and camera simulation
    tracker_params_sim = os.path.join(
        get_package_share_directory(my_package_name),
        'config',
        'ball_tracker_params_sim.yaml'
    )

    # Real robot parameters: Optimized for physical camera and real-world lighting
    tracker_params_robot = os.path.join(
        get_package_share_directory(my_package_name),
        'config',
        'ball_tracker_params_robot.yaml'
    )

    # Dynamic parameter file selection based on sim_mode
    # Uses Python expression to conditionally select the appropriate parameter file
    params_path = PythonExpression([
        '"', tracker_params_sim, '" if "true" == "', sim_mode, '" else "', tracker_params_robot, '"'
    ])

    # Include the main ball_tracker launch file with configured parameters
    # This launches the complete ball tracking pipeline including:
    # - detect_ball node: HSV-based ball detection in camera images
    # - detect_ball_3d node: 3D position estimation using camera geometry
    # - follow_ball node: Control logic for following the detected ball
    tracker_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ball_tracker'), 'launch', 'ball_tracker.launch.py')]),
        launch_arguments={
            # Parameter file: Selected based on sim_mode (simulation vs real robot)
            'params_file': params_path,

            # Image topic: Camera feed for ball detection
            # Expects sensor_msgs/Image messages from the robot's camera
            'image_topic': '/camera/image_raw',

            # Command velocity topic: Output for robot movement commands
            # Publishes geometry_msgs/Twist messages for ball following behavior
            'cmd_vel_topic': '/cmd_vel_tracker',

            # Enable 3D tracking: Provides depth estimation for better following
            # Uses camera intrinsics and ball size to estimate distance
            'enable_3d_tracker': 'true'
        }.items()
    )

    return LaunchDescription([
        sim_mode_dec,      # Launch argument declaration
        tracker_launch,    # Ball tracker nodes and configuration
    ])
