"""
Robot State Publisher (RSP) Launch Configuration

This launch file configures and starts the Robot State Publisher for the perceptor
robot. The RSP is a fundamental ROS 2 node that publishes the robot's kinematic
structure and coordinate frame transformations based on the robot's URDF description.

The Robot State Publisher reads the robot's URDF (Unified Robot Description Format)
file, which contains the complete description of the robot's physical structure,
including links, joints, sensors, and their relationships. It then publishes this
information as TF (Transform) data that other nodes use for coordinate transformations.

Key responsibilities:
1. URDF Processing: Parses and validates the robot description
2. Transform Publishing: Publishes static transforms between robot links
3. Joint State Integration: Combines joint states with kinematic model
4. Coordinate Frame Management: Maintains robot's coordinate system hierarchy

The launch file supports both simulation and real robot modes, with configurable
ros2_control integration for hardware interface management.

Node launched:
- robot_state_publisher: Core kinematic model publisher
  * Subscribes: /joint_states (sensor_msgs/JointState) - joint positions/velocities
  * Publishes: /tf_static (tf2_msgs/TFMessage) - static transform tree
  * Publishes: /robot_description (std_msgs/String) - robot URDF description
  * Parameters: robot_description, use_sim_time

Command-line equivalent:
    ros2 run robot_state_publisher robot_state_publisher --ros-args \
        -p robot_description:="$(xacro /path/to/robot.urdf.xacro use_ros2_control:=true sim_mode:=false)" \
        -p use_sim_time:=false

Usage:
    ros2 launch perceptor rsp.launch.py \
        [use_sim_time:=true/false] \
        [use_ros2_control:=true/false]
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import xacro


def generate_launch_description():
    """
    Generate the launch description for the Robot State Publisher.

    Returns:
        LaunchDescription: Complete launch configuration for robot kinematic model
    """

    # Launch configuration parameters for runtime customization
    # These parameters control the robot description processing and time synchronization
    use_sim_time = LaunchConfiguration('use_sim_time')        # Time synchronization mode
    use_ros2_control = LaunchConfiguration('use_ros2_control') # Hardware interface mode

    # Robot Description Processing
    # Locates and processes the robot's URDF/xacro description file
    pkg_path = os.path.join(get_package_share_directory('perceptor'))
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')

    # Dynamic URDF Generation using xacro
    # Processes the xacro file with runtime parameters to generate final URDF
    # This allows conditional inclusion of ros2_control hardware interfaces
    # and simulation-specific configurations
    robot_description_config = Command([
        'xacro ', xacro_file,                    # Base xacro file path
        ' use_ros2_control:=', use_ros2_control, # Enable/disable hardware interface
        ' use_sim_time:=', use_sim_time          # Simulation vs real robot mode
    ])

    # Robot State Publisher Node Parameters
    # Combines the processed robot description with time synchronization settings
    params = {
        'robot_description': ParameterValue(robot_description_config, value_type=str),  # Complete robot kinematic model
        'use_sim_time': use_sim_time                    # Time synchronization mode
    }

    # Robot State Publisher Node
    # Core node that publishes robot kinematic structure and transforms
    # Functionality: Converts URDF to TF tree and publishes static transforms
    # Dependencies: Requires valid URDF description and joint state data
    node_robot_state_publisher = Node(
        package='robot_state_publisher',    # Robot state publisher package
        executable='robot_state_publisher', # Main RSP executable
        output='screen',                    # Display output in terminal
        parameters=[params]                 # Robot description and time settings
    )

    # Launch Description Assembly
    # Combines launch arguments and robot state publisher node
    return LaunchDescription([
        # Simulation Time Launch Argument
        # Default: false (real robot mode with system time)
        # Valid values: 'true' for simulation, 'false' for real robot
        # Impact: Synchronizes robot model with simulation or system clock
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true, system time if false'
        ),

        # ROS 2 Control Launch Argument
        # Default: true (enable hardware interface)
        # Valid values: 'true' to include ros2_control, 'false' for basic model only
        # Impact: Includes/excludes hardware interface definitions in robot description
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Include ros2_control hardware interface in robot description'
        ),

        node_robot_state_publisher  # Robot kinematic model publisher
    ])
