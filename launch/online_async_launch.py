"""
SLAM Toolbox Online Async Launch Configuration

This launch file configures and starts the SLAM Toolbox in online asynchronous mode
for simultaneous localization and mapping (SLAM). It enables the robot to build a
map of an unknown environment while simultaneously determining its location within
that map using sensor data from laser scanners.

The online async mode is optimized for real-time operation, processing sensor data
as it arrives while maintaining good computational performance. This mode is ideal
for active mapping sessions where the robot is exploring and building maps in real-time.

Key features:
- Real-time map building and localization
- Asynchronous processing for better performance
- Loop closure detection for map consistency
- Dynamic parameter validation and fallback
- Configurable for both simulation and real robot operation

Node launched:
- slam_toolbox (async_slam_toolbox_node): Core SLAM processing engine
  * Subscribes: /scan (sensor_msgs/LaserScan) - laser scan data
  * Subscribes: /tf - transform data for sensor positioning
  * Publishes: /map (nav_msgs/OccupancyGrid) - generated map
  * Publishes: /tf - map to odom transform
  * Services: Various SLAM services for map saving, loading, etc.

Command-line equivalent:
    ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
        --params-file <mapper_params_online_async.yaml> \
        -p use_sim_time:=true

Usage:
    ros2 launch perceptor online_async_launch.py \
        [use_sim_time:=true/false] \
        [params_file:=<custom_params.yaml>]
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import HasNodeParams


def generate_launch_description():
    """
    Generate the launch description for SLAM Toolbox online async mode.

    Returns:
        LaunchDescription: Complete launch configuration for real-time SLAM
    """

    # Launch configuration parameters
    use_sim_time = LaunchConfiguration('use_sim_time')  # Time synchronization mode
    params_file = LaunchConfiguration('params_file')    # SLAM parameters file

    # Default SLAM parameters file path
    # Contains algorithm settings, sensor configurations, and performance tuning
    default_params_file = os.path.join(
        get_package_share_directory("perceptor"),
        'config',
        'mapper_params_online_async.yaml'
    )

    # Simulation Time Launch Argument
    # Default: true (simulation mode)
    # Valid values: 'true' for simulation, 'false' for real robot
    # Impact: Synchronizes SLAM processing with simulation or system time
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock if true, system clock if false'
    )

    # Parameters File Launch Argument
    # Default: mapper_params_online_async.yaml (optimized for online async SLAM)
    # Valid values: Path to any valid SLAM Toolbox parameter file
    # Impact: Configures SLAM algorithm behavior, sensor settings, and performance
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the SLAM Toolbox parameters file for algorithm configuration'
    )

    # Parameter File Validation
    # Checks if the provided parameter file contains slam_toolbox configuration
    # This prevents launch failures due to missing or incorrect parameter files
    has_node_params = HasNodeParams(
        source_file=params_file,
        node_name='slam_toolbox'
    )

    # Dynamic Parameter File Selection
    # Uses provided file if valid, otherwise falls back to default configuration
    # Ensures SLAM node always has valid parameters to prevent startup failures
    actual_params_file = PythonExpression([
        '"', params_file, '" if ', has_node_params,
        ' else "', default_params_file, '"'
    ])

    # Parameter File Fallback Notification
    # Logs when the system falls back to default parameters due to invalid file
    # Helps with debugging parameter file issues during development
    log_param_change = LogInfo(
        msg=[
            'Provided params_file ', params_file,
            ' does not contain slam_toolbox parameters. Using default: ',
            default_params_file
        ],
        condition=UnlessCondition(has_node_params)
    )

    # SLAM Toolbox Async Node
    # Core SLAM processing engine for simultaneous localization and mapping
    # Functionality: Builds maps while tracking robot pose using laser scan data
    # Dependencies: Requires laser scan data (/scan) and transform data (/tf)
    start_async_slam_toolbox_node = Node(
        parameters=[
            actual_params_file,              # SLAM algorithm configuration
            {'use_sim_time': use_sim_time}   # Time synchronization setting
        ],
        package='slam_toolbox',              # SLAM Toolbox package
        executable='async_slam_toolbox_node', # Asynchronous SLAM executable
        name='slam_toolbox',                 # Node name for identification
        output='screen'                      # Display output in terminal
    )

    # Launch Description Assembly
    # Builds the complete launch configuration with proper parameter validation
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)  # Time synchronization argument
    ld.add_action(declare_params_file_cmd)        # Parameters file argument
    ld.add_action(log_param_change)               # Parameter fallback notification
    ld.add_action(start_async_slam_toolbox_node)  # SLAM processing node

    return ld
