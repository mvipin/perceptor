"""
Localization Launch Configuration

This launch file configures and starts the robot localization system using Nav2's
AMCL (Adaptive Monte Carlo Localization) algorithm. It provides the robot with
the ability to determine its position within a known map using sensor data,
primarily from laser scanners and odometry.

The localization system is essential for autonomous navigation, as it enables
the robot to understand where it is in the environment and track its movement
over time. This forms the foundation for path planning and navigation behaviors.

Key components launched:
1. Map Server: Loads and serves the pre-built map of the environment
2. AMCL: Probabilistic localization using particle filters
3. Lifecycle Manager: Manages the startup and coordination of localization nodes

The system uses a particle filter approach where multiple hypotheses about the
robot's position are maintained and updated based on sensor observations and
motion models.

Nodes launched:
- map_server: Serves static map data for localization reference
- amcl: Adaptive Monte Carlo Localization for pose estimation
- lifecycle_manager_localization: Coordinates localization node lifecycle

Command-line equivalents:
    # Map Server
    ros2 run nav2_map_server map_server --ros-args \
        --params-file <nav2_params.yaml> \
        -p use_sim_time:=false \
        -p yaml_filename:=<map_file.yaml>

    # AMCL Localization
    ros2 run nav2_amcl amcl --ros-args \
        --params-file <nav2_params.yaml> \
        -p use_sim_time:=false

    # Lifecycle Manager
    ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
        -p use_sim_time:=false \
        -p autostart:=true \
        -p node_names:="['map_server', 'amcl']"

Usage:
    ros2 launch perceptor localization_launch.py \
        [map:=<map_file.yaml>] \
        [use_sim_time:=true/false] \
        [params_file:=<nav2_params.yaml>]
"""

# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    """
    Generate the launch description for the localization system.

    Returns:
        LaunchDescription: Complete launch configuration for robot localization
    """

    # Package directory for resource location
    bringup_dir = get_package_share_directory('perceptor')

    # Launch configuration parameters
    # These allow runtime customization of the localization system
    namespace = LaunchConfiguration('namespace')        # Node namespace for multi-robot
    map_yaml_file = LaunchConfiguration('map')         # Path to map file
    use_sim_time = LaunchConfiguration('use_sim_time') # Time synchronization mode
    autostart = LaunchConfiguration('autostart')       # Automatic node activation
    params_file = LaunchConfiguration('params_file')   # Navigation parameters file

    # Lifecycle nodes managed by the lifecycle manager
    # These nodes follow the ROS 2 lifecycle pattern for coordinated startup/shutdown
    lifecycle_nodes = ['map_server', 'amcl']

    # Transform topic remappings for namespace compatibility
    # Maps global transform topics to local namespace to support multi-robot systems
    # This ensures transform data is properly scoped when using namespaces
    remappings = [
        ('/tf', 'tf'),              # Transform data remapping
        ('/tf_static', 'tf_static') # Static transform data remapping
    ]

    # Parameter substitutions for dynamic configuration
    # These values are substituted into the parameter file at runtime
    param_substitutions = {
        'use_sim_time': use_sim_time,      # Simulation vs real-time clock
        'yaml_filename': map_yaml_file     # Map file path for map server
    }

    # Dynamic parameter file generation
    # Creates a temporary parameter file with runtime substitutions applied
    # This allows the same parameter file to work in different configurations
    configured_params = RewrittenYaml(
        source_file=params_file,           # Base parameter file
        root_key=namespace,                # Namespace for parameter scoping
        param_rewrites=param_substitutions, # Runtime value substitutions
        convert_types=True                 # Automatic type conversion
    )

    return LaunchDescription([
        # Environment configuration for immediate log output
        # Ensures log messages are displayed immediately rather than buffered
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        # Namespace Launch Argument
        # Default: '' (no namespace)
        # Valid values: Any valid ROS namespace string
        # Impact: Scopes all nodes under the specified namespace for multi-robot systems
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace for all localization nodes'
        ),

        # Map File Launch Argument
        # Default: turtlebot3_world.yaml (example map)
        # Valid values: Path to any valid map YAML file
        # Impact: Determines the reference map used for localization
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(bringup_dir, 'maps', 'turtlebot3_world.yaml'),
            description='Full path to the map YAML file containing map metadata and image path'
        ),

        # Simulation Time Launch Argument
        # Default: false (real robot mode)
        # Valid values: 'true' for simulation, 'false' for real robot
        # Impact: Synchronizes localization with simulation or system time
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true, system clock if false'
        ),

        # Autostart Launch Argument
        # Default: true (automatic activation)
        # Valid values: 'true' to auto-activate, 'false' for manual activation
        # Impact: Controls whether lifecycle nodes start automatically or require manual activation
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically activate localization nodes on startup'
        ),

        # Parameters File Launch Argument
        # Default: nav2_params.yaml (standard Nav2 configuration)
        # Valid values: Path to any valid Nav2 parameter file
        # Impact: Configures AMCL algorithm parameters, map server settings, etc.
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_dir, 'config', 'nav2_params.yaml'),
            description='Full path to the Nav2 parameters file for localization configuration'
        ),

        # Map Server Node
        # Loads and serves static map data for localization and navigation
        # Functionality: Provides map data via /map topic and map service
        # Dependencies: Requires valid map YAML file and corresponding image file
        Node(
            package='nav2_map_server',      # Nav2 map server package
            executable='map_server',        # Map serving executable
            name='map_server',              # Node name for identification
            output='screen',                # Display output in terminal
            parameters=[configured_params], # Dynamic parameter configuration
            remappings=remappings          # Transform topic remappings
        ),

        # AMCL Localization Node
        # Adaptive Monte Carlo Localization for probabilistic pose estimation
        # Functionality: Estimates robot pose using particle filter algorithm
        # Dependencies: Requires map data, laser scan, and odometry inputs
        Node(
            package='nav2_amcl',           # Nav2 AMCL package
            executable='amcl',             # AMCL localization executable
            name='amcl',                   # Node name for identification
            output='screen',               # Display output in terminal
            parameters=[configured_params], # Dynamic parameter configuration
            remappings=remappings          # Transform topic remappings
        ),

        # Lifecycle Manager for Localization
        # Manages the lifecycle of localization nodes (configure, activate, etc.)
        # Functionality: Coordinates startup/shutdown of map_server and amcl
        # Dependencies: Requires lifecycle nodes to be properly configured
        Node(
            package='nav2_lifecycle_manager',        # Nav2 lifecycle management
            executable='lifecycle_manager',          # Lifecycle coordination executable
            name='lifecycle_manager_localization',   # Unique name for localization manager
            output='screen',                         # Display output in terminal
            parameters=[
                {'use_sim_time': use_sim_time},      # Time synchronization
                {'autostart': autostart},            # Automatic activation control
                {'node_names': lifecycle_nodes}      # List of nodes to manage
            ]
        )
    ])
