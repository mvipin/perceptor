#!/usr/bin/env python3
"""
Localization Launch Configuration - Perceptor Robot

This launch file configures and starts the robot localization system using Nav2's
AMCL (Adaptive Monte Carlo Localization) algorithm. It provides the robot with
the ability to determine its position within a known map using sensor data,
primarily from laser scanners and odometry.

The localization system is essential for autonomous navigation, as it enables
the robot to understand where it is in the environment and track its movement
over time. This forms the foundation for path planning and navigation behaviors.

TECHNICAL IMPLEMENTATION:
This implementation is based on the proven Nav2 Jazzy localization_launch.py
with enhanced documentation for the Perceptor robot project. It uses the same
robust parameter handling and conditional node launching as the official Nav2
distribution to ensure reliability and compatibility.

Key components launched:
1. Map Server: Loads and serves the pre-built map of the environment
2. AMCL: Probabilistic localization using particle filters  
3. Lifecycle Manager: Manages the startup and coordination of localization nodes

The system uses a particle filter approach where multiple hypotheses about the
robot's position are maintained and updated based on sensor observations and
motion models. The implementation supports both regular nodes and composable
nodes for performance optimization.

ALGORITHM DETAILS:
- AMCL uses Monte Carlo localization with adaptive particle filtering
- Particle count dynamically adjusts based on localization confidence
- Sensor models include laser likelihood fields and motion models
- Recovery behaviors handle localization failures and kidnapped robot scenarios

Nodes launched:
- map_server: Serves static map data for localization reference
- amcl: Adaptive Monte Carlo Localization for pose estimation  
- lifecycle_manager_localization: Coordinates localization node lifecycle

Command-line equivalents:
    # Map Server (with map file specified)
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
        map:=<map_file.yaml> \
        [use_sim_time:=true/false] \
        [params_file:=<nav2_params.yaml>] \
        [autostart:=true/false] \
        [use_composition:=true/false]

Arguments:
    map: Full path to map YAML file (REQUIRED)
    use_sim_time: Use simulation time if true (default: false)
    params_file: Full path to Nav2 parameters file
    autostart: Automatically start lifecycle nodes (default: true)
    use_composition: Use composable nodes for performance (default: false)
    namespace: Top-level namespace for multi-robot systems (default: '')
    use_respawn: Respawn nodes if they crash (default: false)
    log_level: ROS logging level (default: info)
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
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution, LaunchConfiguration, NotEqualsSubstitution, PythonExpression
from launch_ros.actions import LoadComposableNodes, SetParameter, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')

    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    lifecycle_nodes = ['map_server', 'amcl']

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map', default_value='', description='Full path to map yaml file to load'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack',
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='False',
        description='Use composed bringup if True',
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name',
        default_value='nav2_container',
        description='the name of conatiner that nodes will load in if use composition',
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            Node(
                condition=IfCondition(
                    EqualsSubstitution(LaunchConfiguration('map'), '')
                ),
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                condition=IfCondition(
                    NotEqualsSubstitution(LaunchConfiguration('map'), '')
                ),
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, {'yaml_filename': map_yaml_file}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'autostart': autostart}, {'node_names': lifecycle_nodes}],
            ),
        ],
    )

    # LoadComposableNode for map server twice depending if we should use the
    # value of map from a CLI or launch default or user defined value in the
    # yaml configuration file. They are separated since the conditions
    # currently only work on the LoadComposableNodes commands and not on the
    # ComposableNode node function itself
    load_composable_nodes = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            LoadComposableNodes(
                target_container=container_name_full,
                condition=IfCondition(
                    EqualsSubstitution(LaunchConfiguration('map'), '')
                ),
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_map_server',
                        plugin='nav2_map_server::MapServer',
                        name='map_server',
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                ],
            ),
            LoadComposableNodes(
                target_container=container_name_full,
                condition=IfCondition(
                    NotEqualsSubstitution(LaunchConfiguration('map'), '')
                ),
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_map_server',
                        plugin='nav2_map_server::MapServer',
                        name='map_server',
                        parameters=[
                            configured_params,
                            {'yaml_filename': map_yaml_file},
                        ],
                        remappings=remappings,
                    ),
                ],
            ),
            LoadComposableNodes(
                target_container=container_name_full,
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_amcl',
                        plugin='nav2_amcl::AmclNode',
                        name='amcl',
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_lifecycle_manager',
                        plugin='nav2_lifecycle_manager::LifecycleManager',
                        name='lifecycle_manager_localization',
                        parameters=[
                            {'autostart': autostart, 'node_names': lifecycle_nodes}
                        ],
                    ),
                ],
            ),
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # Add the actions to launch all of the localiztion nodes
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)

    return ld
