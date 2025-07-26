"""
Navigation Launch Configuration

This launch file configures and starts the complete Nav2 navigation stack for autonomous
robot navigation. It provides path planning, obstacle avoidance, behavior coordination,
and motion control capabilities that enable the robot to navigate safely from one location
to another in a known environment.

The navigation system works in conjunction with localization to provide full autonomous
navigation capabilities. It includes global and local path planners, behavior trees for
decision making, and various recovery behaviors for handling navigation failures.

Key Nav2 components launched:
1. Controller Server: Executes path following and obstacle avoidance
2. Smoother Server: Smooths planned paths for better execution
3. Planner Server: Computes global paths from start to goal
4. Behavior Server: Handles recovery behaviors (backup, spin, wait)
5. BT Navigator: Coordinates navigation using behavior trees
6. Waypoint Follower: Manages sequential waypoint navigation
7. Velocity Smoother: Smooths velocity commands for better control

The system supports both regular nodes and composable nodes for performance optimization.
Composable nodes run in a single process to reduce overhead and improve real-time performance.

Nodes launched:
- controller_server: Path following and local obstacle avoidance
- smoother_server: Path smoothing for better trajectory execution
- planner_server: Global path planning (A*, RRT*, etc.)
- behavior_server: Recovery behaviors and navigation actions
- bt_navigator: Behavior tree-based navigation coordination
- waypoint_follower: Sequential waypoint navigation management
- velocity_smoother: Velocity command smoothing and limiting
- lifecycle_manager_navigation: Coordinates navigation node lifecycle

Command-line equivalents:
    # Controller Server
    ros2 run nav2_controller controller_server --ros-args \
        --params-file <nav2_params.yaml> \
        -r cmd_vel:=cmd_vel_nav

    # Planner Server
    ros2 run nav2_planner planner_server --ros-args \
        --params-file <nav2_params.yaml>

    # Behavior Server
    ros2 run nav2_behaviors behavior_server --ros-args \
        --params-file <nav2_params.yaml>

    # BT Navigator
    ros2 run nav2_bt_navigator bt_navigator --ros-args \
        --params-file <nav2_params.yaml>

    # Lifecycle Manager
    ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
        -p node_names:="['controller_server', 'planner_server', 'behavior_server', 'bt_navigator']"

Usage:
    ros2 launch perceptor navigation_launch.py \
        [use_sim_time:=true/false] \
        [params_file:=<nav2_params.yaml>] \
        [use_composition:=true/false]
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
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    """
    Generate the launch description for the Nav2 navigation stack.

    Returns:
        LaunchDescription: Complete launch configuration for autonomous navigation
    """

    # Package directory for resource location
    bringup_dir = get_package_share_directory('perceptor')

    # Launch configuration parameters for runtime customization
    namespace = LaunchConfiguration('namespace')          # Node namespace for multi-robot
    use_sim_time = LaunchConfiguration('use_sim_time')   # Time synchronization mode
    autostart = LaunchConfiguration('autostart')         # Automatic node activation
    params_file = LaunchConfiguration('params_file')     # Navigation parameters file
    use_composition = LaunchConfiguration('use_composition')  # Composable vs regular nodes
    container_name = LaunchConfiguration('container_name')    # Container name for composable nodes
    container_name_full = (namespace, '/', container_name)    # Full container name with namespace
    use_respawn = LaunchConfiguration('use_respawn')     # Node respawn on failure
    log_level = LaunchConfiguration('log_level')         # Logging verbosity level

    # Navigation lifecycle nodes managed by the lifecycle manager
    # These nodes follow the ROS 2 lifecycle pattern for coordinated startup/shutdown
    # Order matters for proper initialization sequence
    lifecycle_nodes = [
        'controller_server',    # Path following and obstacle avoidance
        'smoother_server',      # Path smoothing for better execution
        'planner_server',       # Global path planning algorithms
        'behavior_server',      # Recovery behaviors and navigation actions
        'bt_navigator',         # Behavior tree coordination and decision making
        'waypoint_follower',    # Sequential waypoint navigation management
        'velocity_smoother'     # Velocity command smoothing and limiting
    ]

    # Transform topic remappings for namespace compatibility
    # Maps global transform topics to local namespace for multi-robot support
    # Required for proper coordinate frame handling in namespaced environments
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart}

    configured_params = RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Use composed bringup if True')

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='nav2_container',
        description='the name of conatiner that nodes will load in if use composition')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings +
                        [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}]),
        ]
    )

    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[configured_params],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            ComposableNode(
                package='nav2_smoother',
                plugin='nav2_smoother::SmootherServer',
                name='smoother_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_waypoint_follower',
                plugin='nav2_waypoint_follower::WaypointFollower',
                name='waypoint_follower',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=[configured_params],
                remappings=remappings +
                           [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[{'use_sim_time': use_sim_time,
                             'autostart': autostart,
                             'node_names': lifecycle_nodes}]),
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    # Add the actions to launch all of the navigation nodes
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)

    return ld
