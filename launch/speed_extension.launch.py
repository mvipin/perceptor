#!/usr/bin/env python3
"""
Speed Limit Extension Launch - Perceptor Robot

This launch file provides a modular speed limit extension that can be added
to any existing Nav2 navigation setup. It launches only the speed filter-specific
components without interfering with the core localization and navigation stack.

This modular approach maintains architectural consistency with the standard
Nav2 pattern of separate localization and navigation launch files, while
providing speed limit functionality as an optional extension.

Features:
- Modular design: Works with existing navigation setups
- Independent lifecycle management for speed filter components
- No interference with core navigation functionality
- Easy to enable/disable speed limits
- Compatible with standard Nav2 launch architecture
- Can be used simultaneously with keepout zones

Usage:
    # First launch your standard navigation stack:
    ros2 launch perceptor localization_launch.py map:=home.yaml
    ros2 launch perceptor navigation_launch.py
    
    # Then add speed limits:
    ros2 launch perceptor speed_extension.launch.py speed_mask:=speed_mask.yaml

    # Optional: Also add keepout zones:
    ros2 launch perceptor keepout_extension.launch.py keepout_mask:=keepout_mask.yaml

Arguments:
    speed_mask: Path to the speed zone mask YAML file
    use_sim_time: Use simulation time (default: false)
    autostart: Automatically start speed filter lifecycle nodes (default: true)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    perceptor_dir = get_package_share_directory('perceptor')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    speed_mask_file = LaunchConfiguration('speed_mask')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', 
        default_value='true',
        description='Automatically startup the speed filter components')

    declare_speed_mask_cmd = DeclareLaunchArgument(
        'speed_mask',
        default_value=os.path.join(perceptor_dir, 'maps', 'speed_mask.yaml'),
        description='Full path to speed zone mask yaml file to load')

    # Speed Filter Info Server for speed limit zones
    speed_filter_info_server = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='speed_filter_info_server',  # Unique name to avoid conflict with keepout filter
        output='screen',
        parameters=[
            os.path.join(perceptor_dir, 'config', 'speed_filter_info.yaml'),
            {'use_sim_time': use_sim_time}
        ])

    # Map Server for speed mask (with unique node name to avoid conflicts)
    speed_mask_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='speed_mask_server',  # Unique name to avoid conflict with other map servers
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': speed_mask_file,
            'topic_name': 'speed_filter_mask',
            'frame_id': 'map',
            'publish_frequency': 1.0  # Publish at 1Hz for continuous updates
        }])

    # Lifecycle manager for speed filter-specific nodes
    speed_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='speed_lifecycle_manager',  # Unique name to avoid conflict with other lifecycle managers
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['speed_filter_info_server', 'speed_mask_server']
        }])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_speed_mask_cmd)

    # Add the speed extension nodes
    ld.add_action(speed_filter_info_server)
    ld.add_action(speed_mask_server)
    ld.add_action(speed_lifecycle_manager)

    return ld
