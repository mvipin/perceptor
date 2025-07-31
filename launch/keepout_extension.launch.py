#!/usr/bin/env python3
"""
Keepout Zone Extension Launch - Perceptor Robot

This launch file provides a modular keepout zone extension that can be added
to any existing Nav2 navigation setup. It launches only the keepout-specific
components without interfering with the core localization and navigation stack.

This modular approach maintains architectural consistency with the standard
Nav2 pattern of separate localization and navigation launch files, while
providing keepout zone functionality as an optional extension.

Features:
- Modular design: Works with existing navigation setups
- Independent lifecycle management for keepout components
- No interference with core navigation functionality
- Easy to enable/disable keepout zones
- Compatible with standard Nav2 launch architecture

Usage:
    # First launch your standard navigation stack:
    ros2 launch perceptor localization_launch.py map:=home.yaml
    ros2 launch perceptor navigation_launch.py
    
    # Then add keepout zones:
    ros2 launch perceptor keepout_extension.launch.py keepout_mask:=keepout_mask.yaml

Arguments:
    keepout_mask: Path to the keepout zone mask YAML file
    use_sim_time: Use simulation time (default: false)
    autostart: Automatically start keepout lifecycle nodes (default: true)
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
    keepout_mask_file = LaunchConfiguration('keepout_mask')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', 
        default_value='true',
        description='Automatically startup the keepout components')

    declare_keepout_mask_cmd = DeclareLaunchArgument(
        'keepout_mask',
        default_value=os.path.join(perceptor_dir, 'maps', 'keepout_mask.yaml'),
        description='Full path to keepout mask yaml file to load')

    # Costmap Filter Info Server for keepout zones
    costmap_filter_info_server = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        parameters=[
            os.path.join(perceptor_dir, 'config', 'keepout_filter_info.yaml'),
            {'use_sim_time': use_sim_time}
        ])

    # Map Server for keepout mask (with unique node name to avoid conflicts)
    keepout_mask_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='keepout_mask_server',  # Unique name to avoid conflict with main map_server
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': keepout_mask_file,
            'topic_name': 'keepout_filter_mask',
            'frame_id': 'map',
            'publish_frequency': 1.0  # Publish at 1Hz for continuous updates
        }])

    # Lifecycle manager for keepout-specific nodes
    keepout_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='keepout_lifecycle_manager',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['costmap_filter_info_server', 'keepout_mask_server']
        }])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_keepout_mask_cmd)

    # Add the keepout extension nodes
    ld.add_action(costmap_filter_info_server)
    ld.add_action(keepout_mask_server)
    ld.add_action(keepout_lifecycle_manager)

    return ld
