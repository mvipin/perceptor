"""
Sensor Launch Configuration - HEADLESS PI 5 MODE

This launch file starts all sensors for the Perceptor robot running on Raspberry Pi 5.
It's designed to be launched separately from the base robot for modularity and easier
debugging. This allows you to start/stop sensors independently without affecting
the base robot operation.

SENSOR CONFIGURATION:
- LiDAR: RPLidar A1M8 via USB (360-degree laser scanning, 115200 baud)
- Camera: USB camera via V4L2 driver (computer vision)
- Frame IDs: Properly configured to match robot URDF
- Performance: Optimized for Pi 5 ARM64 architecture

This modular approach allows:
1. Independent sensor debugging and testing
2. Selective sensor activation based on mission requirements
3. Better resource management on Pi 5
4. Easier troubleshooting of individual sensor issues

Sensors launched:
1. RPLidar A1M8: 360-degree laser scanner for navigation and SLAM
2. USB Camera: Visual input for computer vision and ball tracking

Usage:
    # Start all sensors (modular approach)
    ros2 launch perceptor sensors.launch.py

    # Or start individually:
    ros2 launch perceptor rplidar.launch.py  # Wrapper for official launch
    ros2 launch perceptor camera.launch.py

    # Or use official LiDAR launch directly:
    ros2 launch rplidar_ros rplidar_a1_launch.py

Prerequisites:
- RPLidar connected via USB
- USB camera connected and accessible
- Proper device permissions configured
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    """
    Generate the launch description for all robot sensors.

    Returns:
        LaunchDescription: Complete launch configuration for sensor operation
    """

    # Package name configuration
    package_name = 'perceptor'

    # RPLidar Launch
    # 360-degree laser scanner for navigation, SLAM, and obstacle avoidance
    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rplidar.launch.py'
        )])
    )

    # Camera Launch  
    # USB camera for computer vision and ball tracking
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'camera.launch.py'
        )])
    )

    # Launch Description Assembly
    # All sensors for complete robot perception
    return LaunchDescription([
        rplidar,                       # RPLidar laser scanner
        camera,                        # USB camera
    ])
