"""
RPLidar A1M8 Launch Configuration

IMPORTANT: This launch file is deprecated. Use the official rplidar_ros launch instead:
    ros2 launch rplidar_ros rplidar_a1_launch.py

This wrapper launch file includes the official RPLidar A1 launch file from the
rplidar_ros package, which is the recommended approach for RPLidar A1M8 models.

RPLIDAR A1M8 SPECIFICATIONS:
- Model: RPLidar A1M8 (360-degree 2D laser scanner)
- Range: 0.15m to 12m
- Scan Rate: 10Hz
- Sample Rate: 4 KHz (Express mode)
- Communication: USB serial at 115200 baud
- Connection: /dev/ttyUSB0

CONFIGURATION OPTIMIZED FOR A1M8:
- Serial Port: /dev/ttyUSB0
- Baud Rate: 115200 (critical for A1M8)
- Scan Mode: Sensitivity (optimal for A1M8)
- Frame ID: laser
- Angle Compensation: Enabled

The official rplidar_ros launch provides:
- Proper A1M8 hardware initialization
- Optimized scan mode configuration
- Reliable serial communication
- Standard ROS2 topic publishing (/scan)

Node launched (via official launch):
- rplidar_node: Official RPLidar driver
  * Publishes: /scan (sensor_msgs/LaserScan)
  * Hardware: RPLidar A1M8 via USB serial

Usage:
    # Recommended (direct official launch):
    ros2 launch rplidar_ros rplidar_a1_launch.py

    # Alternative (via this wrapper):
    ros2 launch perceptor rplidar.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Generate the launch description for the RPLidar A1M8 sensor.

    Uses the official rplidar_ros A1 launch file with proper configuration
    for the A1M8 model (115200 baud, Sensitivity scan mode).

    Returns:
        LaunchDescription: Complete launch configuration for RPLidar A1M8 operation
    """

    # Use the official RPLidar A1 launch file from rplidar_ros package
    # This is the recommended approach for A1M8 models
    rplidar_a1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_a1_launch.py'
        )]),
        launch_arguments={
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': '115200',
            'frame_id': 'laser',
            'inverted': 'false',
            'angle_compensate': 'true',
            'scan_mode': 'Sensitivity'  # Optimal for A1M8
        }.items()
    )

    return LaunchDescription([
        rplidar_a1_launch
    ])
