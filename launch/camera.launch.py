"""
Camera Launch Configuration

This launch file configures and starts the V4L2 camera node for the perceptor robot.
It initializes a USB camera using the Video4Linux2 (V4L2) driver, providing camera
image data for computer vision applications like ball tracking, navigation, and
autonomous operation.

The camera node publishes raw image data and camera info messages that can be used
by other nodes for image processing, object detection, and visual feedback.

Node launched:
- v4l2_camera_node: Interfaces with USB cameras using V4L2 driver
  * Publishes: /camera/image_raw (sensor_msgs/Image)
  * Publishes: /camera/camera_info (sensor_msgs/CameraInfo)
  * Provides: Camera calibration and image streaming capabilities

Command-line equivalent:
    ros2 run v4l2_camera v4l2_camera_node \
        --ros-args \
        -r __ns:=/camera \
        -p image_size:=[640,480] \
        -p time_per_frame:=[1,6] \
        -p camera_frame_id:=camera_link_optical

Usage:
    ros2 launch perceptor camera.launch.py
"""

import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generate the launch description for the camera system.

    Returns:
        LaunchDescription: Complete launch configuration for the V4L2 camera node
    """

    return LaunchDescription([
        # V4L2 Camera Node
        # Interfaces with USB cameras using the Video4Linux2 driver
        # Provides raw image data and camera information for computer vision tasks
        Node(
            package='v4l2_camera',           # V4L2 camera driver package
            executable='v4l2_camera_node',   # Main camera node executable
            output='screen',                 # Display node output in terminal
            namespace='camera',              # Namespace for camera topics (/camera/*)
            parameters=[{
                # Image resolution configuration
                # Sets the camera capture resolution to 640x480 pixels
                # Valid values: Depends on camera capabilities (common: [320,240], [640,480], [1280,720])
                # Impact: Higher resolution provides more detail but requires more processing power
                'image_size': [640, 480],

                # Frame rate configuration as [numerator, denominator]
                # Current setting: 1/6 = ~0.167 seconds per frame = ~6 FPS
                # Valid values: Depends on camera and resolution (common: [1,30], [1,15], [1,6])
                # Impact: Higher frame rates provide smoother video but increase CPU usage
                'time_per_frame': [1, 6],

                # Camera frame ID for TF transformations
                # Links camera data to the robot's coordinate system
                # Must match the frame defined in the robot's URDF description
                # Used for: 3D vision, navigation, and coordinate transformations
                'camera_frame_id': 'camera_link_optical'
            }]
        )
    ])
