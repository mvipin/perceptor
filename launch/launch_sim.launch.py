"""
Simulation Launch Configuration - SIMULATION MODE

This launch file configures and starts the complete simulation environment for the
Perceptor robot using Gazebo (Ignition Gazebo) with the integrated Create robot base.

SIMULATION MODE CHARACTERISTICS:
- Robot Base: Create 2 robot base (wheel_separation=0.235m, wheel_radius=0.036m)
- Control System: Gazebo differential drive plugin (simplified control)
- Sensors: LiDAR (200mm above base_link), Camera (75mm forward, 100mm up)
- Physics: Gazebo simulation with realistic Create robot specifications
- Topics: Standard robot topics (/cmd_vel, /joint_states, /odom)

The simulation provides a safe and repeatable environment for testing autonomous
behaviors, navigation algorithms, and sensor integration without requiring physical
hardware. It includes realistic physics, sensor simulation, and ROS 2 integration.

Key components launched:
1. Robot State Publisher: Create robot kinematic model and transforms
2. Joystick Control: Nintendo Pro Controller teleoperation interface
3. Twist Multiplexer: Command arbitration system
4. Gazebo Simulation: Physics engine and world environment
5. Robot Spawning: Places Create robot model in simulation world
6. Controllers: Differential drive and joint state management (ros2_control)
7. ROS-Gazebo Bridge: Communication between ROS 2 and Gazebo

Command-line equivalents:
    # Robot State Publisher
    ros2 launch perceptor rsp.launch.py use_sim_time:=true use_ros2_control:=true

    # Joystick Control
    ros2 launch perceptor joystick.launch.py use_sim_time:=true

    # Twist Multiplexer
    ros2 run twist_mux twist_mux --ros-args \
        --params-file <path_to_twist_mux.yaml> \
        -p use_sim_time:=true \
        -r cmd_vel_out:=cmd_vel

    # Gazebo Simulation
    ros2 launch ros_gz_sim gz_sim.launch.py \
        gz_args:="-r -v4 <world_file>" \
        on_exit_shutdown:=true

    # Robot Spawning
    ros2 run ros_gz_sim create -topic robot_description -name my_bot -z 0.1

    # No controllers needed - using direct Gazebo differential drive plugin

    # ROS-Gazebo Bridges
    ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=<bridge_config>
    ros2 run ros_gz_image image_bridge /camera/image_raw

Usage:
    ros2 launch perceptor launch_sim.launch.py [world:=<world_file>]
"""

import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node



def generate_launch_description():
    """
    Generate the launch description for the simulation environment.

    Returns:
        LaunchDescription: Complete launch configuration for Gazebo simulation
    """

    # Package name configuration
    # This must match the actual package name for proper resource location
    package_name = 'perceptor'

    # Robot State Publisher (RSP) Launch - Simulation Mode
    # Includes the robot state publisher with simulation time enabled
    # Functionality: Publishes robot's kinematic structure with sim time synchronization
    # Dependencies: Requires robot URDF/xacro files with ros2_control configuration
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]),
        launch_arguments={
            # Simulation mode: Uses Gazebo's simulation time for synchronization
            'use_sim_time': 'true',
            # Disable ros2_control: Use Create robot's built-in Gazebo differential drive plugin
            'use_ros2_control': 'false'
        }.items()
    )

    # Joystick Control Launch - Simulation Mode
    # Includes joystick control system with simulation time synchronization
    # Functionality: Enables manual teleoperation during simulation
    # Dependencies: Requires physical joystick and joystick configuration
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'joystick.launch.py'
        )]),
        launch_arguments={
            # Simulation time: Synchronizes joystick commands with simulation clock
            'use_sim_time': 'true'
        }.items()
    )

    # Twist Multiplexer Configuration
    # Path to configuration file defining command source priorities and timeouts
    twist_mux_params = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'twist_mux.yaml'
    )

    # Twist Mux Node - Command arbitration for simulation
    # Manages multiple velocity command sources in simulation environment
    # Functionality: Prioritizes commands from joystick, navigation, autonomous systems
    # Dependencies: Requires twist_mux.yaml configuration file
    twist_mux = Node(
        package="twist_mux",              # Twist multiplexer package
        executable="twist_mux",           # Main arbitration executable
        parameters=[
            twist_mux_params,             # Priority and timeout configuration
            {'use_sim_time': True}        # Simulation time synchronization
        ],
        remappings=[
            # Output to Gazebo differential drive plugin
            ('/cmd_vel_out', '/cmd_vel')
        ]
    )

    # Default World Configuration
    # Path to the default simulation world file
    # Can be overridden by the 'world' launch argument
    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'empty.world'
    )

    # World launch configuration
    # Allows dynamic selection of simulation world at launch time
    world = LaunchConfiguration('world')

    # World Launch Argument Declaration
    # Default: empty.world (basic environment with ground plane)
    # Valid values: Any .world file in the worlds/ directory
    # Impact: Determines the simulation environment (obstacles, lighting, etc.)
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Path to the Gazebo world file to load for simulation'
    )

    # Gazebo Simulation Launch
    # Starts the Ignition Gazebo physics simulation environment
    # Functionality: Provides physics engine, rendering, and sensor simulation
    # Dependencies: Requires ros_gz_sim package and world file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={
            # Gazebo arguments: -r (run immediately), -v4 (verbose level 4)
            'gz_args': ['-r -v4 ', world],
            # Shutdown ROS when Gazebo exits to prevent orphaned processes
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Robot Entity Spawner
    # Creates the robot model instance in the Gazebo simulation world
    # Functionality: Reads robot_description and instantiates it in simulation
    # Dependencies: Requires robot_state_publisher to be running with valid URDF
    spawn_entity = Node(
        package='ros_gz_sim',           # Gazebo-ROS integration package
        executable='create',            # Entity creation executable
        arguments=[
            '-topic', 'robot_description',  # Source topic for robot model
            '-name', 'my_bot',              # Name of robot instance in simulation
            '-z', '0.1'                     # Initial Z position (slightly above ground)
        ],
        output='screen'                 # Display spawning output in terminal
    )

    # No controller spawners needed - using direct Gazebo differential drive plugin
    # The gazebo_control.xacro includes built-in differential drive and joint state plugins

    # ROS-Gazebo Bridge Configuration
    # Path to configuration file defining topic mappings between ROS 2 and Gazebo
    bridge_params = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'gz_bridge.yaml'
    )

    # ROS-Gazebo Parameter Bridge
    # Bridges general topics between ROS 2 and Gazebo (odometry, sensors, etc.)
    # Functionality: Translates messages between ROS 2 and Gazebo message formats
    # Dependencies: Requires gz_bridge.yaml configuration file
    ros_gz_bridge = Node(
        package="ros_gz_bridge",         # Gazebo-ROS bridge package
        executable="parameter_bridge",   # Configurable bridge executable
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',  # Configuration file path
        ]
    )

    # ROS-Gazebo Image Bridge
    # Specialized bridge for camera image data between ROS 2 and Gazebo
    # Functionality: Converts Gazebo camera images to ROS 2 sensor_msgs/Image
    # Dependencies: Requires camera sensor in robot URDF and Gazebo simulation
    ros_gz_image_bridge = Node(
        package="ros_gz_image",          # Gazebo image bridge package
        executable="image_bridge",       # Image conversion executable
        arguments=["/camera/image_raw"]  # Camera topic to bridge from Gazebo
    )

    # Launch Description Assembly
    # All simulation components launched with proper dependencies
    return LaunchDescription([
        rsp,                    # Robot state publisher (simulation mode)
        joystick,               # Manual control interface
        twist_mux,              # Command arbitration system
        world_arg,              # World file selection argument
        gazebo,                 # Physics simulation environment
        spawn_entity,           # Robot model instantiation
        ros_gz_bridge,          # General ROS-Gazebo communication
        ros_gz_image_bridge     # Camera data bridging
    ])
