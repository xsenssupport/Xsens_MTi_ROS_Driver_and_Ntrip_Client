from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    # Declare the log level argument
    log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal)',
        choices=['debug', 'info', 'warn', 'error', 'fatal']
    )
    
    parameters_file_path = Path(get_package_share_directory('ntrip'), 'config', 'ntrip-param.yaml')

    # Create the node configuration
    ntrip_node = Node(
        package='ntrip',
        executable='ntrip',
        name='ntrip_client',
        output='screen',
        parameters=[parameters_file_path],
        # Topic Remapping
        remappings=[
            ('nmea', 'nmea'),  # Input NMEA topic
            ('rtcm', 'rtcm')   # Output RTCM topic
        ],
        # Add arguments for log level
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )

    return LaunchDescription([
        log_level,  # Include the log level argument
        ntrip_node  # Include the node configuration
    ])