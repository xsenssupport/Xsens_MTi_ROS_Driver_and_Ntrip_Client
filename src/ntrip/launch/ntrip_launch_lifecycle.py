from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare the log level argument
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal)',
        choices=['debug', 'info', 'warn', 'error', 'fatal']
    )

    config_file_arg = DeclareLaunchArgument(
	'config_file',
	default_value='ntrip-param.yaml',
	description='Name of the parameter file'
    )
    
    parameters_file_path = PathJoinSubstitution([
	get_package_share_directory('ntrip'),
	'config',
	LaunchConfiguration('config_file')
    ])

    # Create the node configuration
    ntrip_lifecycle_node = Node(
        package='ntrip',
        executable='ntrip_lifecycle',
        name='ntrip_client_lifecycle',
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
        log_level_arg,  # Include the log level argument
        config_file_arg,
	    ntrip_lifecycle_node  # Include the node configuration
    ])
