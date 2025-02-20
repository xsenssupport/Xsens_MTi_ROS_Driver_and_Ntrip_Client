from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ntrip',
            executable='ntrip',
            name='ntrip_client',
            output='screen',
            parameters=[{
                # NTRIP Server Configuration
                # Replace these values with your NTRIP caster information
                'host': '203.107.45.154',  # Change to the IP address of Your NTRIP caster
                'port': 8002,          # Change to your port number, WGS84
                'mountpoint': 'AUTO',  # Your NTRIP mountpoint
                'username': 'Your_User_Name',     # Your NTRIP username
                'password': 'Your_Password',     # Your NTRIP password

                # NMEA and Update Rate Configuration
                # Adjust these based on your GNSS receiver's capabilities
                'nmea_input_rate': 4.0,    # Input NMEA rate in Hz (default: 4.0)
                'update_rate': 1.0,        # Desired rate for sending GGA messages (Hz)

                # Connection Configuration
                'reconnect_delay': 5.0,    # Delay between reconnection attempts (seconds)
                'max_reconnect_attempts': 0,  # 0 for infinite attempts

                # Debug Configuration
                'send_default_gga': True,    # Set to False if using real GNSS data
                'debug': False,              # Set to True for detailed debug output
                'output_rtcm_details': False # Set to True for RTCM message details
            }],
            # Topic Remapping
            # Adjust these if your NMEA input topic or RTCM output topic are different
            remappings=[
                ('nmea', 'nmea'),  # Input NMEA topic
                ('rtcm', 'rtcm')   # Output RTCM topic
            ]
        )
    ])