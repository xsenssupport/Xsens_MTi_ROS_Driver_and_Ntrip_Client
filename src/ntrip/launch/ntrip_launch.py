import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='ntrip',
            executable='ntrip',
            name='ntrip_client',
            output='screen',
            parameters=[
                {'ip': '120.253.239.161'},  # Change to the IP address of Your NTRIP service
                {'port': 8002},  # Change to your port number, WGS84
                {'user': 'Your_User_Name'},  # Change to your username
                {'passwd': 'Your_Password'},  # Change to your password
                {'mountpoint': 'RTCM33_GRCE'},  # Change to your mountpoint
                {'report_interval': 1} # the report interval to the NTRIP Caster, default is 1 sec
            ]
        ),
    ])
    
