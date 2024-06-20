import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    ld = LaunchDescription()

    # Set env var to print messages to stdout immediately
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '0'))

    
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([
                FindPackageShare('xsens_mti_ros2_driver'), 'launch', 'xsens_mti_node.launch.py'
                ]),
            )
        )
    ld.add_action(driver_launch)

    # Delay rviz2_node start-up by 1 seconds to avoid warnings on /tf
    rviz_config_path = os.path.join(get_package_share_directory('xsens_mti_ros2_driver'), 'rviz', 'display.rviz')
    rviz_delay_command = f"sleep 1 && rviz2 -d {rviz_config_path}"
    rviz2_delay = ExecuteProcess(
        cmd=['bash', '-c', rviz_delay_command],
        name='xsens_rviz2',
        output='screen',
    )
    ld.add_action(rviz2_delay)


    # Robot State Publisher node
    urdf_file_path = os.path.join(get_package_share_directory('xsens_mti_ros2_driver'), 'urdf', 'MTi_6xx.urdf')
    state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='xsens_state_publisher',
        output='screen',
        arguments=[urdf_file_path],
    )
    ld.add_action(state_publisher_node)

    return ld
