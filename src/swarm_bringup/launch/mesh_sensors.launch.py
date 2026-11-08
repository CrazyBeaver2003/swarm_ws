"""Connect an already running Gazebo world to the virtual ranging sensors."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    bringup = Path(get_package_share_directory('swarm_bringup'))
    localization = Path(get_package_share_directory('mesh_localization'))
    reference = str(bringup / 'config/config_reference.yaml')
    stations = str(bringup / 'config/stations.yaml')
    profile = str(localization / 'config/ranging.yaml')
    return LaunchDescription([
        SetEnvironmentVariable('GZ_IP', '127.0.0.1'),
        Node(package='swarm_simulation', executable='gazebo_bridge',
             parameters=[reference, profile, {'world': 'default', 'publish_rate_hz': 30.0}],
             output='screen'),
        Node(package='mesh_localization', executable='rtt_sensor',
             parameters=[reference, stations, profile], output='screen'),
    ])
