"""Connect Gazebo to virtual ranging sensors and localization."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    bringup = Path(
        get_package_share_directory('swarm_bringup')
    )
    localization = Path(
        get_package_share_directory('mesh_localization')
    )

    reference = str(
        bringup / 'config/config_reference.yaml'
    )
    stations = str(
        bringup / 'config/stations.yaml'
    )
    ranging_config = str(
        localization / 'config/ranging.yaml'
    )
    localization_config = str(
        localization / 'config/localization.yaml'
    )

    return LaunchDescription([
        SetEnvironmentVariable('GZ_IP', '127.0.0.1'),

        Node(
            package='swarm_simulation',
            executable='gazebo_bridge',
            parameters=[
                reference,
                ranging_config,
                {
                    'world': 'default',
                    'publish_rate_hz': 30.0,
                },
            ],
            output='screen',
        ),

        Node(
            package='mesh_localization',
            executable='rtt_sensor',
            parameters=[
                reference,
                stations,
                ranging_config,
            ],
            output='screen',
        ),

        Node(
            package='mesh_localization',
            executable='mesh_localizer',
            namespace='plane_1',
            parameters=[
                reference,
                stations,
                localization_config,
                {'vehicle_id': 1},
            ],
            output='screen',
        ),
    ])