import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    count = int(
        LaunchConfiguration('count').perform(context)
    )
    formation = LaunchConfiguration('formation').perform(context)
    spacing = float(
        LaunchConfiguration('spacing').perform(context)
    )
    external = int(
        LaunchConfiguration('external_nav_state').perform(context)
    )

    if not 2 <= count <= 5:
        raise ValueError('count must be 2..5')

    if not math.isfinite(spacing) or not 45.0 <= spacing <= 120.0:
        raise ValueError('spacing must be 45..120 m')

    if not 23 <= external <= 30:
        raise ValueError('external_nav_state must be 23..30')

    reference = str(
        Path(get_package_share_directory('swarm_bringup'))
        / 'config'
        / 'config_reference.yaml'
    )

    nodes = []

    for index in range(1, count):
        vehicle_id = index + 1

        if formation == 'line':
            behind = index * spacing
            right = 0.0

        elif formation == 'wedge':
            row = (index + 1) // 2

            behind = row * spacing

            side = -1.0 if index % 2 else 1.0
            right = side * row * spacing * 0.8

        else:
            raise ValueError('formation must be line or wedge')

        nodes.append(
            Node(
                package='swarm_control',
                executable='follower',
                name=f'follower_{vehicle_id}',
                parameters=[
                    reference,
                    {
                        'use_sim_time': True,
                        'leader_id': 1,
                        'follower_id': vehicle_id,
                        'external_nav_state': external,
                        'behind_m': float(behind),
                        'right_m': float(right),
                        'above_m': float(35.0 * index),
                        'slot_index': index,
                    },
                ],
                output='screen',
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'count',
            default_value='3',
        ),
        DeclareLaunchArgument(
            'formation',
            default_value='line',
            choices=['line', 'wedge'],
        ),
        DeclareLaunchArgument(
            'spacing',
            default_value='60.0',
        ),
        DeclareLaunchArgument(
            'external_nav_state',
            default_value='23',
        ),
        OpaqueFunction(function=setup),
    ])