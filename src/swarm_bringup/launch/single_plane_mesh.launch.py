"""Launch one aircraft with initialized mesh navigation."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(
        get_package_share_directory('swarm_bringup')
    )
    reference = str(
        share / 'config/config_reference.yaml'
    )

    agent = ExecuteProcess(
        cmd=[
            LaunchConfiguration('agent_executable'),
            'udp4',
            '-p',
            '8888',
        ],
        condition=IfCondition(
            LaunchConfiguration('start_agent')
        ),
        output='screen',
    )

    initializer = Node(
        package='swarm_simulation',
        executable='mesh_startup',
        namespace='plane_1',
        parameters=[
            reference,
            {
                'vehicle_id': 1,
                'px4_namespace': '',
                'use_sim_time': False,
                'startup_timeout_s': 90.0,
            },
        ],
        output='screen',
    )

    control = Node(
        package='swarm_px4_bridge',
        executable='swarm_control_node',
        namespace='/',
        parameters=[
            {'use_sim_time': False},
        ],
        output='screen',
    )

    def initialized(event, context):
        if context.is_shutdown:
            return []

        if event.returncode != 0:
            return [
                LogInfo(
                    msg='ERROR: Mesh initialization failed'
                ),
                EmitEvent(
                    event=Shutdown(
                        reason='navigation init failed'
                    )
                ),
            ]

        return [control]

    return LaunchDescription([
        DeclareLaunchArgument(
            'px4_dir',
            default_value=str(
                Path.home()
                / 'swarm_ws/third_party/PX4-Autopilot'
            ),
        ),

        DeclareLaunchArgument(
            'start_agent',
            default_value='true',
        ),

        DeclareLaunchArgument(
            'agent_executable',
            default_value=str(
                Path.home()
                / '.local/opt/'
                'micro-xrce-dds-agent-2.4.3/'
                'bin/MicroXRCEAgent'
            ),
        ),

        SetEnvironmentVariable(
            'GZ_IP', '127.0.0.1'
        ),
        SetEnvironmentVariable(
            'PX4_UXRCE_DDS_NS', ''
        ),
        SetEnvironmentVariable(
            'PX4_UXRCE_DDS_PORT', '8888'
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=initializer,
                on_exit=initialized,
            )
        ),

        agent,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(
                    share
                    / 'launch/single_plane.launch.py'
                )
            ),
            launch_arguments={
                'px4_dir': LaunchConfiguration('px4_dir'),
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(
                    share
                    / 'launch/mesh_sensors.launch.py'
                )
            )
        ),

        Node(
            package='swarm_px4_bridge',
            executable='telemetry_node',
            namespace='plane_1',
            parameters=[
                reference,
                {
                    'px4_namespace': '',
                    'use_sim_time': True,
                },
            ],
            output='screen',
        ),

        initializer,
    ])