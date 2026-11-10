"""Gazebo/PX4 aircraft with separate navigation and control namespaces."""

import os
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def stop_on_exit(process, label):
    def exited(event, context):
        if context.is_shutdown:
            return []

        return [
            EmitEvent(
                event=Shutdown(
                    reason=f'{label} exited with code {event.returncode}'
                )
            )
        ]

    return RegisterEventHandler(
        OnProcessExit(target_action=process, on_exit=exited)
    )


def start_control_when_ready(initializer, control, vehicle_id):
    def initialized(event, context):
        if context.is_shutdown:
            return []

        if event.returncode != 0:
            return [
                EmitEvent(
                    event=Shutdown(
                        reason=(
                            f'plane_{vehicle_id}: mesh initialization failed'
                        )
                    )
                )
            ]

        return [control]

    return RegisterEventHandler(
        OnProcessExit(
            target_action=initializer,
            on_exit=initialized,
        )
    )


def setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    count = int(arg('count'))
    domain = int(arg('domain_id'))
    port = int(arg('agent_port'))

    if not 1 <= count <= 5:
        raise ValueError('count must be between 1 and 5')

    if not 0 <= domain <= 101 or not 1024 <= port <= 65535:
        raise ValueError(
            'Use domain_id 0..101 and agent_port 1024..65535'
        )

    px4 = Path(arg('px4_dir')).expanduser().resolve()
    build = px4 / 'build/px4_sitl_default'

    models = px4 / 'Tools/simulation/gz/models'
    worlds = px4 / 'Tools/simulation/gz/worlds'
    plugins = build / 'src/modules/simulation/gz_plugins'
    server_config = (
        px4 / 'src/modules/simulation/gz_bridge/server.config'
    )
    agent_path = Path(arg('agent_executable')).expanduser().resolve()

    required_paths = [
        build / 'bin/px4',
        build / 'etc',
        models / 'rc_cessna/model.sdf',
        worlds / 'default.sdf',
        plugins,
        server_config,
        agent_path,
    ]

    for path in required_paths:
        if not path.exists():
            raise RuntimeError(
                f'Required path does not exist: {path}'
            )

    bringup = Path(
        get_package_share_directory('swarm_bringup')
    )
    localization = Path(
        get_package_share_directory('mesh_localization')
    )

    reference = str(bringup / 'config/config_reference.yaml')
    stations = str(bringup / 'config/stations.yaml')
    ranging = str(localization / 'config/ranging.yaml')
    localizer_config = str(
        localization / 'config/localization.yaml'
    )

    run_root = Path(arg('run_root')).expanduser()
    run_root.mkdir(parents=True, exist_ok=True)

    run_dir = Path(
        tempfile.mkdtemp(prefix='swarm_', dir=run_root)
    )

    ids = list(range(1, count + 1))

    def prepend(variable, *paths):
        values = [str(path) for path in paths]
        previous = context.environment.get(variable, '')

        if previous:
            values.append(previous)

        return os.pathsep.join(values)

    environment = {
        'ROS_DOMAIN_ID': str(domain),
        'GZ_IP': '127.0.0.1',
        'GZ_PARTITION': run_dir.name,
        'GZ_SIM_RESOURCE_PATH': prepend(
            'GZ_SIM_RESOURCE_PATH', models, worlds
        ),
        'GZ_SIM_SYSTEM_PLUGIN_PATH': prepend(
            'GZ_SIM_SYSTEM_PLUGIN_PATH', plugins
        ),
        'GZ_SIM_SERVER_CONFIG_PATH': str(server_config),
        'OPENBLAS_NUM_THREADS': '1',
        'OMP_NUM_THREADS': '1',
    }

    actions = [
        SetEnvironmentVariable(key, value)
        for key, value in environment.items()
    ]

    actions.append(
        LogInfo(
            msg=(
                f'Run directory: {run_dir}; '
                f'ROS_DOMAIN_ID={domain}; '
                f'GZ_PARTITION={run_dir.name}'
            )
        )
    )

    def add_process(process, label):
        actions.extend([
            stop_on_exit(process, label),
            process,
        ])

    add_process(
        ExecuteProcess(
            cmd=[
                str(agent_path),
                'udp4',
                '-p',
                str(port),
            ],
            output='both',
        ),
        'DDS Agent',
    )

    add_process(
        ExecuteProcess(
            cmd=[
                'gz', 'sim',
                '-r', '-s',
                '--render-engine', 'ogre',
                str(worlds / 'default.sdf'),
            ],
            output='both',
        ),
        'Gazebo server',
    )

    actions.append(
        ExecuteProcess(
            cmd=[
                'gz', 'sim',
                '-g',
                '--render-engine', 'ogre',
            ],
            condition=IfCondition(
                LaunchConfiguration('gui')
            ),
            output='both',
        )
    )

    add_process(
        Node(
            package='swarm_simulation',
            executable='gazebo_bridge',
            parameters=[
                reference,
                {
                    'world': 'default',
                    'use_sim_time': True,
                    'publish_rate_hz': 30.0,
                    'vehicle_ids': ids,
                    'model_names': [
                        f'rc_cessna_{i}' for i in range(count)
                    ],
                },
            ],
            output='screen',
        ),
        'Gazebo bridge',
    )

    add_process(
        Node(
            package='mesh_localization',
            executable='rtt_sensor',
            parameters=[
                reference,
                stations,
                ranging,
                {'vehicle_ids': ids},
            ],
            output='screen',
        ),
        'RTT sensor',
    )

    for instance, vehicle_id in enumerate(ids):
        namespace = f'plane_{vehicle_id}'

        work = run_dir / f'px4_{instance}'
        work.mkdir()

        px4_env = {
            'PX4_SYS_AUTOSTART': '4003',
            'PX4_SIM_MODEL': 'gz_rc_cessna',
            'PX4_GZ_STANDALONE': '1',
            'PX4_GZ_WORLD': 'default',
            'PX4_GZ_MODEL_NAME': '',
            'PX4_GZ_MODEL_POSE': (
                f'0,{-60 - 40 * instance},0,0,0,0'
            ),
            'PX4_GZ_MODELS': str(models),
            'PX4_GZ_WORLDS': str(worlds),
            'PX4_UXRCE_DDS_NS': namespace,
            'PX4_UXRCE_DDS_PORT': str(port),
        }

        parameters = {
            'EKF2_GPS_CTRL': 0,
            'EKF2_EV_CTRL': 7,
            'EKF2_HGT_REF': 3,
            'EKF2_EV_DELAY': 0,
            'COM_RC_IN_MODE': 4,
            'FW_AIRSPD_MIN': 10,
            'FW_AIRSPD_MAX': 20,
            'FW_AIRSPD_TRIM': 15,
        }

        px4_env.update({
            f'PX4_PARAM_{key}': str(value)
            for key, value in parameters.items()
        })

        add_process(
            ExecuteProcess(
                cmd=[
                    str(build / 'bin/px4'),
                    '-i', str(instance),
                    '-d',
                    '-w', str(work),
                    str(build / 'etc'),
                ],
                cwd=str(work),
                additional_env=px4_env,
                output='both',
            ),
            f'PX4 {vehicle_id}',
        )

        add_process(
            Node(
                package='mesh_localization',
                executable='mesh_localizer',
                namespace=namespace,
                parameters=[
                    reference,
                    stations,
                    localizer_config,
                    {
                        'vehicle_id': vehicle_id,
                        'use_sim_time': True,
                    },
                ],
                output='screen',
            ),
            f'{namespace} localizer',
        )

        add_process(
            Node(
                package='swarm_simulation',
                executable='mesh_px4_bridge',
                namespace=namespace,
                parameters=[
                    reference,
                    {
                        'vehicle_id': vehicle_id,
                        'px4_namespace': namespace,
                        'use_sim_time': True,
                    },
                ],
                output='screen',
            ),
            f'{namespace} mesh bridge',
        )

        add_process(
            Node(
                package='swarm_px4_bridge',
                executable='telemetry_node',
                namespace=namespace,
                parameters=[
                    reference,
                    {
                        'px4_namespace': namespace,
                        'use_sim_time': True,
                    },
                ],
                output='screen',
            ),
            f'{namespace} telemetry',
        )

        initializer = Node(
            package='swarm_simulation',
            executable='mesh_startup',
            namespace=namespace,
            parameters=[
                reference,
                {
                    'vehicle_id': vehicle_id,
                    'px4_namespace': namespace,
                    'use_sim_time': False,
                    'startup_timeout_s': 180.0,
                },
            ],
            output='screen',
        )

        control = Node(
            package='swarm_px4_bridge',
            executable='swarm_control_node',
            namespace=namespace,
            parameters=[
                {
                    'use_sim_time': False,
                    'flight_namespace': f'/{namespace}/flight',
                    'takeoff_height_m': 70.0 + 35.0 * instance,
                    'takeoff_timeout_s': 300.0,
                },
            ],
            output='screen',
        )

        actions.extend([
            stop_on_exit(control, f'{namespace} control'),
            start_control_when_ready(
                initializer, control, vehicle_id
            ),
            initializer,
        ])

    return actions


def generate_launch_description():
    home = Path.home()

    return LaunchDescription([
        DeclareLaunchArgument(
            'px4_dir',
            default_value=str(
                home / 'swarm_ws/third_party/PX4-Autopilot'
            ),
        ),
        DeclareLaunchArgument(
            'agent_executable',
            default_value=str(
                home / '.local/opt/'
                'micro-xrce-dds-agent-2.4.3/bin/MicroXRCEAgent'
            ),
        ),
        DeclareLaunchArgument(
            'count', default_value='2'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'domain_id', default_value='0'
        ),
        DeclareLaunchArgument(
            'agent_port', default_value='8890'
        ),
        DeclareLaunchArgument(
            'run_root',
            default_value=str(home / 'swarm_ws/runs'),
        ),
        OpaqueFunction(function=setup),
    ])