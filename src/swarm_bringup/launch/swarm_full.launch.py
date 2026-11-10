"""Complete simulation, control, operator map and measurement pipeline."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    count, domain = int(value('count')), int(value('domain_id'))
    if not 3 <= count <= 5 or not 0 <= domain <= 101:
        raise ValueError('Use count=3..5 and domain_id=0..101')
    ids = list(range(1, count + 1))
    share = Path(get_package_share_directory('swarm_bringup'))
    reference = str(share / 'config/config_reference.yaml')
    common = [reference, {'use_sim_time': True, 'vehicle_ids': ids}]
    return [
        SetEnvironmentVariable('ROS_DOMAIN_ID', str(domain)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/swarm_sim.launch.py')),
            launch_arguments={'count': str(count), 'domain_id': str(domain),
                              'gui': value('gazebo_gui'), 'run_root': value('run_root')}.items()),
        Node(package='swarm_simulation', executable='gcs_link', name='gcs_link',
             parameters=common, output='screen'),
        Node(package='swarm_simulation', executable='mesh_network', name='mesh_network',
             parameters=common + [str(share / 'config/network.yaml')], output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/formation_control.launch.py')),
            launch_arguments={'count': str(count), 'formation': value('formation'),
                              'spacing': value('spacing'), 'leader_peer_prefix': '/mesh',
                              'external_nav_state': value('external_nav_state')}.items()),
        Node(package='swarm_evaluation', executable='evaluator', name='swarm_evaluation',
             parameters=common + [{'output_directory': value('evaluation_root'),
                                   'external_nav_state': int(value('external_nav_state'))}], output='screen'),
        Node(package='swarm_simulation', executable='operator_panel', name='operator_panel',
             parameters=common + [{'stations_file': str(share / 'config/stations.yaml'),
                                   'host': '127.0.0.1', 'port': int(value('http_port'))}],
             output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('count', default_value='3'),
        DeclareLaunchArgument('domain_id', default_value='0'),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('formation', default_value='wedge', choices=['line', 'wedge']),
        DeclareLaunchArgument('spacing', default_value='60.0'),
        DeclareLaunchArgument('external_nav_state', default_value='23'),
        DeclareLaunchArgument('http_port', default_value='8080'),
        DeclareLaunchArgument('run_root', default_value=str(Path.home() / 'swarm_ws/runs')),
        DeclareLaunchArgument('evaluation_root', default_value=str(Path.home() / 'swarm_ws/results')),
        OpaqueFunction(function=setup),
    ])
