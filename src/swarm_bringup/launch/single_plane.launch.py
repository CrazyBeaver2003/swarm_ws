from pathlib import Path
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    px4_dir = LaunchConfiguration("px4_dir")

    build_dir = PathJoinSubstitution(
        [px4_dir, "build", "px4_sitl_default"]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "px4_dir",
            default_value = os.path.join(Path.home(), "swarm_ws", "third_party", "PX4-Autopilot"),
            description = "Path to the PX4 source directory",
        ),

        ExecuteProcess(
            cmd = [
                PathJoinSubstitution([build_dir, "bin", "px4"]),
                "-d",
            ],
            cwd = PathJoinSubstitution([build_dir, "rootfs"]),
            additional_env={
                "PX4_SYS_AUTOSTART": "4003",
                "PX4_SIM_MODEL": "gz_rc_cessna",
                "PX4_GZ_WORLD": "default",
                "GZ_IP": "127.0.0.1",
                "PX4_GZ_MODEL_POSE": "0,-60,0,0,0,0",
            },
            output="screen",
            emulate_tty=True,
        ),
    ])