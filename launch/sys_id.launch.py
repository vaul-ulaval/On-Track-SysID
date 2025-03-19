import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("on_track_sys_id"),
        "config",
        "on_track_sys_id_params.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("odom_topic", default_value="/ego_racecar/odom"),
            DeclareLaunchArgument("ackermann_cmd_topic", default_value="/drive"),
            DeclareLaunchArgument("save_LUT_name", default_value="sim"),
            DeclareLaunchArgument("plot_model", default_value="True"),
            DeclareLaunchArgument("racecar_version", default_value="SIM"),
            Node(
                package="on_track_sys_id",
                executable="on_track_sys_id",
                name="on_track_sys_id",
                output="screen",
                parameters=[config_file],
                remappings=[
                    ("/odom", LaunchConfiguration("odom_topic")),
                    ("/ackermann_cmd", LaunchConfiguration("ackermann_cmd_topic")),
                ],
            ),
        ]
    )
