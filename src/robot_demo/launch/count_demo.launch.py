from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    count_delay = LaunchConfiguration("count_delay")

    config_file = os.path.join(
        get_package_share_directory("robot_demo"), "config", "count_params.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "count_delay",
                default_value="0.2",
                description="Delay between count steps in seconds",
            ),
            # Robot 1
            Node(
                package="robot_demo",
                executable="count_server",
                name="count_server",
                namespace="robot1",
                output="screen",
                parameters=[config_file],
            ),
            Node(
                package="robot_demo",
                executable="count_client",
                name="count_client",
                namespace="robot1",
                output="screen",
            ),
            # Robot 2
            Node(
                package="robot_demo",
                executable="count_server",
                name="count_server",
                namespace="robot2",
                output="screen",
                parameters=[config_file],
            ),
            Node(
                package="robot_demo",
                executable="count_client",
                name="count_client",
                namespace="robot2",
                output="screen",
            ),
        ]
    )
