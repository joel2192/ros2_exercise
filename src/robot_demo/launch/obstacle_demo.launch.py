from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    lidar_sim = Node(
        package="robot_demo",
        executable="lidar_sim",
        name="lidar_sim",
        output="screen",
    )

    obstacle_detector = Node(
        package="robot_demo",
        executable="obstacle_detector",
        name="obstacle_detector",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("robot_demo"),
                    "config",
                    "obstacle_params.yaml",
                ]
            )
        ],
    )

    return LaunchDescription(
        [
            lidar_sim,
            obstacle_detector,
        ]
    )
