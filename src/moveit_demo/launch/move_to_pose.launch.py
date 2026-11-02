from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # 從 moveit_resources_panda_moveit_config 載入 Panda 的 MoveIt 設定
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .robot_description_semantic(file_path="config/panda.srdf")
        .robot_description_kinematics()
        .to_moveit_configs()
    )

    launch_args = [
        DeclareLaunchArgument("target_x", default_value="0.28"),
        DeclareLaunchArgument("target_y", default_value="-0.2"),
        DeclareLaunchArgument("target_z", default_value="0.5"),
        DeclareLaunchArgument("add_obstacle", default_value="false"),
    ]

    move_to_pose = Node(
        package="moveit_demo",
        executable="move_to_pose",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {
                "target_x": LaunchConfiguration("target_x"),
                "target_y": LaunchConfiguration("target_y"),
                "target_z": LaunchConfiguration("target_z"),
                "add_obstacle": LaunchConfiguration("add_obstacle"),
            },
        ],
    )

    return LaunchDescription(launch_args + [move_to_pose])