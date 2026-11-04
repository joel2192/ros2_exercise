from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # 與官方 demo 的 run.launch.py 使用相同的 Panda 設定
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .planning_pipelines(pipelines=["ompl"])
        .robot_description(file_path="config/panda.urdf.xacro")
        .to_moveit_configs()
    )

    args = [
        DeclareLaunchArgument("object_x", default_value="0.5", description="物體 X（world，公尺）"),
        DeclareLaunchArgument("object_y", default_value="-0.25", description="物體 Y（world，公尺）"),
        DeclareLaunchArgument("place_y", default_value="0.5", description="放置位置在 object 座標系下的 Y 偏移"),
        DeclareLaunchArgument(
            "exit_after_task", default_value="false", description="任務結束後自動結束（批次測試用）"
        ),
    ]

    mtc_node = Node(
        package="mtc_tutorial",
        executable="mtc_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "object_x": ParameterValue(LaunchConfiguration("object_x"), value_type=float),
                "object_y": ParameterValue(LaunchConfiguration("object_y"), value_type=float),
                "place_y": ParameterValue(LaunchConfiguration("place_y"), value_type=float),
                "exit_after_task": ParameterValue(LaunchConfiguration("exit_after_task"), value_type=bool),
            },
        ],
    )

    return LaunchDescription(args + [mtc_node])