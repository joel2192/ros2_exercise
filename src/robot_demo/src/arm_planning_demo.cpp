
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(
      "arm_planning_demo",
      rclcpp::NodeOptions()
          .automatically_declare_parameters_from_overrides(true));

  std::string robot_description;

  if (!node->get_parameter("robot_description", robot_description))
  {
    // 目前節點沒有此參數，稍後會從 MoveIt 取得模型
  }
  // 使用 Panda 的手臂規劃群組
  moveit::planning_interface::MoveGroupInterface move_group(
      node, "panda_arm");
  // 設定規劃時間上限
  move_group.setPlanningTime(5.0);

  // 先使用目前姿態作為目標，驗證規劃管線
  move_group.setStartStateToCurrentState();
  move_group.setJointValueTarget(move_group.getCurrentJointValues());

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto result = move_group.plan(plan);

  if (result == moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_INFO(
        node->get_logger(),
        "Planning succeeded. Trajectory points: %zu",
        plan.trajectory.joint_trajectory.points.size());
  }
  else
  {
    RCLCPP_ERROR(
        node->get_logger(),
        "Planning failed. Error code: %d",
        result.val);
  }

  rclcpp::shutdown();
  return result == moveit::core::MoveItErrorCode::SUCCESS ? 0 : 1;
}
