#include <memory>
#include <thread>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("move_to_pose");
  auto logger = node->get_logger();

  // 目標位置（單位：公尺，座標系為規劃框架，Panda 預設是 panda_link0）
  const double target_x = node->declare_parameter<double>("target_x", 0.28);
  const double target_y = node->declare_parameter<double>("target_y", -0.2);
  const double target_z = node->declare_parameter<double>("target_z", 0.5);
  // true：在場景放入一個箱子；false：移除該箱子
  const bool add_obstacle = node->declare_parameter<bool>("add_obstacle", false);

  // MoveGroupInterface 需要節點持續被 spin 才能取得機器人狀態，
  // 所以另外開一條執行緒跑 executor。
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  int exit_code = 0;
  {
    moveit::planning_interface::MoveGroupInterface move_group(node, "panda_arm");

    // 碰撞物件：以固定 id 新增或移除，重複執行不會累積
    moveit::planning_interface::PlanningSceneInterface scene;
    const std::string obstacle_id = "demo_box";
    if (add_obstacle) {
      moveit_msgs::msg::CollisionObject box;
      box.header.frame_id = move_group.getPlanningFrame();
      box.id = obstacle_id;

      shape_msgs::msg::SolidPrimitive shape;
      shape.type = shape.BOX;
      shape.dimensions = {0.1, 0.1, 0.3};  // x, y, z（公尺）

      geometry_msgs::msg::Pose box_pose;
      box_pose.orientation.w = 1.0;
      box_pose.position.x = 0.3;
      box_pose.position.y = 0.0;
      box_pose.position.z = 0.2;

      box.primitives.push_back(shape);
      box.primitive_poses.push_back(box_pose);
      box.operation = box.ADD;
      scene.applyCollisionObject(box);
      RCLCPP_INFO(logger, "Obstacle '%s' added", obstacle_id.c_str());
    } else {
      scene.removeCollisionObjects({obstacle_id});
    }

    geometry_msgs::msg::Pose target;
    target.orientation.x = 1.0;
    target.orientation.y = 0.0;
    target.orientation.z = 0.0;
    target.orientation.w = 0.0;
    target.position.x = target_x;
    target.position.y = target_y;
    target.position.z = target_z;
    RCLCPP_INFO(logger, "Target: (%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f)", 
      target_x, target_y, target_z, target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w);
    move_group.setPoseTarget(target);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool planned =
      (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!planned) {
      RCLCPP_ERROR(logger, "Planning failed");
      exit_code = 1;
    } else {
      RCLCPP_INFO(logger, "Planning succeeded, executing");
      const auto result = move_group.execute(plan);
      if (result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(logger, "Execution failed (code %d)", result.val);
        exit_code = 2;
      }
    }
  }

  rclcpp::shutdown();
  spinner.join();
  return exit_code;
}