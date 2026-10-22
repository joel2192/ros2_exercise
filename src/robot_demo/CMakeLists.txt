cmake_minimum_required(VERSION 3.8)
project(robot_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(example_interfaces REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CountUntil.action"
)

add_executable(demo_node src/demo_node.cpp)
ament_target_dependencies(demo_node
  rclcpp
  std_msgs
)

add_executable(listener_node src/listener_node.cpp)

ament_target_dependencies(listener_node
  rclcpp
  std_msgs
)

add_executable(add_server src/add_server.cpp)

ament_target_dependencies(add_server
  rclcpp
  example_interfaces
)
add_executable(add_client src/add_client.cpp)

ament_target_dependencies(add_client
  rclcpp
  example_interfaces
)

add_executable(count_server src/count_server.cpp)

ament_target_dependencies(
  count_server
  rclcpp
  rclcpp_action
)

add_executable(count_client src/count_client.cpp)

add_executable(lifecycle_demo src/lifecycle_demo.cpp)
ament_target_dependencies(lifecycle_demo
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

ament_target_dependencies(
  count_client
  rclcpp
  rclcpp_action
)


rosidl_get_typesupport_target(
  cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_link_libraries(
  count_server
  "${cpp_typesupport_target}"
)

target_link_libraries(
  count_client
  "${cpp_typesupport_target}"
)


add_library(hello_component SHARED
  src/hello_component.cpp
)

ament_target_dependencies(hello_component
  rclcpp
  rclcpp_components
  std_msgs
)

rclcpp_components_register_nodes(hello_component
  "robot_demo::HelloComponent"
)

add_executable(lidar_sim src/lidar_sim.cpp)
ament_target_dependencies(lidar_sim
  rclcpp
  sensor_msgs
)

add_executable(lidar_sub src/lidar_sub.cpp)
ament_target_dependencies(lidar_sub
  rclcpp
  sensor_msgs
)

add_executable(obstacle_detector
  src/obstacle_detector.cpp
)

ament_target_dependencies(obstacle_detector
  rclcpp
  sensor_msgs
  std_msgs
)

target_include_directories(obstacle_detector PRIVATE
  include
)



install(TARGETS hello_component
  LIBRARY DESTINATION lib
)

install(TARGETS
  demo_node
  listener_node
  add_server
  add_client
  count_server
  count_client
  lifecycle_demo
  lidar_sim
  lidar_sub
  obstacle_detector
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_obstacle_utils
    test/test_obstacle_utils.cpp
  )

  ament_target_dependencies(test_obstacle_utils
    sensor_msgs
  )

  target_include_directories(test_obstacle_utils PRIVATE
    include
  )
endif()

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
