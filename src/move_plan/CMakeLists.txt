cmake_minimum_required(VERSION 3.8)
project(move_plan)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
add_executable(goal_recieve_node src/goal_recieve_node.cpp)
add_executable(speed_couculate_node src/speed_couculate_node.cpp)
add_executable(speed_calculate_omni_node src/speed_calculate_omni_node.cpp)
add_executable(saved_goal_pub_node src/saved_goal_pub_node.cpp)
add_executable(r2_pass_speed_calculate_node src/r2_pass_speed_calculate_node.cpp)
ament_target_dependencies(
  goal_recieve_node
  "rclcpp"
  "geometry_msgs"
)
ament_target_dependencies(
  speed_couculate_node
  "rclcpp"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
  "nav_msgs"
)
ament_target_dependencies(
  speed_calculate_omni_node
  "rclcpp"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
  "nav_msgs"
)
ament_target_dependencies(
  saved_goal_pub_node
  "rclcpp"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
  "nav_msgs"
)
ament_target_dependencies(
  r2_pass_speed_calculate_node
  "rclcpp"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
  "nav_msgs"
  "std_msgs"
)
#目标点接收测试节点
install(TARGETS goal_recieve_node
  DESTINATION lib/${PROJECT_NAME})
#pid导航速度运算节点
install(TARGETS speed_couculate_node
DESTINATION lib/${PROJECT_NAME})
#全向轮速度运算节点
install(TARGETS speed_calculate_omni_node
DESTINATION lib/${PROJECT_NAME})
#记录着保存的目标点，已经集成
install(TARGETS saved_goal_pub_node
DESTINATION lib/${PROJECT_NAME})
#传球机器人对齐角速度计算
install(TARGETS r2_pass_speed_calculate_node
DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
maps
param
launch
rviz
)
