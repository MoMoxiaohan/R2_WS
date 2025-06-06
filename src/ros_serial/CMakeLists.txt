cmake_minimum_required(VERSION 3.8)
project(ros_serial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial_driver REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_executable(demo01_test src/demo01_test.cpp src/serialport.cpp)  #需要修改这里
target_include_directories(demo01_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ros_serial>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
  ament_target_dependencies(
    demo01_test
    "rclcpp"
    "serial_driver"
    "geometry_msgs"
    "tf2"
    "tf2_ros"
    "nav_msgs"
  )
install(TARGETS demo01_test
DESTINATION lib/${PROJECT_NAME})
add_executable(demo02_recieve src/demo02_recieve.cpp src/serialport.cpp)  #需要修改这里
target_include_directories(demo02_recieve PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ros_serial>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
  ament_target_dependencies(
    demo02_recieve
    "rclcpp"
    "serial_driver"
    "geometry_msgs"
    "tf2"
    "tf2_ros"
    "nav_msgs"
  )
install(TARGETS demo02_recieve
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

ament_package()
