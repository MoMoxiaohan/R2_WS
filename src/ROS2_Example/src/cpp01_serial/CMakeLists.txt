cmake_minimum_required(VERSION 3.8)
project(cpp01_serial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial_driver REQUIRED)
find_package(geometry_msgs REQUIRED)
add_executable(demo01_uart src/demo01_uart.cpp src/serial_pack.cpp)  #需要修改这里
add_executable(demo02_recieve src/demo02_recieve.cpp src/serial_pack.cpp)
target_include_directories(demo01_uart PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_include_directories(demo02_recieve PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(demo01_uart PUBLIC c_std_99 cxx_std_17)
target_compile_features(demo02_recieve PUBLIC c_std_99 cxx_std_17)   # Require C99 and C++17
ament_target_dependencies(
  demo01_uart
  "rclcpp"
  "serial_driver"
  "geometry_msgs"
)
ament_target_dependencies(
  demo02_recieve
  "rclcpp"
  "serial_driver"
)


install(TARGETS demo01_uart
  DESTINATION lib/${PROJECT_NAME})
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
