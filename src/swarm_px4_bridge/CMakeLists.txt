cmake_minimum_required(VERSION 3.8)
project(swarm_px4_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(swarm_interfaces REQUIRED)

add_executable(telemetry_node
  src/telemetry_node.cpp
)
add_executable(swarm_control_node
  src/swarm_control_node.cpp
)

ament_target_dependencies(
  telemetry_node rclcpp px4_msgs
)

ament_target_dependencies(
  swarm_control_node PUBLIC swarm_interfaces
)

target_compile_features(telemetry_node PRIVATE cxx_std_17)
target_compile_features(swarm_control_node PRIVATE cxx_std_17)

target_link_libraries(swarm_control_node PRIVATE
  px4_ros2_cpp::px4_ros2_cpp
  rclcpp::rclcpp
)


install(TARGETS
  telemetry_node
  swarm_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
