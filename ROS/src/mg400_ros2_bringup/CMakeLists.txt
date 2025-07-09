cmake_minimum_required(VERSION 3.8)
project(mg400_ros2_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(mg400_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)

# --- Plugin Library (fast_start_slow_end_adapter) ---
add_library(fast_start_slow_end_adapter_lib SHARED
  src/fast_start_slow_end_adapter.cpp
)
ament_target_dependencies(fast_start_slow_end_adapter_lib
  moveit_core
  moveit_ros_planning
  rclcpp
  pluginlib
)
target_include_directories(fast_start_slow_end_adapter_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
pluginlib_export_plugin_description_file(moveit_core fast_start_slow_end_plugin.xml)

# --- Driver Node Executable ---
add_executable(mg400_driver_node src/mg400_driver_node.cpp)
ament_target_dependencies(mg400_driver_node
  rclcpp
  rclcpp_action
  mg400_msgs
  std_srvs
  control_msgs
  pluginlib
  moveit_core
  moveit_ros_planning
)
target_include_directories(mg400_driver_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
# Link against the real-time library for shared memory
target_link_libraries(mg400_driver_node rt)

# --- Install Rules ---
install(TARGETS
  mg400_driver_node
  fast_start_slow_end_adapter_lib
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install configuration, launch, and model files
install(
  DIRECTORY launch urdf config mock
  DESTINATION share/${PROJECT_NAME}
)

# Install plugin XML
install(
  FILES fast_start_slow_end_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()