cmake_minimum_required(VERSION 3.8)
project(mg400_ros2_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(mg400_msgs REQUIRED)
find_package(std_srvs REQUIRED) #

# Build the hardware interface plugin
add_library(mg400_system_hardware SHARED
  src/mg400_system_hardware.cpp
)
ament_target_dependencies(mg400_system_hardware
  rclcpp
  hardware_interface
  pluginlib
)
# Make sure the plugin can find its own header
target_include_directories(mg400_system_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
pluginlib_export_plugin_description_file(hardware_interface src/ros2_control.xml)

# Build the driver node executable
add_executable(mg400_driver_node src/mg400_driver_node.cpp)
ament_target_dependencies(mg400_driver_node
  rclcpp
  rclcpp_action
  mg400_msgs
  std_srvs
)
target_include_directories(mg400_driver_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(mg400_driver_node rt) 
# Make sure the executable can find its own header
target_include_directories(mg400_driver_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
# Link against the real-time library for shared memory
target_link_libraries(mg400_driver_node rt) 

# Install targets
install(TARGETS
  mg400_system_hardware
  mg400_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install all headers from the include directory
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES src/ros2_control.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()