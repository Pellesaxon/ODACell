cmake_minimum_required(VERSION 3.8)
project(hg_c1030_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hg_c1030_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(hg_sensor_node src/hg_sensor_node.cpp)

target_include_directories(hg_sensor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(hg_sensor_node
  rclcpp
  rclcpp_action
  sensor_msgs
  hg_c1030_msgs
  std_srvs
)

install(TARGETS
  hg_sensor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()