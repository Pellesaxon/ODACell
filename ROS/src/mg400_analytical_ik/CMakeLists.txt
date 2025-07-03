cmake_minimum_required(VERSION 3.8)
project(mg400_analytical_ik)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(kdl_parser REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/mg400_kinematics_plugin.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  moveit_core
  pluginlib
  rclcpp
  tf2_kdl
  kdl_parser
)

# Link against the libraries of the dependencies
target_link_libraries(${PROJECT_NAME}
  ${moveit_core_LIBRARIES}
  ${pluginlib_LIBRARIES}
  ${kdl_parser_LIBRARIES}
)

# Export the plugin definition
pluginlib_export_plugin_description_file(moveit_core mg400_kinematics_plugins.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(FILES mg400_kinematics_plugins.xml
  DESTINATION share/${PROJECT_NAME})

ament_package()
