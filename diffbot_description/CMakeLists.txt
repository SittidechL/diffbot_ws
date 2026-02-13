cmake_minimum_required(VERSION 3.8)
project(diffbot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(diffbot_hardware SHARED
  src/diffbot_hardware.cpp
)

ament_target_dependencies(diffbot_hardware
  hardware_interface
  pluginlib
  rclcpp
)

pluginlib_export_plugin_description_file(
  hardware_interface
  plugin.xml
)

install(
  TARGETS diffbot_hardware
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(diffbot_hardware)
ament_export_dependencies(hardware_interface pluginlib rclcpp)

ament_package()
