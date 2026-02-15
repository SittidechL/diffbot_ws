cmake_minimum_required(VERSION 3.8)
project(diffbot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

include_directories(include)

add_library(diffbot_hardware SHARED
  src/diffbot_system.cpp
)

ament_target_dependencies(diffbot_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(
  hardware_interface diffbot_hardware.xml
)

install(
  TARGETS diffbot_hardware
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  FILES diffbot_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
