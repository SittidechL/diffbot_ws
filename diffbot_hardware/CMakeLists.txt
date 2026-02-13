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
  FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)
