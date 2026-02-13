#include "diffbot_hardware/diffbot_hardware.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace diffbot_hardware
{

hardware_interface::CallbackReturn
TeensyDiffDrive::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
TeensyDiffDrive::export_state_interfaces()
{
  return {
    hardware_interface::StateInterface(
      info_.joints[0].name, hardware_interface::HW_IF_POSITION, &left_pos_),
    hardware_interface::StateInterface(
      info_.joints[1].name, hardware_interface::HW_IF_POSITION, &right_pos_)
  };
}

std::vector<hardware_interface::CommandInterface>
TeensyDiffDrive::export_command_interfaces()
{
  return {
    hardware_interface::CommandInterface(
      info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &left_cmd_),
    hardware_interface::CommandInterface(
      info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &right_cmd_)
  };
}

hardware_interface::CallbackReturn
TeensyDiffDrive::on_activate(const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
TeensyDiffDrive::on_deactivate(const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
TeensyDiffDrive::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
TeensyDiffDrive::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  return hardware_interface::return_type::OK;
}

}  // namespace diffbot_hardware


PLUGINLIB_EXPORT_CLASS(
  diffbot_hardware::TeensyDiffDrive,
  hardware_interface::SystemInterface
)
