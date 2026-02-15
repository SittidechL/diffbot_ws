#include "diffbot_hardware/diffbot_system.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <iostream>
#include <cmath>

namespace diffbot_hardware
{

hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_positions_.assign(info.joints.size(), 0.0);
  hw_velocities_.assign(info.joints.size(), 0.0);
  hw_commands_.assign(info.joints.size(), 0.0);

  // --------- Open Serial ---------
  serial_fd_ = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
  if (serial_fd_ < 0)
  {
    perror("Error opening serial port");
    return hardware_interface::CallbackReturn::ERROR;
  }

  struct termios tty;
  memset(&tty, 0, sizeof tty);

  if (tcgetattr(serial_fd_, &tty) != 0)
  {
    perror("Error from tcgetattr");
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfsetospeed(&tty, B115200);
  cfsetispeed(&tty, B115200);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_iflag = 0;

  tcsetattr(serial_fd_, TCSANOW, &tty);

  std::cout << "Serial port opened successfully!" << std::endl;

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
DiffBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < hw_positions_.size(); ++i)
  {
    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_positions_[i]);

    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY,
      &hw_velocities_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
DiffBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < hw_commands_.size(); ++i)
  {
    command_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY,
      &hw_commands_[i]);
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (serial_fd_ > 0)
  {
    close(serial_fd_);
    serial_fd_ = -1;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffBotSystemHardware::read(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (serial_fd_ < 0)
    return hardware_interface::return_type::ERROR;

  static std::string serial_buffer;
  char temp[128];

  int n = ::read(serial_fd_, temp, sizeof(temp));
  if (n > 0)
  {
    serial_buffer.append(temp, n);

    size_t pos;
    while ((pos = serial_buffer.find('\n')) != std::string::npos)
    {
      std::string line = serial_buffer.substr(0, pos);
      serial_buffer.erase(0, pos + 1);

      double left_pos = 0.0;
      double left_vel = 0.0;
      double right_pos = 0.0;
      double right_vel = 0.0;

      if (sscanf(line.c_str(), "O %lf %lf %lf %lf",
                 &left_pos,
                 &left_vel,
                 &right_pos,
                 &right_vel) == 4)
      {
        hw_positions_[0] = left_pos;
        hw_positions_[1] = right_pos;

        hw_velocities_[0] = left_vel;
        hw_velocities_[1] = right_vel;
      }
    }
  }

  return hardware_interface::return_type::OK;
}



hardware_interface::return_type DiffBotSystemHardware::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (serial_fd_ < 0)
    return hardware_interface::return_type::ERROR;

  char buffer[64];

  int length = snprintf(
    buffer,
    sizeof(buffer),
    "C %.3f %.3f\n",
    hw_commands_[0],
    hw_commands_[1]);

  if (length > 0)
  {
    ::write(serial_fd_, buffer, length);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace diffbot_hardware

PLUGINLIB_EXPORT_CLASS(
  diffbot_hardware::DiffBotSystemHardware,
  hardware_interface::SystemInterface)
