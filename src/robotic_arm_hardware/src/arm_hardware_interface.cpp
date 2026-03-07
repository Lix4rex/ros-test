#include "robotic_arm_hardware/arm_hardware_interface.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <cmath>
#include <algorithm>
#include <cstring>

// ==========================
// CONFIG
// ==========================
static const std::vector<std::string> JOINT_NAMES = {
  "first_arm_joint",
  "second_arm_joint",
  "third_arm_joint",
  "fourth_arm_joint"
};

static const char * SERIAL_PORT = "/dev/ttyUSB0";
static int serial_fd = -1;

// ==========================
// SERIAL UTILS
// ==========================
static int open_serial(const char *device)
{
  int fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0)
    return -1;

  termios tty{};
  tcgetattr(fd, &tty);

  cfsetospeed(&tty, B115200);
  cfsetispeed(&tty, B115200);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  tty.c_iflag = 0;
  tty.c_oflag = 0;
  tty.c_lflag = 0;

  tcsetattr(fd, TCSANOW, &tty);
  return fd;
}

// ==========================
// ROS2 CONTROL
// ==========================
hardware_interface::CallbackReturn
ArmHardwareInterface::on_init(const hardware_interface::HardwareInfo &)
{
  hw_positions_.resize(JOINT_NAMES.size(), 0.0);
  hw_commands_.resize(JOINT_NAMES.size(), 0.0);

  RCLCPP_INFO(
    rclcpp::get_logger("ArmHardwareInterface"),
    "ESP32 robotic arm hardware initialized"
  );

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ArmHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> states;

  for (size_t i = 0; i < JOINT_NAMES.size(); ++i)
  {
    states.emplace_back(
      JOINT_NAMES[i],
      hardware_interface::HW_IF_POSITION,
      &hw_positions_[i]
    );
  }

  return states;
}

std::vector<hardware_interface::CommandInterface>
ArmHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> cmds;

  for (size_t i = 0; i < JOINT_NAMES.size(); ++i)
  {
    cmds.emplace_back(
      JOINT_NAMES[i],
      hardware_interface::HW_IF_POSITION,
      &hw_commands_[i]
    );
  }

  return cmds;
}

hardware_interface::return_type
ArmHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  hw_positions_ = hw_commands_;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
ArmHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (serial_fd < 0)
  {
    serial_fd = open_serial(SERIAL_PORT);
    if (serial_fd < 0)
    {
      RCLCPP_WARN(
        rclcpp::get_logger("ArmHardwareInterface"),
        "ESP32 not connected (%s)", SERIAL_PORT
      );
      return hardware_interface::return_type::OK;
    }
  }

  int angles[4];

  for (size_t i = 0; i < hw_commands_.size(); ++i)
  {
    double deg = hw_commands_[i] * 180.0 / M_PI;
    angles[i] = std::clamp(static_cast<int>(deg), 0, 180);
  }

  char buffer[64];
  snprintf(buffer, sizeof(buffer),
           "S,%d,%d,%d,%d\n",
           angles[0], angles[1], angles[2], angles[3]);

           ::write(serial_fd, buffer, strlen(buffer));


  return hardware_interface::return_type::OK;
}

PLUGINLIB_EXPORT_CLASS(
  ArmHardwareInterface,
  hardware_interface::SystemInterface
)
