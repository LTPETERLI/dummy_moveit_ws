#include "dummy_hardware/dummy_robot_system.hpp"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include <utility>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr std::size_t kArmJointCount = 6;
constexpr std::size_t kGripperIndex = 6;
constexpr std::size_t kTotalJoints = 7;

double degrees_to_radians(double degrees)
{
  return degrees * kPi / 180.0;
}

double radians_to_degrees(double radians)
{
  return radians * 180.0 / kPi;
}

std::string trim_ascii(std::string value)
{
  const auto not_space = [](unsigned char ch) {
      return !std::isspace(ch);
    };
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
  value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
  return value;
}

bool parse_csv_doubles(const std::string & csv, std::array<double, 6> & out)
{
  std::stringstream stream(csv);
  std::string token;
  std::size_t index = 0;
  while (std::getline(stream, token, ',')) {
    if (index >= out.size()) {
      return false;
    }
    try {
      out[index] = std::stod(trim_ascii(token));
    } catch (const std::exception &) {
      return false;
    }
    ++index;
  }
  return index == out.size();
}

}  // namespace

namespace dummy_hardware
{

hardware_interface::CallbackReturn DummyRobotSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != kTotalJoints) {
    RCLCPP_ERROR(
      rclcpp::get_logger("dummy_hardware"),
      "Expected %zu joints in URDF (6 arm + 1 gripper), got %zu.",
      kTotalJoints, info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (std::size_t joint_number = 0; joint_number < kArmJointCount; ++joint_number) {
    const auto & joint = info_.joints[joint_number];
    const auto expected_name = std::string("Joint") + std::to_string(joint_number + 1);
    if (joint.name != expected_name) {
      RCLCPP_ERROR(
        rclcpp::get_logger("dummy_hardware"),
        "Arm joint order mismatch. Expected %s, got %s.",
        expected_name.c_str(), joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  if (info_.joints[kGripperIndex].name != "Joint7") {
    RCLCPP_ERROR(
      rclcpp::get_logger("dummy_hardware"),
      "Gripper joint mismatch. Expected Joint7, got %s.",
      info_.joints[kGripperIndex].name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_states_position_.assign(info_.joints.size(), 0.0);
  hw_states_velocity_.assign(info_.joints.size(), 0.0);
  hw_commands_position_.assign(info_.joints.size(), 0.0);
  last_hw_commands_position_.assign(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  last_hw_states_position_.assign(info_.joints.size(), 0.0);

  const auto & params = info_.hardware_parameters;
  if (params.count("port") != 0U) {
    serial_port_ = params.at("port");
  }
  if (params.count("baud_rate") != 0U) {
    baud_rate_ = std::stoi(params.at("baud_rate"));
  }
  if (params.count("read_timeout_ms") != 0U) {
    read_timeout_ms_ = std::stoi(params.at("read_timeout_ms"));
  }
  if (params.count("activate_delay_ms") != 0U) {
    activate_delay_ms_ = std::stoi(params.at("activate_delay_ms"));
  }
  if (params.count("command_mode") != 0U) {
    command_mode_ = std::stoi(params.at("command_mode"));
  }
  if (params.count("startup_enable") != 0U) {
    startup_enable_ = params.at("startup_enable") == "true";
  }
  if (params.count("write_epsilon") != 0U) {
    write_epsilon_ = std::stod(params.at("write_epsilon"));
  }
  if (params.count("joint_offset_deg") != 0U) {
    if (!parse_csv_doubles(params.at("joint_offset_deg"), joint_offset_deg_)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("dummy_hardware"),
        "Failed to parse joint_offset_deg='%s' (need 6 comma-separated numbers).",
        params.at("joint_offset_deg").c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  if (params.count("joint_sign") != 0U) {
    if (!parse_csv_doubles(params.at("joint_sign"), joint_sign_)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("dummy_hardware"),
        "Failed to parse joint_sign='%s' (need 6 comma-separated numbers).",
        params.at("joint_sign").c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  if (params.count("gripper_open_value") != 0U) {
    gripper_open_value_ = std::stod(params.at("gripper_open_value"));
  }
  if (params.count("gripper_closed_value") != 0U) {
    gripper_closed_value_ = std::stod(params.at("gripper_closed_value"));
  }
  if (params.count("gripper_threshold") != 0U) {
    gripper_threshold_ = std::stod(params.at("gripper_threshold"));
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DummyRobotSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!open_port()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DummyRobotSystem::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  close_port();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DummyRobotSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!is_port_open() && !open_port()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Drain any stale data left in the serial buffer from a previous session.
  tcflush(serial_fd_, TCIOFLUSH);
  usleep(200 * 1000);
  tcflush(serial_fd_, TCIOFLUSH);

  if (startup_enable_) {
    if (!send_simple_command("!START")) {
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (!send_simple_command("#CMDMODE " + std::to_string(command_mode_))) {
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (!send_simple_command("!HAND_EN")) {
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  last_gripper_state_ = GripperState::Unknown;
  last_gripper_command_value_ = gripper_closed_value_;

  if (activate_delay_ms_ > 0) {
    usleep(static_cast<useconds_t>(activate_delay_ms_ * 1000));
  }

  if (read(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)) !=
    hardware_interface::return_type::OK)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_states_position_[kGripperIndex] = gripper_closed_value_;
  hw_states_velocity_[kGripperIndex] = 0.0;

  hw_commands_position_ = hw_states_position_;
  last_hw_commands_position_ = hw_commands_position_;
  last_hw_states_position_ = hw_states_position_;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DummyRobotSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Best-effort safe shutdown; ignore failures so we always close the port.
  send_simple_command("!HAND_DIS");
  send_simple_command("!DISABLE");
  close_port();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DummyRobotSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(info_.joints.size() * 2);

  for (std::size_t index = 0; index < info_.joints.size(); ++index) {
    state_interfaces.emplace_back(
      info_.joints[index].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[index]);
    state_interfaces.emplace_back(
      info_.joints[index].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[index]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DummyRobotSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());

  for (std::size_t index = 0; index < info_.joints.size(); ++index) {
    command_interfaces.emplace_back(
      info_.joints[index].name, hardware_interface::HW_IF_POSITION, &hw_commands_position_[index]);
  }

  return command_interfaces;
}

hardware_interface::return_type DummyRobotSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!is_port_open()) {
    return hardware_interface::return_type::ERROR;
  }

  std::string reply;
  if (!query_line("#GETJPOS", reply)) {
    return hardware_interface::return_type::ERROR;
  }

  std::vector<double> joints_deg;
  if (!parse_joint_reply(reply, joints_deg)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("dummy_hardware"),
      "Failed to parse #GETJPOS reply: %s",
      reply.c_str());
    return hardware_interface::return_type::ERROR;
  }

  const double dt = period.seconds();
  for (std::size_t j = 0; j < kArmJointCount; ++j) {
    const auto previous_position = hw_states_position_[j];
    const auto current_position =
      degrees_to_radians((joints_deg[j] - joint_offset_deg_[j]) * joint_sign_[j]);
    hw_states_position_[j] = current_position;
    hw_states_velocity_[j] = dt > 0.0 ? (current_position - previous_position) / dt : 0.0;
  }

  hw_states_position_[kGripperIndex] = last_gripper_command_value_;
  hw_states_velocity_[kGripperIndex] = 0.0;

  last_hw_states_position_ = hw_states_position_;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DummyRobotSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!is_port_open()) {
    return hardware_interface::return_type::ERROR;
  }

  bool arm_changed = false;
  for (std::size_t j = 0; j < kArmJointCount; ++j) {
    if (std::isnan(last_hw_commands_position_[j]) ||
      std::fabs(hw_commands_position_[j] - last_hw_commands_position_[j]) > write_epsilon_)
    {
      arm_changed = true;
      break;
    }
  }

  if (arm_changed) {
    std::ostringstream command;
    command << std::fixed << std::setprecision(3) << "&";
    for (std::size_t j = 0; j < kArmJointCount; ++j) {
      if (j != 0U) {
        command << ",";
      }
      command << radians_to_degrees(hw_commands_position_[j]) * joint_sign_[j] + joint_offset_deg_[j];
    }
    if (!send_simple_command(command.str())) {
      return hardware_interface::return_type::ERROR;
    }
    for (std::size_t j = 0; j < kArmJointCount; ++j) {
      last_hw_commands_position_[j] = hw_commands_position_[j];
    }
  }

  const double gripper_cmd = hw_commands_position_[kGripperIndex];
  const bool want_open = gripper_cmd > gripper_threshold_;
  const auto desired_state = want_open ? GripperState::Open : GripperState::Closed;
  if (desired_state != last_gripper_state_) {
    const std::string cmd = want_open ? "!HAND_O" : "!HAND_C";
    if (!send_simple_command(cmd)) {
      return hardware_interface::return_type::ERROR;
    }
    last_gripper_state_ = desired_state;
    last_gripper_command_value_ = want_open ? gripper_open_value_ : gripper_closed_value_;
  }
  last_hw_commands_position_[kGripperIndex] = hw_commands_position_[kGripperIndex];

  return hardware_interface::return_type::OK;
}

bool DummyRobotSystem::open_port()
{
  if (is_port_open()) {
    return true;
  }

  serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("dummy_hardware"),
      "Failed to open serial port %s: %s",
      serial_port_.c_str(), std::strerror(errno));
    return false;
  }

  termios tty {};
  if (tcgetattr(serial_fd_, &tty) != 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("dummy_hardware"),
      "tcgetattr failed for %s: %s",
      serial_port_.c_str(), std::strerror(errno));
    close_port();
    return false;
  }

  cfmakeraw(&tty);
  const auto termios_baud = termios_baud_rate(baud_rate_);
  if (cfsetispeed(&tty, termios_baud) != 0 || cfsetospeed(&tty, termios_baud) != 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("dummy_hardware"),
      "Failed to set serial baud rate %d for %s.",
      baud_rate_, serial_port_.c_str());
    close_port();
    return false;
  }

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("dummy_hardware"),
      "tcsetattr failed for %s: %s",
      serial_port_.c_str(), std::strerror(errno));
    close_port();
    return false;
  }

  tcflush(serial_fd_, TCIOFLUSH);
  return true;
}

void DummyRobotSystem::close_port()
{
  if (serial_fd_ >= 0) {
    close(serial_fd_);
    serial_fd_ = -1;
  }
}

bool DummyRobotSystem::is_port_open() const
{
  return serial_fd_ >= 0;
}

bool DummyRobotSystem::write_line(const std::string & line)
{
  if (!is_port_open()) {
    return false;
  }

  const bool already_terminated =
    line.size() >= 2 && line[line.size() - 2] == '\r' && line.back() == '\n';
  const auto payload = already_terminated ? line : line + "\r\n";
  const auto written = ::write(serial_fd_, payload.c_str(), payload.size());
  if (written != static_cast<ssize_t>(payload.size())) {
    RCLCPP_ERROR(
      rclcpp::get_logger("dummy_hardware"),
      "Serial write failed for command '%s': %s",
      line.c_str(), std::strerror(errno));
    return false;
  }
  tcdrain(serial_fd_);
  return true;
}

bool DummyRobotSystem::read_line(std::string & line, int timeout_ms) const
{
  if (!is_port_open()) {
    return false;
  }

  line.clear();
  const auto timeout_s = timeout_ms / 1000;
  const auto timeout_us = (timeout_ms % 1000) * 1000;

  while (true) {
    fd_set read_set;
    FD_ZERO(&read_set);
    FD_SET(serial_fd_, &read_set);

    timeval timeout {};
    timeout.tv_sec = timeout_s;
    timeout.tv_usec = timeout_us;

    const auto ready = select(serial_fd_ + 1, &read_set, nullptr, nullptr, &timeout);
    if (ready <= 0) {
      return false;
    }

    char ch = '\0';
    const auto bytes = ::read(serial_fd_, &ch, 1);
    if (bytes <= 0) {
      return false;
    }

    if (ch == '\n' || ch == '\r') {
      line = trim_ascii(line);
      if (!line.empty()) {
        return true;
      }
      continue;
    }

    line.push_back(ch);
  }
}

bool DummyRobotSystem::query_line(const std::string & request, std::string & response)
{
  if (!write_line(request)) {
    return false;
  }
  return read_line(response, read_timeout_ms_);
}

bool DummyRobotSystem::send_simple_command(const std::string & command)
{
  std::string response;
  if (!query_line(command, response)) {
    return false;
  }

  // Motion commands (& / @ / >) produce two responses in CMDMODE 2:
  // first the FIFO push reply (a number), then a delayed "ok" from the
  // command handler task. Drain that second line here so it does not get
  // mistaken for the next command's reply.
  if (!command.empty() && (command[0] == '&' || command[0] == '@' || command[0] == '>')) {
    std::string secondary;
    read_line(secondary, 100);  // best-effort
  }

  // Accept any response containing "ok" (covers "ok ...", "Started ok",
  // "Stopped ok", "Disabled ok", "ok hand enable real_angle:..." etc.)
  if (response.find("ok") != std::string::npos) {
    return true;
  }

  // Reject explicit error responses
  if (response.find("error") != std::string::npos) {
    RCLCPP_ERROR(
      rclcpp::get_logger("dummy_hardware"),
      "Firmware reported error for command '%s': %s",
      command.c_str(), response.c_str());
    return false;
  }

  // Accept numeric-only response (& / @ / > push reply = FIFO free space)
  bool is_number = !response.empty();
  for (std::size_t i = 0; i < response.size(); ++i) {
    const char c = response[i];
    const bool valid_digit = std::isdigit(static_cast<unsigned char>(c)) != 0;
    const bool sign = (i == 0U) && (c == '-' || c == '+');
    if (!valid_digit && !sign) {
      is_number = false;
      break;
    }
  }
  if (is_number) {
    return true;
  }

  RCLCPP_ERROR(
    rclcpp::get_logger("dummy_hardware"),
    "Unexpected response for command '%s': %s",
    command.c_str(), response.c_str());
  return false;
}

bool DummyRobotSystem::parse_joint_reply(
  const std::string & reply, std::vector<double> & joints_deg) const
{
  joints_deg.clear();
  std::istringstream stream(reply);
  std::string status;
  stream >> status;
  if (status != "ok") {
    return false;
  }

  double value = 0.0;
  while (stream >> value) {
    joints_deg.push_back(value);
  }

  return joints_deg.size() == kArmJointCount;
}

bool DummyRobotSystem::parse_single_reply(const std::string & reply, double & value)
{
  std::istringstream stream(reply);
  std::string status;
  stream >> status;
  if (status != "ok") {
    return false;
  }
  return static_cast<bool>(stream >> value);
}

unsigned int DummyRobotSystem::termios_baud_rate(int baud_rate)
{
  switch (baud_rate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    default:
      return B115200;
  }
}

}  // namespace dummy_hardware

PLUGINLIB_EXPORT_CLASS(dummy_hardware::DummyRobotSystem, hardware_interface::SystemInterface)
