#ifndef DUMMY_HARDWARE__DUMMY_ROBOT_SYSTEM_HPP_
#define DUMMY_HARDWARE__DUMMY_ROBOT_SYSTEM_HPP_

#include <array>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "dummy_hardware/binary_protocol.hpp"

namespace dummy_hardware
{

class DummyRobotSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DummyRobotSystem)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ====== 串口操作（保留旧接口兼容） ======
  bool open_port();
  void close_port();
  bool is_port_open() const;
  bool write_line(const std::string & line);
  bool read_line(std::string & line, int timeout_ms) const;
  bool query_line(const std::string & request, std::string & response);
  bool send_simple_command(const std::string & command);
  bool parse_joint_reply(const std::string & reply, std::vector<double> & joints_deg) const;
  static bool parse_single_reply(const std::string & reply, double & value);

  // ====== 二进制协议接口（新增） ======
  bool write_binary_frame(const void * frame, size_t size);
  bool read_binary_frame(void * frame, size_t expected_size, int timeout_ms);
  bool send_motion_command_binary(const std::array<float, 6> & joint_positions_deg);
  bool read_state_response_binary(StateResponseFrame & response);
  bool send_simple_command_binary(CommandType cmd_type, uint8_t param = 0);
  bool read_simple_response_binary(SimpleResponseFrame & response);

  static unsigned int termios_baud_rate(int baud_rate);

  // ====== 配置参数 ======
  int serial_fd_{-1};
  std::string serial_port_{"/dev/ttyACM0"};
  int baud_rate_{115200};
  int read_timeout_ms_{80};
  int activate_delay_ms_{200};
  int command_mode_{2};
  bool startup_enable_{true};
  double write_epsilon_{1e-4};

  // 新增：是否使用二进制协议
  bool use_binary_protocol_{true};  // 默认使用二进制协议

  std::array<double, 6> joint_offset_deg_{{0.0, 0.0, 90.0, 0.0, 0.0, 0.0}};
  std::array<double, 6> joint_sign_{{1.0, 1.0, 1.0, 1.0, -1.0, -1.0}};

  double gripper_open_value_{0.012};
  double gripper_closed_value_{0.0};
  double gripper_threshold_{0.006};

  enum class GripperState { Unknown, Open, Closed };
  GripperState last_gripper_state_{GripperState::Unknown};
  double last_gripper_command_value_{0.0};

  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;
  std::vector<double> hw_commands_position_;
  std::vector<double> last_hw_commands_position_;
  std::vector<double> last_hw_states_position_;

  // 新增：二进制协议缓冲区
  uint8_t rx_buffer_[256];
  size_t rx_buffer_pos_{0};
};

}  // namespace dummy_hardware

#endif  // DUMMY_HARDWARE__DUMMY_ROBOT_SYSTEM_HPP_
