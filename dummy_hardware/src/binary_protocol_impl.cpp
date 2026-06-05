/**
 * 二进制协议实现（添加到 dummy_robot_system.cpp 的新函数）
 *
 * 这些函数需要添加到现有的 dummy_robot_system.cpp 文件中
 */

#include "dummy_hardware/dummy_robot_system.hpp"
#include "rclcpp/logging.hpp"

#include <sys/select.h>
#include <unistd.h>
#include <cstring>
#include <chrono>

namespace dummy_hardware
{

// ============================================================================
// 二进制帧读写底层函数
// ============================================================================

/**
 * 写入二进制帧到串口
 */
bool DummyRobotSystem::write_binary_frame(const void * frame, size_t size)
{
  if (!is_port_open()) {
    RCLCPP_ERROR(rclcpp::get_logger("dummy_hardware"), "Serial port not open");
    return false;
  }

  const auto written = ::write(serial_fd_, frame, size);
  if (written != static_cast<ssize_t>(size)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("dummy_hardware"),
      "Binary write failed: expected %zu bytes, wrote %zd",
      size, written);
    return false;
  }

  // 等待发送完成
  tcdrain(serial_fd_);
  return true;
}

/**
 * 从串口读取二进制帧（带超时）
 */
bool DummyRobotSystem::read_binary_frame(void * frame, size_t expected_size, int timeout_ms)
{
  if (!is_port_open()) {
    return false;
  }

  uint8_t * buffer = static_cast<uint8_t *>(frame);
  size_t bytes_read = 0;
  bool found_header = false;

  const auto start_time = std::chrono::steady_clock::now();
  const auto timeout_duration = std::chrono::milliseconds(timeout_ms);

  while (bytes_read < expected_size) {
    // 检查超时
    auto elapsed = std::chrono::steady_clock::now() - start_time;
    if (elapsed >= timeout_duration) {
      return false;
    }

    // 使用 select 等待数据
    fd_set read_set;
    FD_ZERO(&read_set);
    FD_SET(serial_fd_, &read_set);

    auto remaining_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      timeout_duration - elapsed).count();
    timeval tv;
    tv.tv_sec = remaining_ms / 1000;
    tv.tv_usec = (remaining_ms % 1000) * 1000;

    const auto ready = select(serial_fd_ + 1, &read_set, nullptr, nullptr, &tv);
    if (ready <= 0) {
      continue;
    }

    // 读取一个字节
    uint8_t byte;
    const auto n = ::read(serial_fd_, &byte, 1);
    if (n <= 0) {
      continue;
    }

    // 状态机：查找 header
    if (!found_header) {
      if (byte == BINARY_HEADER) {
        buffer[0] = byte;
        bytes_read = 1;
        found_header = true;
      }
      continue;
    }

    // 已找到 header，继续读取
    buffer[bytes_read++] = byte;

    // 检查是否读取完成
    if (bytes_read == expected_size) {
      if (buffer[expected_size - 1] == BINARY_FOOTER) {
        return true;
      } else {
        found_header = false;
        bytes_read = 0;
      }
    }
  }

  return false;
}

/**
 * 发送运动命令（二进制）
 */
bool DummyRobotSystem::send_motion_command_binary(
  const std::array<float, 6> & joint_positions_deg)
{
  MotionCommandFrame frame;
  build_motion_command(frame, joint_positions_deg.data());

  return write_binary_frame(&frame, MotionCommandFrame::SIZE);
}

/**
 * 读取状态响应（二进制）
 */
bool DummyRobotSystem::read_state_response_binary(StateResponseFrame & response)
{
  if (!read_binary_frame(&response, StateResponseFrame::SIZE, read_timeout_ms_)) {
    return false;
  }

  if (!validate_state_response(response)) {
    return false;
  }

  return true;
}

/**
 * 发送简单命令（二进制）
 */
bool DummyRobotSystem::send_simple_command_binary(CommandType cmd_type, uint8_t param)
{
  SimpleCommandFrame frame;
  build_simple_command(frame, cmd_type, param);

  return write_binary_frame(&frame, SimpleCommandFrame::SIZE);
}

/**
 * 读取简单响应（二进制）
 */
bool DummyRobotSystem::read_simple_response_binary(SimpleResponseFrame & response)
{
  if (!read_binary_frame(&response, SimpleResponseFrame::SIZE, read_timeout_ms_)) {
    return false;
  }

  if (!validate_simple_response(response)) {
    return false;
  }

  return true;
}

}  // namespace dummy_hardware
