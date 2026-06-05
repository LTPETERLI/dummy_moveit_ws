#ifndef DUMMY_HARDWARE__BINARY_PROTOCOL_HPP_
#define DUMMY_HARDWARE__BINARY_PROTOCOL_HPP_

#include <cstdint>
#include <cstring>

/**
 * 二进制协议定义（ROS 2 和 STM32 共用）
 *
 * 设计目标：
 * - 紧凑：27字节（比原来的99字节少73%）
 * - 快速：无需字符串解析
 * - 可靠：CRC16校验
 */

namespace dummy_hardware
{

// ============================================================================
// 常量定义
// ============================================================================

constexpr uint8_t BINARY_HEADER = 0xAA;     // 同步字节
constexpr uint8_t BINARY_FOOTER = 0x55;     // 结束字节

// 命令类型
enum class CommandType : uint8_t
{
  MOTION_COMMAND = 0x01,      // 运动命令（6个关节位置）
  GRIPPER_COMMAND = 0x02,     // 夹爪命令（开/关）
  QUERY_STATE = 0x03,         // 查询状态
  ENABLE_MOTORS = 0x10,       // 使能电机
  DISABLE_MOTORS = 0x11,      // 失能电机
  HOMING = 0x12,              // 回零
  SET_MODE = 0x13             // 设置控制模式
};

// 响应类型
enum class ResponseType : uint8_t
{
  STATE_REPLY = 0x81,         // 状态响应（6个关节位置）
  ACK = 0x82,                 // 确认响应
  ERROR = 0x83                // 错误响应
};

// 错误码
enum class ErrorCode : uint8_t
{
  NONE = 0x00,
  CHECKSUM_ERROR = 0x01,
  INVALID_COMMAND = 0x02,
  OUT_OF_RANGE = 0x03,
  MOTOR_ERROR = 0x04,
  TIMEOUT = 0x05
};

// ============================================================================
// 协议结构体定义（必须使用 packed 避免内存对齐）
// ============================================================================

// 运动命令帧（发送：ROS 2 → STM32）
struct __attribute__((packed)) MotionCommandFrame
{
  uint8_t header;              // 0xAA
  uint8_t cmd_type;            // CommandType::MOTION_COMMAND
  float joint_positions[6];    // 6个关节位置（度）
  uint16_t checksum;           // CRC16校验
  uint8_t footer;              // 0x55

  static constexpr size_t SIZE = 30;  // 1 + 1 + 24 + 2 + 1 + 1 = 30字节
};

// 状态响应帧（接收：STM32 → ROS 2）
struct __attribute__((packed)) StateResponseFrame
{
  uint8_t header;              // 0xAA
  uint8_t response_type;       // ResponseType::STATE_REPLY
  float joint_positions[6];    // 6个关节位置（度）
  float joint_velocities[6];   // 6个关节速度（度/秒，可选）
  uint8_t gripper_state;       // 夹爪状态（0=关闭，1=打开）
  uint8_t error_code;          // 错误码
  uint16_t checksum;           // CRC16校验
  uint8_t footer;              // 0x55

  static constexpr size_t SIZE = 56;  // 1 + 1 + 24 + 24 + 1 + 1 + 2 + 1 + 1 = 56字节
};

// 简单命令帧（夹爪、使能等）
struct __attribute__((packed)) SimpleCommandFrame
{
  uint8_t header;              // 0xAA
  uint8_t cmd_type;            // CommandType
  uint8_t param;               // 参数（如：夹爪 0=关闭 1=打开）
  uint16_t checksum;           // CRC16
  uint8_t footer;              // 0x55

  static constexpr size_t SIZE = 6;
};

// 简单响应帧（ACK/ERROR）
struct __attribute__((packed)) SimpleResponseFrame
{
  uint8_t header;              // 0xAA
  uint8_t response_type;       // ResponseType::ACK or ERROR
  uint8_t error_code;          // ErrorCode
  uint16_t checksum;           // CRC16
  uint8_t footer;              // 0x55

  static constexpr size_t SIZE = 6;
};

// ============================================================================
// CRC16 校验函数
// ============================================================================

/**
 * CRC16-CCITT 校验算法
 * 多项式：0x1021
 * 初值：0xFFFF
 */
inline uint16_t calculate_crc16(const uint8_t * data, size_t length)
{
  uint16_t crc = 0xFFFF;

  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;

    for (uint8_t bit = 0; bit < 8; ++bit) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc = crc << 1;
      }
    }
  }

  return crc;
}

/**
 * 验证 CRC16 校验码
 */
inline bool verify_crc16(const uint8_t * data, size_t length, uint16_t expected_crc)
{
  return calculate_crc16(data, length) == expected_crc;
}

// ============================================================================
// 辅助函数
// ============================================================================

/**
 * 构建运动命令帧
 */
inline void build_motion_command(
  MotionCommandFrame & frame,
  const float joint_positions[6])
{
  frame.header = BINARY_HEADER;
  frame.cmd_type = static_cast<uint8_t>(CommandType::MOTION_COMMAND);
  std::memcpy(frame.joint_positions, joint_positions, sizeof(float) * 6);
  frame.footer = BINARY_FOOTER;

  // 计算校验码（不包括 checksum 和 footer）
  frame.checksum = calculate_crc16(
    reinterpret_cast<const uint8_t *>(&frame),
    offsetof(MotionCommandFrame, checksum));
}

/**
 * 验证运动命令帧
 */
inline bool validate_motion_command(const MotionCommandFrame & frame)
{
  if (frame.header != BINARY_HEADER || frame.footer != BINARY_FOOTER) {
    return false;
  }

  if (frame.cmd_type != static_cast<uint8_t>(CommandType::MOTION_COMMAND)) {
    return false;
  }

  return verify_crc16(
    reinterpret_cast<const uint8_t *>(&frame),
    offsetof(MotionCommandFrame, checksum),
    frame.checksum);
}

/**
 * 构建简单命令帧
 */
inline void build_simple_command(
  SimpleCommandFrame & frame,
  CommandType cmd_type,
  uint8_t param = 0)
{
  frame.header = BINARY_HEADER;
  frame.cmd_type = static_cast<uint8_t>(cmd_type);
  frame.param = param;
  frame.footer = BINARY_FOOTER;

  frame.checksum = calculate_crc16(
    reinterpret_cast<const uint8_t *>(&frame),
    offsetof(SimpleCommandFrame, checksum));
}

/**
 * 验证状态响应帧
 */
inline bool validate_state_response(const StateResponseFrame & frame)
{
  if (frame.header != BINARY_HEADER || frame.footer != BINARY_FOOTER) {
    return false;
  }

  if (frame.response_type != static_cast<uint8_t>(ResponseType::STATE_REPLY)) {
    return false;
  }

  return verify_crc16(
    reinterpret_cast<const uint8_t *>(&frame),
    offsetof(StateResponseFrame, checksum),
    frame.checksum);
}

/**
 * 验证简单响应帧
 */
inline bool validate_simple_response(const SimpleResponseFrame & frame)
{
  if (frame.header != BINARY_HEADER || frame.footer != BINARY_FOOTER) {
    return false;
  }

  return verify_crc16(
    reinterpret_cast<const uint8_t *>(&frame),
    offsetof(SimpleResponseFrame, checksum),
    frame.checksum);
}

}  // namespace dummy_hardware

#endif  // DUMMY_HARDWARE__BINARY_PROTOCOL_HPP_
