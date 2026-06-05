/**
 * STM32 固件端二进制协议实现
 *
 * 文件：binary_protocol.c / binary_protocol.h
 *
 * 这是参考实现，需要根据你的 STM32 项目结构调整
 */

#ifndef BINARY_PROTOCOL_H
#define BINARY_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// 协议常量（与 ROS 2 端保持一致）
// ============================================================================

#define BINARY_HEADER 0xAA
#define BINARY_FOOTER 0x55

// 命令类型
typedef enum {
    CMD_MOTION_COMMAND = 0x01,
    CMD_GRIPPER_COMMAND = 0x02,
    CMD_QUERY_STATE = 0x03,
    CMD_ENABLE_MOTORS = 0x10,
    CMD_DISABLE_MOTORS = 0x11,
    CMD_HOMING = 0x12,
    CMD_SET_MODE = 0x13
} CommandType;

// 响应类型
typedef enum {
    RESP_STATE_REPLY = 0x81,
    RESP_ACK = 0x82,
    RESP_ERROR = 0x83
} ResponseType;

// 错误码
typedef enum {
    ERR_NONE = 0x00,
    ERR_CHECKSUM = 0x01,
    ERR_INVALID_CMD = 0x02,
    ERR_OUT_OF_RANGE = 0x03,
    ERR_MOTOR_ERROR = 0x04,
    ERR_TIMEOUT = 0x05
} ErrorCode;

// ============================================================================
// 协议帧结构（与 ROS 2 端保持一致）
// ============================================================================

// 运动命令帧（30字节）
typedef struct __attribute__((packed)) {
    uint8_t header;              // 0xAA
    uint8_t cmd_type;            // CMD_MOTION_COMMAND
    float joint_positions[6];    // 6个关节位置（度）
    uint16_t checksum;           // CRC16
    uint8_t footer;              // 0x55
} MotionCommandFrame;

// 状态响应帧（56字节）
typedef struct __attribute__((packed)) {
    uint8_t header;              // 0xAA
    uint8_t response_type;       // RESP_STATE_REPLY
    float joint_positions[6];    // 6个关节位置（度）
    float joint_velocities[6];   // 6个关节速度（度/秒）
    uint8_t gripper_state;       // 0=关闭，1=打开
    uint8_t error_code;          // 错误码
    uint16_t checksum;           // CRC16
    uint8_t footer;              // 0x55
} StateResponseFrame;

// 简单命令帧（6字节）
typedef struct __attribute__((packed)) {
    uint8_t header;
    uint8_t cmd_type;
    uint8_t param;
    uint16_t checksum;
    uint8_t footer;
} SimpleCommandFrame;

// 简单响应帧（6字节）
typedef struct __attribute__((packed)) {
    uint8_t header;
    uint8_t response_type;
    uint8_t error_code;
    uint16_t checksum;
    uint8_t footer;
} SimpleResponseFrame;

// ============================================================================
// CRC16 校验函数
// ============================================================================

/**
 * CRC16-CCITT 校验
 */
uint16_t calculate_crc16(const uint8_t *data, uint16_t length);

/**
 * 验证 CRC16
 */
bool verify_crc16(const uint8_t *data, uint16_t length, uint16_t expected_crc);

// ============================================================================
// 协议处理函数
// ============================================================================

/**
 * 解析运动命令帧
 * @return ErrorCode
 */
ErrorCode parse_motion_command(const MotionCommandFrame *frame, float positions_out[6]);

/**
 * 构建状态响应帧
 */
void build_state_response(
    StateResponseFrame *frame,
    const float positions[6],
    const float velocities[6],
    uint8_t gripper_state,
    ErrorCode error_code
);

/**
 * 解析简单命令帧
 */
ErrorCode parse_simple_command(const SimpleCommandFrame *frame, uint8_t *param_out);

/**
 * 构建简单响应帧
 */
void build_simple_response(
    SimpleResponseFrame *frame,
    ResponseType resp_type,
    ErrorCode error_code
);

/**
 * 串口接收状态机（在中断或主循环中调用）
 */
void process_rx_byte(uint8_t byte);

#endif // BINARY_PROTOCOL_H
