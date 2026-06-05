/**
 * STM32 固件端二进制协议实现
 *
 * 文件：binary_protocol.c
 */

#include "binary_protocol.h"
#include <string.h>

// ============================================================================
// 全局变量
// ============================================================================

// 接收缓冲区和状态
static uint8_t rx_buffer[128];
static uint16_t rx_index = 0;
static bool rx_in_frame = false;

// 机器人状态（外部定义）
extern float current_joint_positions[6];   // 当前关节位置
extern float current_joint_velocities[6];  // 当前关节速度
extern uint8_t current_gripper_state;      // 夹爪状态

// 函数声明（外部实现）
extern void set_joint_targets(const float positions[6]);
extern void set_gripper_target(uint8_t open);
extern void send_uart_byte(uint8_t byte);
extern void send_uart_bytes(const uint8_t *data, uint16_t length);

// ============================================================================
// CRC16 实现
// ============================================================================

uint16_t calculate_crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;

        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = crc << 1;
            }
        }
    }

    return crc;
}

bool verify_crc16(const uint8_t *data, uint16_t length, uint16_t expected_crc)
{
    return calculate_crc16(data, length) == expected_crc;
}

// ============================================================================
// 协议解析函数
// ============================================================================

ErrorCode parse_motion_command(const MotionCommandFrame *frame, float positions_out[6])
{
    // 1. 验证 header 和 footer
    if (frame->header != BINARY_HEADER || frame->footer != BINARY_FOOTER) {
        return ERR_INVALID_CMD;
    }

    // 2. 验证命令类型
    if (frame->cmd_type != CMD_MOTION_COMMAND) {
        return ERR_INVALID_CMD;
    }

    // 3. 验证 CRC
    uint16_t expected_crc = frame->checksum;
    size_t crc_offset = offsetof(MotionCommandFrame, checksum);
    if (!verify_crc16((const uint8_t*)frame, crc_offset, expected_crc)) {
        return ERR_CHECKSUM;
    }

    // 4. 范围检查（可选）
    for (int i = 0; i < 6; i++) {
        if (frame->joint_positions[i] < -180.0f || frame->joint_positions[i] > 180.0f) {
            return ERR_OUT_OF_RANGE;
        }
    }

    // 5. 复制数据
    memcpy(positions_out, frame->joint_positions, sizeof(float) * 6);

    return ERR_NONE;
}

ErrorCode parse_simple_command(const SimpleCommandFrame *frame, uint8_t *param_out)
{
    // 1. 验证 header 和 footer
    if (frame->header != BINARY_HEADER || frame->footer != BINARY_FOOTER) {
        return ERR_INVALID_CMD;
    }

    // 2. 验证 CRC
    size_t crc_offset = offsetof(SimpleCommandFrame, checksum);
    if (!verify_crc16((const uint8_t*)frame, crc_offset, frame->checksum)) {
        return ERR_CHECKSUM;
    }

    // 3. 返回参数
    *param_out = frame->param;

    return ERR_NONE;
}

// ============================================================================
// 协议构建函数
// ============================================================================

void build_state_response(
    StateResponseFrame *frame,
    const float positions[6],
    const float velocities[6],
    uint8_t gripper_state,
    ErrorCode error_code)
{
    frame->header = BINARY_HEADER;
    frame->response_type = RESP_STATE_REPLY;

    memcpy(frame->joint_positions, positions, sizeof(float) * 6);
    memcpy(frame->joint_velocities, velocities, sizeof(float) * 6);

    frame->gripper_state = gripper_state;
    frame->error_code = error_code;
    frame->footer = BINARY_FOOTER;

    // 计算 CRC
    size_t crc_offset = offsetof(StateResponseFrame, checksum);
    frame->checksum = calculate_crc16((const uint8_t*)frame, crc_offset);
}

void build_simple_response(
    SimpleResponseFrame *frame,
    ResponseType resp_type,
    ErrorCode error_code)
{
    frame->header = BINARY_HEADER;
    frame->response_type = resp_type;
    frame->error_code = error_code;
    frame->footer = BINARY_FOOTER;

    // 计算 CRC
    size_t crc_offset = offsetof(SimpleResponseFrame, checksum);
    frame->checksum = calculate_crc16((const uint8_t*)frame, crc_offset);
}

// ============================================================================
// 命令处理函数
// ============================================================================

static void handle_motion_command(const MotionCommandFrame *frame)
{
    float positions[6];
    ErrorCode err = parse_motion_command(frame, positions);

    if (err == ERR_NONE) {
        // 设置目标位置
        set_joint_targets(positions);

        // 发送 ACK
        SimpleResponseFrame ack;
        build_simple_response(&ack, RESP_ACK, ERR_NONE);
        send_uart_bytes((const uint8_t*)&ack, sizeof(ack));
    } else {
        // 发送错误响应
        SimpleResponseFrame error_resp;
        build_simple_response(&error_resp, RESP_ERROR, err);
        send_uart_bytes((const uint8_t*)&error_resp, sizeof(error_resp));
    }
}

static void handle_query_state(const SimpleCommandFrame *frame)
{
    uint8_t param;
    ErrorCode err = parse_simple_command(frame, &param);

    if (err == ERR_NONE) {
        // 构建并发送状态响应
        StateResponseFrame state;
        build_state_response(
            &state,
            current_joint_positions,
            current_joint_velocities,
            current_gripper_state,
            ERR_NONE
        );
        send_uart_bytes((const uint8_t*)&state, sizeof(state));
    } else {
        // 发送错误响应
        SimpleResponseFrame error_resp;
        build_simple_response(&error_resp, RESP_ERROR, err);
        send_uart_bytes((const uint8_t*)&error_resp, sizeof(error_resp));
    }
}

static void handle_gripper_command(const SimpleCommandFrame *frame)
{
    uint8_t param;
    ErrorCode err = parse_simple_command(frame, &param);

    if (err == ERR_NONE) {
        // 设置夹爪状态（0=关闭，1=打开）
        set_gripper_target(param);

        // 发送 ACK
        SimpleResponseFrame ack;
        build_simple_response(&ack, RESP_ACK, ERR_NONE);
        send_uart_bytes((const uint8_t*)&ack, sizeof(ack));
    } else {
        SimpleResponseFrame error_resp;
        build_simple_response(&error_resp, RESP_ERROR, err);
        send_uart_bytes((const uint8_t*)&error_resp, sizeof(error_resp));
    }
}

static void handle_enable_motors(const SimpleCommandFrame *frame)
{
    // 使能所有电机
    // TODO: 实现你的电机使能逻辑

    SimpleResponseFrame ack;
    build_simple_response(&ack, RESP_ACK, ERR_NONE);
    send_uart_bytes((const uint8_t*)&ack, sizeof(ack));
}

static void handle_disable_motors(const SimpleCommandFrame *frame)
{
    // 失能所有电机
    // TODO: 实现你的电机失能逻辑

    SimpleResponseFrame ack;
    build_simple_response(&ack, RESP_ACK, ERR_NONE);
    send_uart_bytes((const uint8_t*)&ack, sizeof(ack));
}

// ============================================================================
// 接收状态机
// ============================================================================

void process_rx_byte(uint8_t byte)
{
    // 状态机：查找帧头
    if (!rx_in_frame) {
        if (byte == BINARY_HEADER) {
            rx_buffer[0] = byte;
            rx_index = 1;
            rx_in_frame = true;
        }
        return;
    }

    // 接收中：继续填充缓冲区
    if (rx_index < sizeof(rx_buffer)) {
        rx_buffer[rx_index++] = byte;
    } else {
        // 缓冲区溢出，重置
        rx_in_frame = false;
        rx_index = 0;
        return;
    }

    // 检查是否接收到完整帧（至少要有 header + cmd_type + footer）
    if (rx_index < 3) {
        return;
    }

    // 根据命令类型判断帧长度
    uint8_t cmd_type = rx_buffer[1];
    uint16_t expected_size = 0;

    if (cmd_type == CMD_MOTION_COMMAND) {
        expected_size = sizeof(MotionCommandFrame);
    } else if (cmd_type == CMD_QUERY_STATE ||
               cmd_type == CMD_GRIPPER_COMMAND ||
               cmd_type == CMD_ENABLE_MOTORS ||
               cmd_type == CMD_DISABLE_MOTORS) {
        expected_size = sizeof(SimpleCommandFrame);
    } else {
        // 未知命令，重置
        rx_in_frame = false;
        rx_index = 0;
        return;
    }

    // 检查是否接收完整
    if (rx_index >= expected_size) {
        // 验证 footer
        if (rx_buffer[expected_size - 1] == BINARY_FOOTER) {
            // 帧完整，处理命令
            switch (cmd_type) {
                case CMD_MOTION_COMMAND:
                    handle_motion_command((MotionCommandFrame*)rx_buffer);
                    break;

                case CMD_QUERY_STATE:
                    handle_query_state((SimpleCommandFrame*)rx_buffer);
                    break;

                case CMD_GRIPPER_COMMAND:
                    handle_gripper_command((SimpleCommandFrame*)rx_buffer);
                    break;

                case CMD_ENABLE_MOTORS:
                    handle_enable_motors((SimpleCommandFrame*)rx_buffer);
                    break;

                case CMD_DISABLE_MOTORS:
                    handle_disable_motors((SimpleCommandFrame*)rx_buffer);
                    break;
            }
        }

        // 重置状态机
        rx_in_frame = false;
        rx_index = 0;
    }
}

// ============================================================================
// 初始化函数
// ============================================================================

void binary_protocol_init(void)
{
    rx_index = 0;
    rx_in_frame = false;
}

// ============================================================================
// UART 中断处理示例
// ============================================================================

/*
// 在 STM32 的 UART 中断中调用：
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        uint8_t byte = USART_ReceiveData(USART1);
        process_rx_byte(byte);  // 处理接收到的字节
    }
}
*/

// ============================================================================
// DMA 接收示例（推荐，性能更好）
// ============================================================================

/*
// 使用 DMA 循环接收到缓冲区
#define DMA_RX_BUFFER_SIZE 256
static uint8_t dma_rx_buffer[DMA_RX_BUFFER_SIZE];
static uint16_t dma_last_pos = 0;

// 在主循环中调用
void process_dma_rx(void)
{
    // 获取 DMA 当前位置
    uint16_t current_pos = DMA_RX_BUFFER_SIZE - DMA_GetCurrDataCounter(DMA1_Channel5);

    // 处理新接收的字节
    while (dma_last_pos != current_pos) {
        process_rx_byte(dma_rx_buffer[dma_last_pos]);
        dma_last_pos = (dma_last_pos + 1) % DMA_RX_BUFFER_SIZE;
    }
}
*/
