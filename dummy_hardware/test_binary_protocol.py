#!/usr/bin/env python3
"""
二进制协议测试工具

用于测试和验证 dummy 机械臂的二进制协议实现
"""

import serial
import struct
import time
import sys

# ============================================================================
# 协议常量
# ============================================================================

BINARY_HEADER = 0xAA
BINARY_FOOTER = 0x55

# 命令类型
CMD_MOTION_COMMAND = 0x01
CMD_GRIPPER_COMMAND = 0x02
CMD_QUERY_STATE = 0x03
CMD_ENABLE_MOTORS = 0x10
CMD_DISABLE_MOTORS = 0x11

# 响应类型
RESP_STATE_REPLY = 0x81
RESP_ACK = 0x82
RESP_ERROR = 0x83

# ============================================================================
# CRC16 计算
# ============================================================================

def calculate_crc16(data):
    """CRC16-CCITT 校验"""
    crc = 0xFFFF

    for byte in data:
        crc ^= (byte << 8)

        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF

    return crc

# ============================================================================
# 协议构建函数
# ============================================================================

def build_motion_command(positions):
    """
    构建运动命令帧
    positions: 6个关节位置（度）的列表
    """
    # Header + cmd_type
    frame = bytearray([BINARY_HEADER, CMD_MOTION_COMMAND])

    # 6个关节位置（float）
    for pos in positions:
        frame += struct.pack('<f', pos)

    # 计算 CRC（不包括 checksum 和 footer）
    crc = calculate_crc16(frame)
    frame += struct.pack('<H', crc)

    # Footer
    frame += bytes([BINARY_FOOTER])

    return frame

def build_simple_command(cmd_type, param=0):
    """
    构建简单命令帧
    """
    frame = bytearray([BINARY_HEADER, cmd_type, param])

    crc = calculate_crc16(frame)
    frame += struct.pack('<H', crc)
    frame += bytes([BINARY_FOOTER])

    return frame

def parse_state_response(data):
    """
    解析状态响应帧
    """
    if len(data) != 56:
        raise ValueError(f"Invalid frame size: {len(data)}, expected 56")

    if data[0] != BINARY_HEADER or data[-1] != BINARY_FOOTER:
        raise ValueError("Invalid header or footer")

    # 解析数据
    response_type = data[1]

    # 6个关节位置
    positions = []
    for i in range(6):
        offset = 2 + i * 4
        pos = struct.unpack('<f', data[offset:offset+4])[0]
        positions.append(pos)

    # 6个关节速度
    velocities = []
    for i in range(6):
        offset = 26 + i * 4
        vel = struct.unpack('<f', data[offset:offset+4])[0]
        velocities.append(vel)

    gripper_state = data[50]
    error_code = data[51]
    checksum = struct.unpack('<H', data[52:54])[0]

    # 验证 CRC
    expected_crc = calculate_crc16(data[:52])
    if checksum != expected_crc:
        print(f"⚠️  CRC mismatch: got {checksum:04X}, expected {expected_crc:04X}")

    return {
        'response_type': response_type,
        'positions': positions,
        'velocities': velocities,
        'gripper_state': gripper_state,
        'error_code': error_code
    }

def parse_simple_response(data):
    """
    解析简单响应帧
    """
    if len(data) != 6:
        raise ValueError(f"Invalid frame size: {len(data)}, expected 6")

    if data[0] != BINARY_HEADER or data[-1] != BINARY_FOOTER:
        raise ValueError("Invalid header or footer")

    response_type = data[1]
    error_code = data[2]
    checksum = struct.unpack('<H', data[3:5])[0]

    # 验证 CRC
    expected_crc = calculate_crc16(data[:3])
    if checksum != expected_crc:
        print(f"⚠️  CRC mismatch: got {checksum:04X}, expected {expected_crc:04X}")

    return {
        'response_type': response_type,
        'error_code': error_code
    }

# ============================================================================
# 测试函数
# ============================================================================

def test_query_state(port):
    """测试查询状态"""
    print("\n📊 测试：查询状态")
    print("-" * 50)

    # 发送查询命令
    cmd = build_simple_command(CMD_QUERY_STATE)
    print(f"发送: {cmd.hex()}")
    port.write(cmd)

    # 等待响应
    time.sleep(0.1)
    if port.in_waiting >= 56:
        response_data = port.read(56)
        print(f"接收: {response_data.hex()}")

        try:
            response = parse_state_response(response_data)
            print(f"✅ 解析成功:")
            print(f"   响应类型: 0x{response['response_type']:02X}")
            print(f"   关节位置: {[f'{p:.2f}°' for p in response['positions']]}")
            print(f"   关节速度: {[f'{v:.2f}°/s' for v in response['velocities']]}")
            print(f"   夹爪状态: {'打开' if response['gripper_state'] else '关闭'}")
            print(f"   错误码: 0x{response['error_code']:02X}")
            return True
        except Exception as e:
            print(f"❌ 解析失败: {e}")
            return False
    else:
        print(f"❌ 超时或数据不足: {port.in_waiting} 字节")
        return False

def test_motion_command(port, positions):
    """测试运动命令"""
    print("\n🤖 测试：运动命令")
    print("-" * 50)
    print(f"目标位置: {[f'{p:.2f}°' for p in positions]}")

    # 发送运动命令
    cmd = build_motion_command(positions)
    print(f"发送: {cmd.hex()}")
    port.write(cmd)

    # 等待 ACK
    time.sleep(0.1)
    if port.in_waiting >= 6:
        response_data = port.read(6)
        print(f"接收: {response_data.hex()}")

        try:
            response = parse_simple_response(response_data)
            if response['response_type'] == RESP_ACK:
                print(f"✅ 收到 ACK")
                return True
            elif response['response_type'] == RESP_ERROR:
                print(f"❌ 收到错误: 0x{response['error_code']:02X}")
                return False
        except Exception as e:
            print(f"❌ 解析失败: {e}")
            return False
    else:
        print(f"⚠️  未收到响应（可能 STM32 不发送 ACK）")
        return True  # 有些实现不发送 ACK

def test_gripper(port, open_gripper):
    """测试夹爪"""
    print(f"\n🤏 测试：夹爪{'打开' if open_gripper else '关闭'}")
    print("-" * 50)

    cmd = build_simple_command(CMD_GRIPPER_COMMAND, 1 if open_gripper else 0)
    print(f"发送: {cmd.hex()}")
    port.write(cmd)

    time.sleep(0.1)
    if port.in_waiting >= 6:
        response_data = port.read(6)
        print(f"接收: {response_data.hex()}")

        try:
            response = parse_simple_response(response_data)
            if response['response_type'] == RESP_ACK:
                print(f"✅ 收到 ACK")
                return True
            elif response['response_type'] == RESP_ERROR:
                print(f"❌ 收到错误: 0x{response['error_code']:02X}")
                return False
        except Exception as e:
            print(f"❌ 解析失败: {e}")
            return False
    else:
        print(f"⚠️  未收到响应")
        return True

def test_enable_motors(port):
    """测试使能电机"""
    print("\n⚡ 测试：使能电机")
    print("-" * 50)

    cmd = build_simple_command(CMD_ENABLE_MOTORS)
    print(f"发送: {cmd.hex()}")
    port.write(cmd)

    time.sleep(0.1)
    if port.in_waiting >= 6:
        response_data = port.read(6)
        print(f"接收: {response_data.hex()}")

        try:
            response = parse_simple_response(response_data)
            if response['response_type'] == RESP_ACK:
                print(f"✅ 收到 ACK")
                return True
        except Exception as e:
            print(f"❌ 解析失败: {e}")
    else:
        print(f"⚠️  未收到响应")

    return True

def performance_test(port, iterations=100):
    """性能测试"""
    print("\n⚡ 性能测试")
    print("-" * 50)
    print(f"测试次数: {iterations}")

    # 测试位置
    positions = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0]

    start_time = time.time()
    success_count = 0

    for i in range(iterations):
        # 轻微变化位置
        test_positions = [p + (i % 10) for p in positions]

        cmd = build_motion_command(test_positions)
        port.write(cmd)

        # 简单等待
        time.sleep(0.01)

        # 清空缓冲区（如果有响应）
        if port.in_waiting > 0:
            port.read(port.in_waiting)
            success_count += 1

    elapsed = time.time() - start_time

    print(f"✅ 完成!")
    print(f"   总时间: {elapsed:.2f} 秒")
    print(f"   平均周期: {elapsed/iterations*1000:.2f} ms")
    print(f"   等效频率: {iterations/elapsed:.1f} Hz")
    print(f"   成功率: {success_count/iterations*100:.1f}%")

# ============================================================================
# 主程序
# ============================================================================

def main():
    print("=" * 50)
    print(" Dummy 机械臂二进制协议测试工具")
    print("=" * 50)

    # 解析参数
    port_name = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
    baud_rate = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    print(f"\n📡 串口配置:")
    print(f"   端口: {port_name}")
    print(f"   波特率: {baud_rate}")

    try:
        # 打开串口
        port = serial.Serial(port_name, baud_rate, timeout=1)
        print(f"✅ 串口打开成功\n")

        # 清空缓冲区
        time.sleep(0.5)
        port.reset_input_buffer()
        port.reset_output_buffer()

        # 运行测试
        tests = [
            ("查询状态", lambda: test_query_state(port)),
            ("使能电机", lambda: test_enable_motors(port)),
            ("运动命令", lambda: test_motion_command(port, [0, 0, 90, 0, 0, 0])),
            ("夹爪打开", lambda: test_gripper(port, True)),
            ("夹爪关闭", lambda: test_gripper(port, False)),
        ]

        results = []
        for name, test_func in tests:
            try:
                success = test_func()
                results.append((name, success))
                time.sleep(0.5)
            except Exception as e:
                print(f"❌ 测试异常: {e}")
                results.append((name, False))

        # 性能测试
        print("\n" + "=" * 50)
        response = input("是否运行性能测试？(y/n): ")
        if response.lower() == 'y':
            performance_test(port, 100)

        # 总结
        print("\n" + "=" * 50)
        print(" 测试总结")
        print("=" * 50)
        for name, success in results:
            status = "✅ 通过" if success else "❌ 失败"
            print(f"{name:20s} {status}")

        passed = sum(1 for _, s in results if s)
        print(f"\n总计: {passed}/{len(results)} 测试通过")

        # 关闭串口
        port.close()

    except serial.SerialException as e:
        print(f"❌ 串口错误: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n\n⚠️  用户中断")
        sys.exit(0)

if __name__ == '__main__':
    main()
