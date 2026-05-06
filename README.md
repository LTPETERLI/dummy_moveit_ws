# Dummy 六轴机械臂 ROS 2 控制与视觉抓取工作空间
<img width="963" height="1494" alt="6b1c9630fee3a6e8a61224c6f77ec10c" src="https://github.com/user-attachments/assets/4b137102-b662-40bf-b1ca-d8f514a05403" />

基于开源 [Dummy-Robot](https://github.com/peng-zhihui/Dummy-Robot) 六轴机械臂搭建的 **ROS 2 Humble + MoveIt 2 + ros2_control** 完整控制栈，含 URDF 建模、真机硬件接口、运动规划、夹爪控制与 D435i 视觉感知，可在 RViz 仿真与真机上完成桌面物体抓取-放置任务。

## 工作空间结构

```
.
├── dummy-ros2_description/      # URDF/Xacro 模型 + meshes（fusion360 导出）
├── dummy_moveit_config/         # MoveIt 2 配置（SRDF / 规划组 / controllers / launch）
├── dummy_hardware/              # ros2_control 真机硬件插件（C++17）
├── dummy_controller/            # 上游 Python 真机桥（参考用，不在 ROS 链路上）
├── dummy_server/                # pymoveit2 Python 调用层
├── dummy_vision/                # D435i 手眼标定与视觉抓取
├── 复现路线.md                  # 阶段性复现笔记
└── Dummy机械臂硬件测试操作文档.md  # 真机测试与故障排查 SOP
```

## 关键技术栈

- **ROS 2 Humble** + **MoveIt 2**（`move_group`、`Setup Assistant`、`MoveIt Task Constructor`）
- **ros2_control**（`hardware_interface::SystemInterface`、`JointTrajectoryController`）
- **URDF / Xacro**（fusion360 → URDF 链路）
- **RealSense ROS** + D435i RGB-D 视觉
- **STM32F4 主控** + USB CDC ASCII 协议
- C++17、Python 3、CMake、`colcon`

## 主要工作

1. **真机硬件接口**：自研 [`dummy_hardware`](dummy_hardware/) 插件，通过 USB CDC 串口与 STM32F4 主控双向通信（`!START` / `&j1..j6` / `#GETJPOS` / `!HAND_O,C`）；解决 CMDMODE 2 下 `&` 命令"双回复"时序错位与 16-slot FIFO 拥塞，将控制频率调优到 30 Hz。

2. **关节标定与映射**：实测 6 个关节的 firmware↔ROS 方向（sign）与零点偏移（offset），作为 xacro 参数透传到硬件插件，让 RViz 模型与真机姿态对齐而无需重编 C++。

3. **URDF 重构**：补齐 mesh material 颜色定义、修正 Joint2/Joint3 限位与 firmware 实际范围匹配、补全夹爪 `gripper_carrier` + `finger_left/right`（Joint7 prismatic + mimic）。

4. **MoveIt 配置**：用 Setup Assistant 通过 10000 次随机采样自动生成 self-collision matrix；配置 `dummy_arm`（J1-J6 JTC）与 `dummy_gripper`（二态 `!HAND_O/C`）两个 planning group。

5. **视觉感知**：D435i 手眼标定与目标三维定位（参考 [`dummy_vision/`](dummy_vision/)）。

## 快速启动

### 依赖

- Ubuntu 22.04 + ROS 2 Humble
- 额外包：`ros-humble-moveit`、`ros-humble-ros2-control`、`ros-humble-joint-state-publisher-gui`
- 视觉部分需 `realsense-ros`（独立 clone 到 `src/`，见下）

### 1. clone 工作空间

```bash
mkdir -p ~/dummy_ws/src && cd ~/dummy_ws/src
git clone https://github.com/LTPETERLI/<REPO_NAME>.git .
# 视觉部分（独立仓库）：
# git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-master
```

### 2. 编译

```bash
cd ~/dummy_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. 仿真（fake hardware）

```bash
ros2 launch dummy_moveit_config demo.launch.py
```

RViz 打开后，MotionPlanning 面板选择 `dummy_arm` 规划组，可以拖动末端目标做规划与执行。

### 4. 真机控制

> ⚠️ **真机操作前请先看 [Dummy机械臂硬件测试操作文档.md](Dummy机械臂硬件测试操作文档.md)** 了解上电、串口校验、紧急停止等关键流程。

```bash
ros2 launch dummy_moveit_config demo_custom.launch.py serial_port:=/dev/ttyACM0
```

## 上游致谢

本仓库基于以下开源项目衍生 / 二次开发：

- [Dummy-Robot](https://github.com/peng-zhihui/Dummy-Robot) — 机械臂硬件设计 (peng-zhihui)
- [木子改良版 dummy](https://gitee.com/switchpi/dummy) — 固件与运动学
- [pymoveit2](https://github.com/AndrejOrsula/pymoveit2) — Python MoveIt 2 调用层
- [fusion2urdf](https://github.com/syuntoku14/fusion2urdf) — fusion360 → URDF 工具
- [hata8210/dummy_moveit_ws](https://github.com/hata8210/dummy_moveit_ws)（如适用）— MoveIt 2 配置参考

## License

本仓库遵循上游 [BSD 3-Clause License](LICENSE)，仅用于学习交流，不用于商业用途。
