# Dummy 机械臂硬件测试操作文档

> 用于在 Linux 下通过 USB-C 直连方式测试 Dummy 6-DOF 机械臂 + 夹爪。
> 适用固件：`Core-STM32F4-fw_RTX_30_20260119`（任同学结构件，J6 减速比 30，含夹爪代码）。

---

## 1. 测试前安全检查

每次开始前必须确认：

- [ ] 机械臂底座**用螺丝/胶带固定在桌面**（运动时反作用力会推翻松动的机械臂）
- [ ] 机械臂前方**至少留 50cm 空旷**（没有易碎物、没有人手）
- [ ] 12V 稳压电源已准备（**不要用 24V**，PDF 第 2 页警告 24V 接错芯片必烧）
- [ ] 桌面平整，机械臂落地不会摔
- [ ] **物理摆放折叠位**：把机械臂手动摆到 **7 字折叠状态**，对准底座刻度。**这一步在每次上电前都要做**——上电瞬间的位置就是固件零点

---

## 2. 上电与 USB 连接

### 2.1 上电

1. 接 12V 稳压电源到 REF 主板（**红正黑负**，DC5.5×2.5 接头）
2. **不**先插 USB-C
3. 等 2-3 秒看主板 OLED 屏：
   - ✓ OLED 亮 + 显示 6 个就绪驱动板 → 硬件正常
   - ⚠️ OLED 亮但少于 6 个就绪 → 缺失的轴需要校准（看第 9 节）
   - ❌ OLED 不亮 → 检查 12V 接线 / 主板有问题

### 2.2 USB-C 连接

1. 插上 USB-C 数据线到电脑
2. 在终端运行：
   ```bash
   ls /dev/ttyACM* 2>&1
   ```
3. 期望看到 `/dev/ttyACM0`
4. **如果没看到 ttyACM**：拔掉 type-C，**翻 180° 再插回去**（type-C 正反方向是不同芯片，反向时枚举为 CP2102 ttyUSB 但走 UART4 通道命令子集少）
5. 重新执行 `ls`，确认 `/dev/ttyACM0` 出现

### 2.3 进入串口终端

```bash
picocom -b 115200 --omap crcrlf --echo /dev/ttyACM0
```

参数说明：
- `--omap crcrlf` → 按回车自动发 `\r\n`（固件要求）
- `--echo` → 回显输入字符

退出方式：按 **`Ctrl-A`** 松开后按 **`Ctrl-X`**

---

## 3. 命令速查表

### 3.1 状态读取（不改变机械臂状态，安全）

| 命令 | 含义 | 期望回复 |
|---|---|---|
| `#GETJPOS` | 读取 6 个关节角度（度） | `ok j1 j2 j3 j4 j5 j6` |
| `#GETLPOS` | 读取末端笛卡尔位姿 | `ok x y z a b c` |

### 3.2 全机控制

| 命令 | 含义 | 期望回复 |
|---|---|---|
| `!START` | 6 关节使能（电机锁定保持位置） | `Started ok` |
| `!STOP` | 紧急停止（清空命令队列、立刻停） | `Stopped ok` |
| `!HOME` | 自动展开到 L-pose `(0, 0, 90, 0, 0, 0)` | `Started ok` |
| `!RESET` | 自动收回到 REST_POSE `(0, -75, 180, 0, 0, 0)` | `Started ok` |
| `!DISABLE` | 6 关节失能（电机断电、可手扶） | `Disabled ok` |
| `!CALIBRATION` | 重新校准零点（**用前必须看第 9 节**） | `calibration ok` |

### 3.3 命令模式

| 命令 | 含义 |
|---|---|
| `#CMDMODE 1` | 顺序模式：每条 `&` 命令等运动完成才回 ok（**测试推荐**） |
| `#CMDMODE 2` | 可中断模式：命令立即回 ok，下一条可打断（**ROS 工作模式**） |

### 3.4 关节运动

| 命令 | 含义 |
|---|---|
| `&j1,j2,j3,j4,j5,j6` | 关节空间运动到目标角度（度） |
| `&j1,j2,j3,j4,j5,j6,speed` | 同上 + 指定速度 |
| `@x,y,z,a,b,c` | 笛卡尔空间运动（mm + 度，需要逆解） |

### 3.5 夹爪控制（独立通道，命令不能与 `&` 共用）

| 命令 | 含义 | 期望回复 |
|---|---|---|
| `!HAND_EN` | 夹爪电机使能 | `ok hand enable ... real_angle:XX` |
| `!HAND_DIS` | 夹爪电机失能 | `ok hand disable ... real_angle:XX` |
| `!HAND_O` | 夹爪张开（电流模式，最大开口） | `ok hand open` |
| `!HAND_C` | 夹爪合拢（电流模式，最大闭合） | `ok hand close` |
| `!HAND_POS N` | 夹爪到位置 N（**当前固件 bug：实际不工作**，N 1~100） | `ok hand position N` |
| `!HAND_I N` | 夹爪电流上限 N（0~2.0A） | `ok hand current N` |
| `!HAND_ZERO` | 夹爪自动校准开口端点 | `ok hand offset MIN:XX max:XX` |

### 3.6 单关节调参（节点 1~6，**不接受 0=ALL**）

| 命令 | 含义 |
|---|---|
| `#I_LIMIT_J <node> <I>` | 设节点 node 电流上限（A） |
| `#SPEED_J <node> <S>` | 设节点 node 速度上限（度/秒） |
| `#ACC_J <node> <S>` | 设节点 node 加速度（0~100） |
| `#OFFSET_J <node>` | 把节点 node 当前位置设为 home offset |
| `#REBOOT <node>` | 重启节点 node 驱动板 |
| `#SET_DCE_KP <node> <KP>` | 调 PID Kp |
| `#SET_DCE_KI/KD/KV` | 调 PID Ki/Kd/Kv |

---

## 4. 标准测试流程

### 4.1 基础链路验证（5 分钟）

进 picocom 后逐条敲（每条按回车）：

```
#GETJPOS
```
期望接近 REST_POSE `ok ~0 ~-75 ~180 ~0 ~0 ~0`（折叠位摆放正确的话）

```
!START
```
期望 `Started ok`，电机会发出"咔咔"声然后锁定

```
#CMDMODE 1
```
期望 `ok Set command mode to [1]`

```
!HOME
```
期望 `Started ok`，机械臂展开 5-10 秒到 L-pose（**手放在 `!STOP` 准备紧急停止**）

```
#GETJPOS
```
期望 `ok 0.00 0.00 90.00 0.00 0.00 0.00`

### 4.2 6 轴单关节方向测试

**前置条件**：已完成 4.1，机械臂在 L-pose `(0,0,90,0,0,0)`，CMDMODE 1。

每个轴测 +20° 和 -20°，每条命令后等 5 秒再读 #GETJPOS：

#### J1（基座旋转）
```
&20,0,90,0,0,0
#GETJPOS
&-20,0,90,0,0,0
#GETJPOS
&0,0,90,0,0,0
```

#### J2（大臂俯仰）
```
&0,20,90,0,0,0
#GETJPOS
&0,-20,90,0,0,0
#GETJPOS
&0,0,90,0,0,0
```

#### J3（小臂俯仰，限位 35~180，从 90 出发只能 90±55）
```
&0,0,110,0,0,0
#GETJPOS
&0,0,70,0,0,0
#GETJPOS
&0,0,90,0,0,0
```

#### J4（小臂自旋）
```
&0,0,90,20,0,0
#GETJPOS
&0,0,90,-20,0,0
#GETJPOS
&0,0,90,0,0,0
```

#### J5（腕俯仰）
```
&0,0,90,0,20,0
#GETJPOS
&0,0,90,0,-20,0
#GETJPOS
&0,0,90,0,0,0
```

#### J6（末端自旋）
```
&0,0,90,0,0,20
#GETJPOS
&0,0,90,0,0,-20
#GETJPOS
&0,0,90,0,0,0
```

**注意事项**：
- 不要把 `&` 命令和 `#GETJPOS` 紧挨着发，**中间换行 + 等 5 秒**，否则 #GETJPOS 在 & 还没执行完就读了状态，数据不准
- CMDMODE 1 下 `&` 命令会等到位才回 ok，如果命令一直没回 ok，说明机械臂还在运动或卡住

### 4.3 夹爪测试

```
!HAND_EN
!HAND_O
```
等 5 秒，夹爪应张开到最大（电流模式自动停在端点）

```
!HAND_C
```
等 5 秒，夹爪应合拢到最小

```
!HAND_DIS
```

`!HAND_POS N` 在当前固件下不工作（验证过 1/50/100 都映射到同一位置），ROS 接入时使用二态控制（`!HAND_O` / `!HAND_C`）。

### 4.4 收尾

```
!RESET
```
回到折叠位

```
!DISABLE
```
失能

按 `Ctrl-A` 松手后按 `Ctrl-X` 退出 picocom。

---

## 5. 已知标定数据（已验证，2026-05-06）

### 5.1 关键位置

| 位置 | firmware 角度（°） | 来源 |
|---|---|---|
| 上电折叠位 | (~0, ~-77, ~180, ~0, ~0, ~0) | 手动摆放，约等于 REST_POSE |
| `!HOME` 后 L-pose | (0.00, 0.00, 90.00, 0.00, 0.00, 0.00) | 干净零点 |
| `!RESET` 后 REST_POSE | (0.00, -75.00, 180.00, 0.00, 0.00, 0.00) | 标称折叠位 |

### 5.2 关节 +20° 方向（用户站机械臂正后方观察）

| 关节 | firmware +20° → 物理方向 |
|---|---|
| J1 | 大臂俯视 CCW（左转） |
| J2 | 大臂前倾（朝远离用户） |
| J3 | 小臂缩拢（朝靠近用户） |
| J4 | 小臂沿轴 CW（朝末端方向看） |
| J5 | 末端朝右偏 |
| J6 | 末端法兰 CW（朝末端方向看） |

### 5.3 ROS 关节映射（先用上游假设，待 RViz 验证）

```
joint_offset_deg = [0, 0, 90, 0, 0, 0]   # ROS J3 = firmware J3 - 90
joint_sign       = [1, 1, 1, 1, -1, -1]  # J5/J6 反向
```

公式：
- 读：`ros_rad = deg_to_rad((firmware_deg - offset_deg) * sign)`
- 写：`firmware_deg = rad_to_deg(ros_rad) * sign + offset_deg`

### 5.4 已识别的硬件特征

- **J1 物理结构限位**：J1 单边方向只能转到约 ±5°（机械结构/线缆限制），ROS 端 URDF Joint1 limit 后续应收紧
- **L-pose 几度公差**：固件零点 (0,0,90,0,0,0) 跟"完美 7 字"垂直度差几度（装配公差），不影响功能但可用 `!CALIBRATION` 校准（看第 9 节）
- **type-C 方向敏感**：正方向 → STM32 USB CDC（ttyACM0，命令全集），反方向 → CP2102（ttyUSB0，命令子集少不能控夹爪）

---

## 6. 串口协议关键细节

| 项 | 值 |
|---|---|
| 波特率 | 115200 8N1 |
| 行尾 | `\r\n`（必须，固件不接受单 `\n`） |
| 命令必须**英文大写** | `!START` 行，`!start` 不行 |
| `!` 命令在任何时候都可发 | 包括未使能时 |
| `#` 命令需要电机使能（UART4 通道） | USB CDC 通道无此限制 |
| `&` 关节命令推荐用 `&` 不用 `>` | 官方手册推荐，行为相同 |
| CMDMODE 1 ok 表示运动完成 | CMDMODE 2 ok 表示命令入队 |

---

## 7. 紧急停止与异常处理

### 7.1 立刻停机的方法（按优先级）

1. **`!STOP`**（picocom 里敲）→ 清空命令队列 + 立即停止当前运动
2. **拔 12V 电源**（终极手段）→ 机械臂瞬间断电，可能因重力下垂

### 7.2 常见故障

| 现象 | 可能原因 | 处理 |
|---|---|---|
| `/dev/ttyACM0` 不出现 | type-C 反向 | 翻 180° 再插 |
| `#GETJPOS` 不回复（USB 通道） | 应该有回复，重启固件试试 | 拔插 USB / 重启电源 |
| `&` 命令回 ok 但机械臂没动 | CMDMODE 1 下电机失能 | 先发 `!START` |
| 命令回 ok 但 #GETJPOS 显示没到位 | 时序问题，#GETJPOS 在命令完成前读了 | 等 5 秒再读 |
| 某轴 `#GETJPOS` 数字异常（飘几十度） | 该轴电机校准丢了 | 单独校准（第 9 节）|
| `!HAND_POS N` 不动 | 当前固件 bug | 改用 `!HAND_O` / `!HAND_C` |
| OLED 不亮 | 12V 没上 / CAN 总线没就绪 | 检查 12V + 驱动板供电 |

---

## 8. ROS 端集成（参考，工作完成后填）

启动命令：
```bash
ros2 launch dummy_moveit_config demo_custom.launch.py serial_port:=/dev/ttyACM0
```

启动前操作：
- picocom 单独发 `!HOME` 让机械臂去 L-pose（避免 ROS 启动时读到 J3 超 URDF 限位）
- 关 picocom（释放串口）
- 再启动 ROS

验证：
- `ros2 control list_controllers` → arm/gripper/state_broadcaster 全 active
- `ros2 topic echo /joint_states --once` → Joint1-6 ≈ 0、Joint7 = 0
- RViz MotionPlanning 拖滑块 → 真机响应

---

## 9. 校准流程（可选，零位偏差大时再做）

### 9.1 整机校准（!CALIBRATION）

⚠️ **必须在精确 L-pose 状态下发命令，否则会让机械臂朝错误方向冲撞**。

步骤：
1. 进 picocom，发 `!DISABLE` 失能
2. **手动**把机械臂掰到精确 L-pose：
   - 大臂垂直向上
   - 小臂水平向前（远离基座方向）
   - 对准底座刻度
3. 一只手扶住 L-pose 不动，另一只手在键盘敲：
   ```
   !CALIBRATION
   ```
4. 固件会自动：
   - 把 J2/J3 电流降到 0.5A 让你能继续手扶
   - 应用当前位置为 home（第一次）
   - 自动收回到 REST_POSE
   - 应用 home（第二次）
   - **重启**主板
5. 重启后机械臂的 home 偏置已更新，需要重新走一遍 `picocom 连接 → !START → ...`

### 9.2 单关节电机校准（OLED 缺轴时做）

按 PDF 第 13 页：

⚠️ **必须单独通电该驱动板**，不要联机校准。

1. 把待校准的驱动板**单独**通 12V 电（不接其他驱动板）
2. 等 LED1 常亮、LED2 闪烁
3. 按住该驱动板上的 **K1 键三秒后松开**
4. 电机会**正转两圈反转一圈**
5. 完成后 LED1、LED2 各闪烁一下
6. 校准数据存在芯片 EEPROM，断电不丢

---

## 10. 命令完整列表（USB CDC 通道，OnUsbAsciiCmd）

### `!` 命令（任何时候可发）

```
!START          - 6 关节使能
!STOP           - 紧急停止
!HOME           - 展开到 L-pose (0,0,90,0,0,0)
!RESET          - 收回到 REST_POSE (0,-75,180,0,0,0)
!CALIBRATION    - 整机校准（必须先手动到 L-pose）
!DISABLE        - 6 关节失能
!HAND_EN        - 夹爪使能
!HAND_DIS       - 夹爪失能
!HAND_O         - 夹爪张开（电流模式）
!HAND_C         - 夹爪合拢（电流模式）
!HAND_POS N     - 夹爪位置 N (1~100，当前固件 bug)
!HAND_I N       - 夹爪电流上限 N (0~2.0A)
!HAND_ZERO      - 夹爪自动校准端点
```

### `#` 命令（查询和调参）

```
#GETJPOS                    - 读 6 关节角度
#GETLPOS                    - 读末端笛卡尔位姿
#CMDMODE M                  - 切命令模式 (1=顺序, 2=可中断)
#REBOOT N                   - 重启关节 N (1~6)
#OFFSET_J N                 - 关节 N 当前位置设为 home offset
#ACC_J N S                  - 关节 N 加速度 S (0~100)
#SPEED_J N S                - 关节 N 速度 S (度/秒)
#I_LIMIT_J N I              - 关节 N 电流上限 I (A)
#SET_DCE_KP N KP            - 关节 N PID Kp
#SET_DCE_KI N KI            - 关节 N PID Ki
#SET_DCE_KD N KD            - 关节 N PID Kd
#SET_DCE_KV N KV            - 关节 N PID Kv
```

### `&` / `@` / `>` 命令（运动控制）

```
&j1,j2,j3,j4,j5,j6           - 关节空间运动（度）
&j1,j2,j3,j4,j5,j6,speed     - 同上 + 速度
@x,y,z,a,b,c                 - 笛卡尔空间运动（mm,度）
@x,y,z,a,b,c,speed           - 同上 + 速度
>j1,j2,j3,j4,j5,j6           - 等价于 & 的旧版命令
```

---

## 11. 维护与变更记录

### 2026-05-06：首次硬件验证完成
- USB CDC 链路验证 ✓
- 6 轴 `!HOME` / `!RESET` 多关节运动学 ✓
- 6 轴单关节方向（每轴 ±20°）✓
- 夹爪 `!HAND_O` / `!HAND_C` 二态控制 ✓
- 夹爪 `!HAND_POS` 不可用（实测 1/50/100 都不动）⚠️
- J1 物理限位（单边 ~5°）已识别 ⚠️
- L-pose 几度公差已识别（不影响功能，可后续 `!CALIBRATION` 校准）

### 后续待补
- ROS Phase D（RViz 拖滑块）逐轴 sign 最终确认
- URDF Joint1/Joint2/Joint3 limit 与实际范围对齐
- 整机精确 `!CALIBRATION`
- 夹爪 `!HAND_POS` 修复（修固件加 `SetPercentToAngle` 入口）
