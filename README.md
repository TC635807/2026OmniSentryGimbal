<div align="center">

# 2026OmniSentryGimbal

**RoboMaster 哨兵机器人 · 云台板主控固件**

STM32F407IGHx · FreeRTOS · CMake + Ninja · yaw/pitch 双轴闭环 · 与视觉上位机串口协议

![MCU](https://img.shields.io/badge/MCU-STM32F407IGHx-03234B?style=flat-square&logo=stmicroelectronics&logoColor=white)
![RTOS](https://img.shields.io/badge/RTOS-FreeRTOS-1E8449?style=flat-square)
![Build](https://img.shields.io/badge/build-CMake%20%2B%20Ninja-064F8C?style=flat-square&logo=cmake&logoColor=white)
![Toolchain](https://img.shields.io/badge/toolchain-arm--none--eabi--gcc-A42E2B?style=flat-square)
![Language](https://img.shields.io/badge/language-C%2FC%2B%2B-00599C?style=flat-square&logo=cplusplus&logoColor=white)

配套仓库：[底盘板固件 · 2026OmniSentryChassis](https://github.com/TC635807/2026OmniSentryChassis)

</div>

---

## 这是什么

2026 赛季 RoboMaster 哨兵机器人的**云台板（上 C 板）**主控固件。
整台哨兵由**上下两块 C 板（STM32F407）通过 CAN 互联**，本仓库负责云台这一侧：

- 解算 IMU（BMI088）姿态，对 `yaw` / `pitch` 做双轴闭环控制
- 解析 DBUS 遥控器帧，并支持 USB 命令通道（视觉上位机 / 操作手）
- 通过自定义串口协议与**视觉上位机**交换姿态与目标量 → [云台串口协议.md](云台串口协议.md)
- 解析 DJI **裁判系统**数据帧（比赛状态、血量、弹量等）
- 控制拨弹盘与两个摩擦轮（发射机构）

## 硬件构成

| 部件 | 型号 | 接口 | 备注 |
|---|---|---|---|
| 主控 | STM32F407IGHx | — | 云台板（上 C 板） |
| 姿态 | BMI088 | — | 六轴 IMU，`ImuTask` 解算 |
| yaw | DJI 6020 | CAN | ID2，帧头 `0x1FF` / `0x2FF` |
| pitch | LK 电机 | CAN | ID1，帧头 `0x141` |
| 摩擦轮 ×2 | DJI 电机 | CAN | 左 ID2 / 右 ID1 |
| 拨弹盘 | DJI 3508 | CAN | ID2，帧头 `0x200` / `0x1FF` |
| 遥控器 | DBUS 接收机 | USART | 18 字节帧 |
| 上位机 | 视觉 / 操作手 | USART + USB | 见下方协议 |
| 下板 | 底盘板 | CAN | 与云台板互联 |

> 上下板分工与完整接线以实物和仓库文档 `Communication_Summary.md` 为准。

## 软件架构

FreeRTOS（CMSIS-RTOS 封装）多任务 + 分层模块化：

| 目录 | 职责 |
|---|---|
| `Task/` | `GimbalTask`（云台闭环）、`FireTask`（发射机构）、`ImuTask`（姿态解算）、`ControlCenterTask`（遥控器/USB 指令 → 目标量）、`UsbConnectTask`、`TestTask` |
| `Communication/` | `dbus`、`usart_dma` / `usart_protocol` / `usart_decode`、`usb_protocol` / `usb_decode`、`referee_protocol` / `referee_decode` |
| `Algorithm/` | CRC8 / CRC16、FusionAHRS、MahonyAHRS 姿态融合 |
| `PID/` | `angle_pid`、`speed_pid`、`temp_pid`（含 `PidBase`） |
| `BSP/` | `bsp_can`（CAN 抽象）、`bsp_dwt`（DWT 微秒级计时）、`debug` |
| `BMI088/` | IMU 驱动 |
| `Message_Bus/` | 任务间消息总线 |
| `Debug_vars/` | 在线调试变量（配合上位机示波） |
| `USB_DEVICE/`、`Middlewares/` | ST USB Device 库、FreeRTOS |
| `Core/`、`Drivers/` | CubeMX 生成的启动/外设/HAL |

## 云台 ↔ 视觉 串口协议

两个方向都是**小端序**，帧头固定 `'S'` `'P'`，末 2 字节为 CRC16（校验不包含 CRC 字段自身）。完整定义见 [云台串口协议.md](云台串口协议.md)。

**云台 → 视觉（43 字节）**

| 偏移 | 长度 | 类型 | 字段 | 说明 |
|---:|---:|---|---|---|
| 0–1 | 2 | `uint8_t` | `head` | `'S'` `'P'` |
| 2 | 1 | `uint8_t` | `mode` | 云台当前模式 |
| 3–18 | 16 | `float[4]` | `q` | 姿态四元数 w/x/y/z |
| 19–22 | 4 | `float` | `yaw` | 云台 yaw 角 |
| 23–26 | 4 | `float` | `yaw_vel` | yaw 角速度 |
| 27–30 | 4 | `float` | `pitch` | 云台 pitch 角 |
| 31–34 | 4 | `float` | `pitch_vel` | pitch 角速度 |
| 35–38 | 4 | `float` | `bullet_speed` | 当前弹速 |
| 39–40 | 2 | `uint16_t` | `bullet_count` | 累计发射次数 |
| 41–42 | 2 | `uint16_t` | `crc16` | 覆盖第 0–40 字节 |

**视觉 → 云台（29 字节）**

| 偏移 | 长度 | 类型 | 字段 | 说明 |
|---:|---:|---|---|---|
| 0–1 | 2 | `uint8_t` | `head` | `'S'` `'P'` |
| 2 | 1 | `uint8_t` | `mode` | 控制模式 |
| 3–14 | 12 | `float[3]` | `yaw, yaw_vel, yaw_acc` | 期望 yaw 及一/二阶导 |
| 15–26 | 12 | `float[3]` | `pitch, pitch_vel, pitch_acc` | 期望 pitch 及一/二阶导 |
| 27–28 | 2 | `uint16_t` | `crc16` | 覆盖第 0–26 字节 |

上位机侧配套脚本也在仓库里，可直接联调：

- `generate_gimbal_packet.py` —— 生成一帧「云台 → 视觉」数据
- `generate_gimbal_packet_decode.py` —— 解析「视觉 → 云台」数据

## 编译与烧录

**依赖**

- CMake ≥ 3.22、Ninja
- `arm-none-eabi-gcc` / `arm-none-eabi-g++` **需要加入 PATH**（见 `cmake/gcc-arm-none-eabi.cmake`）
  推荐直接使用 STM32CubeCLT（自带 `gnu-tools-for-stm32`）

**编译**

```bash
cmake --preset Debug
cmake --build --preset Debug     # 发布版把 Debug 换成 Release
```

产物在 `build/<preset>/`，可执行目标名为 `sentriomeni2026`。

仓库里的 `.vscode/settings.json` 使用的是 CubeCLT 的 `cube-cmake` 包装（配合 STM32 VS Code 扩展）。
若用标准 `cmake`，等价写法：

```bash
cmake -B build/Debug -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Debug
cmake --build build/Debug
```

**烧录**：ST-LINK + STM32CubeProgrammer（或 OpenOCD），烧 `build/Debug/sentriomeni2026.elf`。

**外设配置**：`2026sentriomeni.ioc` 是 STM32CubeMX 工程文件。改引脚/外设后重新生成代码时，注意**不要覆盖** `Task/`、`Communication/`、`Algorithm/` 等手写目录。

## 目录结构

```
2026OmniSentryGimbal/
├── Algorithm/            # CRC8 / CRC16、FusionAHRS、MahonyAHRS
├── BMI088/               # IMU 驱动
├── BSP/                  # bsp_can、bsp_dwt、debug
├── Chassis/              # 与底盘侧共用的定义
├── Communication/        # DBUS / USART / USB / 裁判系统协议
├── Core/                 # CubeMX 生成的启动与外设初始化（含 freertos.c）
├── Drivers/              # ST HAL + CMSIS（勿手改）
├── Middlewares/          # FreeRTOS（勿手改）
├── Debug_vars/           # 在线调试变量
├── Message_Bus/          # 任务间消息总线
├── PID/                  # 角度环 / 速度环 / 温度环
├── Task/                 # FreeRTOS 任务
├── USB_DEVICE/           # USB 设备配置
├── cmake/                # 工具链文件
├── 2026sentriomeni.ioc   # CubeMX 工程
├── CMakeLists.txt
├── CMakePresets.json
├── 云台串口协议.md        # 与视觉上位机的协议定义
└── Communication_Summary.md
```

## 说明与限制

- `Drivers/`、`Middlewares/` 是 ST 官方 HAL / CMSIS / FreeRTOS 库，随 CubeMX 生成，**请勿手改**。
- 本仓库未附带 LICENSE：第三方库遵循各自原始许可（ST HAL/CMSIS 为 BSD-3-Clause、FreeRTOS 为 MIT 等），二次使用前请自行确认。
- 未配置 CI，编译与烧录流程需本地验证。

## 致谢

- 裁判系统协议字段参考 DJI RoboMaster 官方《裁判系统串口协议》
- HAL / CMSIS / FreeRTOS / USB Device 库来自 STMicroelectronics 及各自社区

---

### English

Firmware for the **gimbal board** of our 2026 RoboMaster sentry robot — STM32F407IGHx + FreeRTOS + CMake/Ninja + arm-none-eabi-gcc.
It performs IMU attitude estimation (BMI088), dual-axis yaw/pitch closed-loop control (DJI 6020 + LK motor), DBUS/USB command input,
DJI referee-system frame decoding, and a custom CRC16 serial protocol with the vision host (see `云台串口协议.md`).
Counterpart: [chassis board firmware](https://github.com/TC635807/2026OmniSentryChassis).
