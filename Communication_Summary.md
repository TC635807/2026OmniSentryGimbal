# Communication 模块总结文档

## 概述

本文档总结了 `Communication/` 文件夹中的所有通信模块，包括 DBUS 遥控器通信和 USART 串口通信。这些模块构成了 STM32F4 项目中的完整通信系统，支持与遥控器、上位机和其他单片机的数据交换。

## 模块架构

```
Communication/
├── Inc/
│   ├── dbus.h              # DBUS 遥控器通信
│   ├── usart_dma.h         # USART DMA 通信管理
│   ├── usart_protocol.h    # 自定义串口协议解析
│   └── usart_decode.h      # 应用层数据解码
└── Src/
    ├── dbus.cpp            # DBUS 解码实现
    ├── usart_dma.cpp       # DMA 通信实现
    ├── usart_protocol.cpp  # 协议解析实现
    └── usart_decode.cpp    # 数据解码实现
```

## 1. DBUS 模块

### 功能
处理 DBUS 遥控器通信协议，解析来自遥控器的 18 字节数据帧。

### 核心结构体

```c
typedef struct
{
    int16_t ch[5];   // 5个通道数据（摇杆，范围：-1024~1024）
    uint8_t s1;      // 开关 S1 (0-2)
    uint8_t s2;      // 开关 S2 (0-2)
    
    struct {
        int16_t x;   // 鼠标 X 轴
        int16_t y;   // 鼠标 Y 轴
        int16_t z;   // 鼠标 Z 轴（滚轮）
        uint8_t l;   // 左键
        uint8_t r;   // 右键
    } mouse;
    
    uint16_t key;    // 键盘按键（16位掩码）
} DBUS_t;
```

### 全局变量
- `extern DBUS_t dbus;` - 存储解析后的 DBUS 数据

### 主要函数

#### `void dbus_decode(volatile uint8_t* buf, int len)`
- **功能**：解析 DBUS 数据帧
- **参数**：
  - `buf`：指向接收缓冲区的指针
  - `len`：数据长度（必须为 18）
- **说明**：将 18 字节原始数据解析到 `dbus` 结构体中

#### `void DBUS_Decode(volatile uint8_t* buf, int len)`
- **功能**：C 语言接口，调用 `dbus_decode`
- **用途**：供 C 代码调用

### 使用示例
```c
// 在 DMA 回调中调用
void MyCallback(volatile uint8_t* buf, int len) {
    DBUS_Decode(buf, len);
    // 现在可以使用 dbus.ch[0], dbus.s1 等数据
}
```

## 2. USART DMA 模块

### 功能
管理 UART DMA 双缓冲通信，支持多实例、空闲中断检测和 DMA 发送。

### 核心类：`UsartDma`

#### 构造函数
```cpp
UsartDma(UART_HandleTypeDef* huart, DecodeCallback cb);
```
- `huart`：UART 句柄指针
- `cb`：解码回调函数 `void (*)(volatile uint8_t* buf, int len)`

#### 主要方法
- `HAL_StatusTypeDef Transmit_DMA(const uint8_t* data, uint16_t len);` - DMA 发送数据
- `static void IRQHandler(UART_HandleTypeDef* huart);` - 中断处理函数
- `static UsartDma* GetInstance(UART_HandleTypeDef* huart);` - 获取实例

#### 内部机制
- **双缓冲接收**：使用两个缓冲区交替接收，避免数据覆盖
- **空闲中断**：检测数据接收完成
- **多实例支持**：最多支持 3 个 UART 实例

### C 语言接口

#### `void Uart_Init(UART_HandleTypeDef* huart, void (*decode_func)(volatile uint8_t* buf, int len))`
- **功能**：初始化 USART DMA 通信
- **调用时机**：在 `main()` 中初始化外设后调用

#### `void Uart_IRQHandler(UART_HandleTypeDef* huart)`
- **功能**：中断处理函数
- **调用位置**：在 `stm32f4xx_it.c` 的 USART 中断中调用

#### `HAL_StatusTypeDef Uart_Transmit_DMA(UART_HandleTypeDef* huart, const uint8_t* data, uint16_t len)`
- **功能**：通过 DMA 发送数据

### 配置宏
- `UART_RX_BUF_LEN`：接收缓冲区长度（默认 256）
- `DT7DMA_MAX_INSTANCES`：最大实例数（默认 3）

## 3. USART 协议模块

### 功能
解析自定义串口协议，帧格式：`0xFE 0xEF` + 类型 + 长度 + 载荷 + 校验和。

### 核心类：`UartProtocol`

#### 构造函数
```cpp
explicit UartProtocol(FrameCallback cb);
```
- `cb`：帧回调函数 `std::function<void(uint8_t, const uint8_t*, uint8_t)>`

#### 主要方法
- `void input(uint8_t byte);` - 输入单个字节进行解析
- `void feedDMABuffer(volatile uint8_t* buf, int len);` - 批量输入 DMA 缓冲区数据
- `uint8_t buildFrame(uint8_t* out_buf, uint8_t frame_type, const uint8_t* payload, uint8_t len);` - 构建协议帧
- `static uint8_t calcChecksum(const uint8_t* data, uint8_t len);` - 计算校验和

#### 协议格式
```
字节位置  内容        说明
0-1       0xFE 0xEF  帧头
2         帧类型     用户定义（如 0x01=电机命令）
3         载荷长度   N (0-32)
4-(4+N-1) 载荷数据   用户数据
4+N       校验和     (帧类型 + 所有载荷字节) & 0xFF
```

#### 状态机
类内部使用状态机解析：
1. `WAIT_HEAD1` - 等待第一个帧头
2. `WAIT_HEAD2` - 等待第二个帧头
3. `WAIT_TYPE` - 等待帧类型
4. `WAIT_LEN` - 等待载荷长度
5. `WAIT_PAYLOAD` - 接收载荷数据
6. `WAIT_CHECKSUM` - 等待校验和

## 4. USART 解码模块

### 功能
解析应用层数据，处理特定类型的协议帧。

### 核心结构体
```cpp
struct ControlCommand {
    float yaw_target;     // 偏航目标
    float pitch_target;   // 俯仰目标
    float chassis_vx;     // 底盘 X 速度
    float chassis_vy;     // 底盘 Y 速度
    bool updated;         // 是否有新数据
};
```

### 全局变量
- `extern ControlCommand g_control_cmd;` - 全局控制命令
- `extern UartProtocol proto6;` - 协议实例（USART6）
- `extern UartDecode decoder6;` - 解码实例

### 核心类：`UartDecode`

#### 构造函数
```cpp
explicit UartDecode(UartProtocol& proto);
```
- 自动注册回调到协议对象

#### 主要方法
- `void onFrame(uint8_t type, const uint8_t* data, uint8_t len);` - 帧处理回调
- `void handleMotorCmd(const uint8_t* data, uint8_t len);` - 处理电机命令（类型 0x01）

### C 语言接口
#### `void MyUartCallback(volatile uint8_t* buf, int len)`
- **功能**：DMA 回调函数，将数据传递给协议解析器
- **调用位置**：在 `UsartDma` 的回调中调用

## 快速入门指南

### 1. 初始化 DBUS 通信

```c
// 在 main.c 中
extern DBUS_t dbus;

// 在 USART 初始化后
Uart_Init(&huart1, DBUS_Decode);

// 在 stm32f4xx_it.c 中
void USART1_IRQHandler(void) {
    Uart_IRQHandler(&huart1);
}
```

### 2. 使用 DBUS 数据

```c
// 在任何任务或函数中访问
if (dbus.s1 == 1) {
    // 开关 S1 在上位
    float throttle = dbus.ch[2] / 1024.0f;  // 转换为 -1.0~1.0
}
```

### 3. 初始化自定义串口通信

```cpp
// 在 C++ 文件中
extern UartProtocol proto6;
extern UartDecode decoder6;

// 在初始化代码中
Uart_Init(&huart6, MyUartCallback);

// 在中断中
void USART6_IRQHandler(void) {
    Uart_IRQHandler(&huart6);
}
```

### 4. 发送自定义协议帧

```cpp
// 准备数据
uint8_t payload[] = {0x00, 0x64};  // 示例数据
uint8_t frame[32];
uint8_t len = proto6.buildFrame(frame, 0x01, payload, 2);

// 发送
Uart_Transmit_DMA(&huart6, frame, len);
```

### 5. 接收和处理数据

```cpp
// 数据自动通过回调处理
// 可以访问全局变量获取数据
extern ControlCommand g_control_cmd;

if (g_control_cmd.updated) {
    float target_yaw = g_control_cmd.yaw_target;
    g_control_cmd.updated = false;  // 清除更新标志
}
```

## 模块间关系

```
┌─────────────┐    ┌──────────────┐    ┌──────────────┐
│   DBUS      │    │  USART DMA   │    │  USART       │
│  遥控器数据  │◄───┤  双缓冲接收  │◄───┤  硬件接口    │
└──────┬──────┘    └──────┬───────┘    └──────────────┘
       │                  │
       ▼                  ▼
┌─────────────┐    ┌──────────────┐    ┌──────────────┐
│ 应用层使用   │    │ USART协议解析 │    │ USART解码    │
│ (PID,底盘)   │    │   (状态机)   │───►│  (应用逻辑)  │
└─────────────┘    └──────────────┘    └──────────────┘
```

## 调试技巧

### 1. 查看 DBUS 数据
```c
// 在调试变量中查看
extern float debug_yaw, debug_roll, debug_pitch;
// 这些变量在 dbus_decode 中更新
```

### 2. 协议调试
- 使用逻辑分析仪或串口助手查看原始数据
- 检查帧头 `0xFE 0xEF` 是否正确
- 验证校验和计算

### 3. DMA 调试
- 检查 `UART_RX_BUF_LEN` 是否足够
- 确认空闲中断使能
- 检查双缓冲切换标志

## 常见问题

### Q1: DBUS 数据不更新
- 检查遥控器对频
- 确认 USART 波特率设置为 100kbps
- 检查 DMA 缓冲区长度是否为 18

### Q2: 自定义协议帧解析失败
- 确认帧头为 `0xFE 0xEF`
- 检查校验和计算
- 确认载荷长度不超过 32 字节

### Q3: DMA 接收不完整
- 增加 `UART_RX_BUF_LEN`
- 检查空闲中断配置
- 确认 DMA 双缓冲正确初始化

### Q4: 多 UART 实例冲突
- 检查 `DT7DMA_MAX_INSTANCES` 设置
- 确保每个 UART 使用不同的实例
- 在中断中正确调用 `Uart_IRQHandler`

## 扩展建议

### 1. 添加新协议类型
在 `usart_decode.cpp` 的 `onFrame` 函数中添加新的 `case` 分支：

```cpp
case 0x02:  // 新协议类型
    handleNewCommand(data, len);
    break;
```

### 2. 增加发送队列
为避免发送冲突，可以实现发送队列：

```cpp
class UartTxQueue {
    // 实现队列管理
    bool send(const uint8_t* data, uint16_t len);
};
```

### 3. 添加错误统计
在协议解析中添加错误计数器：

```cpp
struct ProtocolStats {
    uint32_t frames_received;
    uint32_t checksum_errors;
    uint32_t framing_errors;
};
```

---

## 总结

本通信系统提供了完整的 DBUS 和 USART 通信解决方案：
1. **DBUS 模块**：专门处理遥控器通信，简单易用
2. **USART DMA 模块**：高效的底层通信，支持多实例
3. **协议模块**：灵活的自定义协议解析
4. **解码模块**：应用层数据处理

通过合理使用这些模块，可以快速实现与遥控器、上位机和其他单片机的可靠通信。

**最后更新**：2026年1月9日  
**适用项目**：STM32F4 系列单片机，基于 HAL 库和 FreeRTOS
