//
// Created by TC63 on 2026/2/8.
// 根据云台串口协议重新设计的USB连接任务
// 用于与视觉模块（小电脑）进行云台串口协议通信
//

#ifndef SENTRIOMENI2026_USBCONNECTTASK_H
#define SENTRIOMENI2026_USBCONNECTTASK_H
//
#pragma once
#include "TaskBase.h"
#include "main.h"
#include "cmsis_os.h"
#include "usbd_cdc_if.h"  // 用于APP_RX_DATA_SIZE定义
#include "CRC16.h"         // 用于CRC16校验

// 云台串口协议常量定义
#define GIMBAL_PROTOCOL_SOF_1 0x53      // 帧头第一个字节 'S'
#define GIMBAL_PROTOCOL_SOF_2 0x50      // 帧头第二个字节 'P'

// 数据包大小定义（根据云台串口协议.md）
#define GIMBAL_TO_VISION_SIZE 43        // 云台→视觉数据包大小：43字节（修正：原41字节错误）
#define VISION_TO_GIMBAL_SIZE 29        // 视觉→云台数据包大小：29字节

// 云台模式定义
#define GIMBAL_MODE_IDLE    0           // 空闲模式
#define GIMBAL_MODE_AIM     1           // 自瞄模式
#define GIMBAL_MODE_SMALL   2           // 小符模式
#define GIMBAL_MODE_BIG     3           // 大符模式

// 控制模式定义
#define CONTROL_MODE_NONE   0           // 不控制
#define CONTROL_MODE_CONTROL 1          // 控制但不开火
#define CONTROL_MODE_FIRE   2           // 控制且开火

// 用户描述的原始数据结构
#ifdef __cplusplus
extern "C" {
#endif

#pragma pack(push, 1)

// 视觉→云台数据包结构（29字节）
typedef struct {
    uint8_t head[2];      // 帧头：0x53 0x50 ('S' 'P')
    uint8_t mode;         // 控制模式：0=不控制, 1=控制但不开火, 2=控制且开火
    float yaw;           // 目标Yaw角度（弧度）
    float yaw_vel;       // 目标Yaw角速度（弧度/秒）
    float yaw_acc;       // 目标Yaw角加速度（弧度/秒²）
    float pitch;         // 目标Pitch角度（弧度）
    float pitch_vel;     // 目标Pitch角速度（弧度/秒）
    float pitch_acc;     // 目标Pitch角加速度（弧度/秒²）
    uint16_t crc16;      // CRC16校验值（小端序）
} vision_to_gimbal_t;
    // 全局变量用于存储视觉到云台的数据（临时存储，将映射到control_target）
    struct VisionToGimbalData {
        float mode;
        float yaw;
        float yaw_vel;
        float yaw_acc;
        float pitch;
        float pitch_vel;
        float pitch_acc;
    };
// 云台→视觉数据包结构（43字节）
typedef struct {
    uint8_t head[2];      // 帧头：0x53 0x50 ('S' 'P')
    uint8_t mode;         // 云台模式：0=空闲, 1=自瞄, 2=小符, 3=大符
    float q[4];           // 四元数，顺序为 [w, x, y, z]
    float yaw;           // Yaw角度（弧度）
    float yaw_vel;       // Yaw角速度（弧度/秒）
    float pitch;         // Pitch角度（弧度）
    float pitch_vel;     // Pitch角速度（弧度/秒）
    float bullet_speed;  // 子弹速度（m/s）
    uint16_t bullet_count; // 子弹累计发送次数
    uint16_t crc16;      // CRC16校验值（小端序）
} gimbal_to_vision_t;

#pragma pack(pop)

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
class StartUsbConnectTask : public TaskBase {
public:
    StartUsbConnectTask();  // 构造函数声明
    
    void run() override;   // 继承并实现 run()
    
private:
    // USB数据接收缓冲区
    unsigned char rx_buffer[APP_RX_DATA_SIZE];  // 使用CDC定义的缓冲区大小
    unsigned short rx_index;                    // 当前缓冲区索引
    unsigned long last_receive_time;            // 最后接收时间
    unsigned long last_send_time;               // 最后发送时间

    uint32_t UsbConnectLastTime=0;
};
#endif


#ifdef __cplusplus
extern "C" {
#endif

    void USB_Data_Received(const unsigned char* data, unsigned short length);
    void StartUsbConnectTask_Init();

#ifdef __cplusplus
}
#endif


#endif //SENTRIOMENI2026_USBCONNECTTASK_H
