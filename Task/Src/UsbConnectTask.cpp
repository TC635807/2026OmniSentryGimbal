//
// UsbConnectTask - USB连接任务
// 基于用户提供的代码重写，与视觉模块进行云台串口协议通信
// 协议参考：云台串口协议.md
//

#include "UsbConnectTask.h"
#include "cmsis_os.h"
#include "usbd_cdc_if.h"
#include "message_bus.h"
#include "CRC16.h"
#include "bsp_dwt.h"
#include "usb_decode.h"
#include <cmath>
// 引用ImuTask中的全局变量（坐标系已对齐）
extern std::array<float,4> q;     // 四元数（AHRS融合输出，直接发送给视觉）
extern float gyro[3];             // gyro[0]=Roll角速度, gyro[1]=Pitch角速度, gyro[2]=Yaw角速度
extern float roll, pitch, yaw;    // 欧拉角（度）
// 全局控制目标声明
extern ControlTarget control_target;

// 协议解析器
extern USBDecode usb_decoder;

// 外部声明hUsbDeviceFS
extern USBD_HandleTypeDef hUsbDeviceFS;

// USB接收队列（从usbd_cdc_if.c定义）
extern QueueHandle_t usbRxQueue;

extern uint16_t TotalAmmoShooted;
extern float AmmoSpeed;

VisionToGimbalData vision_data;

USBProtocol::GimbalToVision tx_pkt;

StartUsbConnectTask::StartUsbConnectTask() {
    // 构造函数
    memset(&vision_data, 0, sizeof(vision_data));
}

void StartUsbConnectTask::run() {
    uint8_t byte;
    for (;;) {
        // 视觉-->云台
        if (usbRxQueue != NULL) {
            while (xQueueReceive(usbRxQueue, &byte, 0) == pdTRUE)
            {
                if (usb_decoder.feed(byte))
                {
                    const auto& pkt = usb_decoder.packet();
                    vision_data.mode = pkt.mode;
                    vision_data.yaw = remainderf(pkt.yaw,360);
                    vision_data.yaw_vel = pkt.yaw_vel;
                    vision_data.yaw_acc = pkt.yaw_acc;
                    vision_data.pitch = pkt.pitch;
                    vision_data.pitch_vel = pkt.pitch_vel;
                    vision_data.pitch_acc = pkt.pitch_acc;
                }
            }
        }

        // 云台-->视觉
        if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
        {
 
                // 确定云台模式：根据friction_ready决定
                uint8_t gimbal_mode = 0; // 默认空闲模式
                if (control_target.friction_ready) {
                    gimbal_mode = 1; // 自瞄模式
                }

                // 直接发送ImuTask中FusionAHRS输出的四元数q
                // 各轴角速度：gyro[2]=Yaw, gyro[1]=Pitch, gyro[0]=Roll
                USBProtocol::buildFrame(
                    tx_pkt,
                    gimbal_mode,
                    q,                                  // 直接发送原始四元数
                    yaw * (3.14159265f / 180.0f), // yaw(deg→rad)
                    gyro[2],                            // yaw角速度
                    pitch * (3.14159265f / 180.0f),     // pitch角度(deg→rad)
                    gyro[1],                            // pitch角速度
                    AmmoSpeed,                          // 子弹速度
                    TotalAmmoShooted                    // 子弹计数
                );

                // 发送数据
             
                while (
                    CDC_Transmit_FS(
                           reinterpret_cast<uint8_t*>(&tx_pkt),
                           USBProtocol::GIMBAL_TO_VISION_LEN
                       ) == USBD_BUSY) {
                        osDelay(5);
                }
            
        }
        osDelay(10);
    }
}

// 全局任务实例
static StartUsbConnectTask usbconnectTask;

#ifdef __cplusplus
extern "C" {
#endif

void USB_Data_Received(const unsigned char *data, unsigned short length) {
    for (unsigned short i = 0; i < length; i++) {
        uint8_t byte = data[i];
        (void) byte;
    }
}

void StartUsbConnectTask_Init() {
    usbconnectTask.start((char *) "StartUsbConnectTask", 2048, osPriorityNormal);
}
#ifdef __cplusplus
}
#endif