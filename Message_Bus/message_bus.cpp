//
// Created by 20852 on 2025/10/17.
//

#include "message_bus.h"
#include <cstdint>
#include "stdint.h"
#ifdef __cplusplus

// 全局数据实例定义
// LocalImuData imu_data;  // 已废弃——ImuTask全局变量直接访问
ControlTarget control_target;
MotorOutputData motor_output;
uint16_t TotalAmmoShooted=0;
float AmmoSpeed=0;
uint16_t hot_17mm=0;
#endif
