//
// Created by 20852 on 2025/10/17.
//

#ifndef STARTM3508_MESSAGE_BUS_H
#define STARTM3508_MESSAGE_BUS_H

#pragma once
#ifdef __cplusplus

#include "bsp_can.h"
#include "dbus.h"

// IMU本地数据（已废弃——数据现已直接通过ImuTask全局变量访问）
// struct LocalImuData {
//     float roll, pitch, yaw;
//     float gyro[3], accel[3];
//     float temperature;
//     float gimbal_yaw_total_angle;
// };

// 控制目标数据（ControlCenter写入）
struct ControlTarget {
    // 底盘目标速度
    float chassis_vx, chassis_vy, chassis_wz;  // 麦克纳姆轮速度指令
    
    // 云台目标角度
    float gimbal_yaw_angle;      // Yaw目标总角度
    float gimbal_yaw_angle_single;      // Yaw目标单圈角度
    float gimbal_pitch_angle;    // Pitch目标角度

    // 摩擦轮目标速度
    float friction_ready;  // 摩擦轮速度
    
    // 拨弹盘状态
    int feeder_ready;           // 拨弹状态
    
    // 控制模式
    enum ControlMode {
        MODE_RC = 0,    // 遥控器控制
        MODE_RC_follow = 1,    // 遥控器底盘跟随云台控制
        MODE_AI=2 ///xiaoidnan
    } control_mode;
    
    // 急停标志
    bool emergency_stop;
};

// 电机输出数据（用于任务间共享电机控制输出）
struct MotorOutputData {
    int16_t feeder_current;    // 拨弹盘电机电流
    int16_t friction_left;     // 左摩擦轮电流
    int16_t friction_right;    // 右摩擦轮电流
};

// 全局数据实例
// extern LocalImuData imu_data;  // 已废弃——ImuTask全局变量直接访问
extern ControlTarget control_target;
extern MotorOutputData motor_output;
extern uint16_t TotalAmmoShooted;
extern float AmmoSpeed;
extern uint16_t hot_17mm;
#endif

#endif //STARTM3508_MESSAGE_BUS_H
