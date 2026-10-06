//
// Created by TC63 on 2026/2/1.
//
#include "imuTask.h"
#include <cmath>
#include "BMI088.h"
#include "bsp_can.h"
#include "bsp_dwt.h"
#include "ImuTempControl.h"
#include "FusionAHRS.h"
#include "debug_vars.h"
#include "debug.h"
#include "dbus.h"
float gyro[3], accel[3], temp, mag[3];
std::array<float,4> q;
float roll, pitch, yaw;
float roll_dir, pitch_dir, yaw_dir;
static FusionAHRS ahrs(1000.0f);   // 1kHz
volatile float roll_out, pitch_out, yaw_out;
extern DBUS_t dbus;
float gimbal_yaw_total=0;
// 动态零偏
static float yaw_bias = 0.0f;
// 上电零点
static float yaw_zero = 0.0f;
static bool first_run = true;
// 参数
#define GYRO_ZERO_THRESHOLD 0.005236f   // rad/s，对应0.3 deg/s
#define BIAS_ALPHA 0.999f          // 零偏滑动平均系数
#define LOOP_DT 0.001f             // 循环周期 1ms
#define DEG_TO_RAD 0.017453292519943295  // π/180
#define RAD_TO_DEG 57.2957795130823
#include "ImuTask.h"
void StartImuTask::run() {
    BMI088_Init();
    ImuTempControl_Init();
    DWT_Delay_ms(1000);

    // 上电时记录初始 yaw 作为零点
    BMI088_Read(gyro, accel, &temp);
    for (;;) {
        //真实数据读取
        BMI088_Read(gyro, accel, &temp);


        //控温
        ImuTempControl_Update(45, temp, 0.001f);

        // 更新融合算法
        ahrs.update(
            gyro[0],
            gyro[1],
            gyro[2],
            accel[0],
            accel[1],
            accel[2]
        );

        // 获取四元数
        q = ahrs.getQuaternion();
   
                //根据四元数计算欧拉角
        roll_dir  = atan2f(2.0f*(q[0]*q[1] + q[2]*q[3]), 1.0f - 2.0f*(q[1]*q[1] + q[2]*q[2]))*RAD_TO_DEG;
        pitch_dir = asinf(2.0f*(q[0]*q[2] - q[3]*q[1]))*RAD_TO_DEG;
        yaw_dir   = atan2f(2.0f*(q[0]*q[3] + q[1]*q[2]), 1.0f - 2.0f*(q[2]*q[2] + q[3]*q[3]))*RAD_TO_DEG;
        //if(yaw_dir!=0){yaw=yaw_dir;}
        if(pitch_dir!=0){pitch=pitch_dir;}
        if(roll_dir!=0){roll=roll_dir;}
        yaw=yaw_dir;
        if(first_run)
        {
            yaw_zero = yaw;
            first_run = false;
        }
        yaw_out = yaw - yaw_zero;
        roll_out = roll;
        pitch_out = pitch;
        
        // static float prev_actual_yaw = 0.0f;
        // static bool first_actual = true;

        // if (first_actual) {
        //     prev_actual_yaw = yaw;   // 首次运行，将当前实际角度作为基准
        //     first_actual = false;
        // }

        // // 计算实际角度变化量
        // float delta_actual = yaw - prev_actual_yaw;
        // if (delta_actual > 180.0f) {
        //     delta_actual -= 360.0f;
        // } else if (delta_actual < -180.0f) {
        //     delta_actual += 360.0f;
        // }

        // gimbal_yaw_total += delta_actual;     // 累加总角度
        // prev_actual_yaw = yaw;       // 更新上一次角度



        DWT_Delay_ms(1); // 1ms

    }

}
#ifdef __cplusplus
extern "C" {
#endif

    static StartImuTask imuTask;

    void StartImuTask_Init() {
        imuTask.start((char*)"imuTask", 1024, osPriorityNormal);
    }
#ifdef __cplusplus
}
#endif