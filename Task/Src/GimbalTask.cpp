//
// GimbalTask - 云台控制任务实现
// 负责计算当前值和ControlCenter的误差并计算PID
//
#define RAD_TO_DEG 57.2957795131f
#define DEG_TO_RAD 0.01745329252f
#include "GimbalTask.h"
#include "UsbConnectTask.h"
#include "angle_pid.h"
#include "message_bus.h"
extern float gyro[3];
extern float roll, pitch, yaw;
extern float gimbal_yaw_total;
extern float yaw_out;
#include "Motor.h"
#include "bsp_can.h"
#include "cmsis_os.h"
#include "speed_pid.h"
float yaw_little_rad=0;
// 引用全局电机变量（在bsp_can.cpp中定义）
extern DJI_motor_info motor_5;
extern LK_motor_info LK_motor_1;
int16_t yaw_current_out;
int16_t pitch_current_out;
extern   VisionToGimbalData vision_data;
float cheak =0.0f;
void StartGimbalTask::run() {
    // 初始化PID控制器
    // Yaw角度环PID参数 (KP, KI, KD, max_output, max_integral)+
    // 调整参数以减少抖动：减小KP，增加KD，添加小的KI
  AnglePID YawAnglePID(80.0f, 2.0f, 2.0f, 20000.0f, 480.0f);    // Yaw速度环PID参数 (KP, KI, KD, max_out      , max_iout)
    SpeedPID YawSpeedPID(20.0f, 0.0f, 0.02f, 40000.0f, 200000.0f);
    // Pitch角度环PID
    AnglePID PitchAnglePID(2.7f, 2.0f, 0.06f, 10000.0f, 5000.0f);
    PitchAnglePID.maxIntegral =1000;
    // Pitch速度环PID
    SpeedPID PitchSpeedPID(40.0f, 1.0f, 0.0f, 30000.0f, 20000.0f);


    float dt = 0.001f; // 1ms控制周期
    
    // 角度限幅 (度)
    const float pitch_angle_limit_max = 20.0f; // Pitch最大角度 +20度
    const float pitch_angle_limit_min = -17.0f;// Pitch最小角度 -10度

    for (;;){
        //限幅
        if(pitch>pitch_angle_limit_max+5.0f||pitch<pitch_angle_limit_min-5.0f)
            {
            PitchAnglePID.integral=0;}
        if(control_target.gimbal_pitch_angle>pitch_angle_limit_max) {

            control_target.gimbal_pitch_angle=pitch_angle_limit_max;

        }
        if(control_target.gimbal_pitch_angle<pitch_angle_limit_min) {

            control_target.gimbal_pitch_angle=pitch_angle_limit_min;

        }
        setYawTarget(&yaw_little_rad,yaw_out,control_target.gimbal_yaw_angle_single);
        // ========== Yaw轴控制 ==========

        // 1. 角度外环：计算目标速度（使用float版本，避免int32_t转换问题）
        float yaw_target_speed = YawAnglePID.Calculate_angle(
            yaw_little_rad,
            yaw,
            dt
        );
        
        // 2. 速度内环：计算控制电流
        // 将电机速度从rpm转换为deg/s (DJI3508电机：8192编码器分辨率，减速比可能为1:1)
        // 根据实际情况调整速度转换系数
        //float motor5_speed_deg = motor_5.rotor_speed * 0.1f; // 假设转换系数

        float yaw_current = YawSpeedPID.Calculate(
            yaw_target_speed,
            gyro[2],
            dt
        );
        
        // 电流限幅
        yaw_current_out = (int16_t)clamp(yaw_current, -50000.0f, 50000.0f);

        // ========== Pitch轴控制 ==========
        // (这里使用LK_motor_1控制pitch，需要根据实际情况调整)
        bsp_can1_lkreadstatus2();
        float pitch_target_speed = //vision_data.pitch*5;
            PitchAnglePID.Calculate_with(
            control_target.gimbal_pitch_angle+0.1,
            pitch,
            dt
        );
        cheak=PitchAnglePID.integral;
        // Pitch速度环 (使用LK电机)
        float pitch_current = PitchSpeedPID.Calculate(
            pitch_target_speed,
            gyro[1],
            dt
        );
        
        pitch_current_out = (int16_t)clamp(pitch_current, -30000.0f, 30000.0f);//+ (-pitch) * 20;

        osDelay(1); // 控制周期与输出周期同步    }
}
}
// 在设置目标角度时，自动选择最短路径方向
void setYawTarget(float* target, float current, float input) {
  float normalized = input;
    while (normalized - current > 180.0f) normalized -= 360.0f;
    while (normalized - current < -180.0f) normalized += 360.0f;
    *target = normalized;}
#ifdef __cplusplus
extern "C" {
#endif

    static StartGimbalTask gimbalTask;

    void StartGimbalTask_Init() {
        gimbalTask.start((char*)"gimbalTask", 2048, osPriorityNormal);
    }
#ifdef __cplusplus
}
#endif