//
// 改进的SpeedPID类 - 支持更多参数和功能
// 基于FireTask.cpp的实际需求进行扩展
//
#ifndef STARTM3508_PID_H
#define STARTM3508_PID_H

#pragma once
#include "../PidBase.h"

#ifdef __cplusplus
class SpeedPID : public PidBase {
public:
    // 构造函数：支持FireTask.cpp中的所有参数（7参数版本）
    SpeedPID(float kp, float ki, float kd, 
             float max_out, float min_out,
             float err_max, float err_min);
    
    // 构造函数：兼容PidBase的5参数版本（用于C接口）
    SpeedPID(float kp, float ki, float kd, 
             float max_out, float max_iout);
    
    // 设置误差限幅
    void setErrorLimits(float max, float min);
    
    // 获取误差限幅
    float getErrorMax() const;
    float getErrorMin() const;
    
    // 重写Calculate函数以支持误差限幅
    float Calculate(float target, float actual, float dt) override;
    
private:
    float error_max_;   // 最大误差限幅
    float error_min_;   // 最小误差限幅
};
#endif

#ifdef __cplusplus
extern "C" {
#endif

    void speed_pid_clear();
    int16_t  speed_pid_calculate(float setpoint, float feedback, float dt);

#ifdef __cplusplus
}
#endif

#endif //STARTM3508_PID_H
