//
// 改进的SpeedPID类实现
// 支持FireTask.cpp中的7参数构造函数和误差限幅功能
//
#include "../Inc/speed_pid.h"
#include <cmath>

// 构造函数：支持FireTask.cpp中的所有参数（7参数）
SpeedPID::SpeedPID(float kp, float ki, float kd, 
                   float max_out, float min_out,
                   float err_max, float err_min)
    : PidBase(kp, ki, kd, std::fmax(std::fabs(max_out), std::fabs(min_out)), 1000.0f), // 默认积分限幅1000
      error_max_(err_max),
      error_min_(err_min)
{
    // 设置输出限幅
    maxOutput = std::fmax(std::fabs(max_out), std::fabs(min_out));
    // 保存原始的max_out和min_out用于输出限幅
    if (max_out >= min_out) {
        // 正常情况：max_out > min_out
    } else {
        // 如果min_out > max_out，交换它们
        float temp = max_out;
        max_out = min_out;
        min_out = temp;
    }
}

// 扩展的构造函数：兼容PidBase的5参数版本
SpeedPID::SpeedPID(float kp, float ki, float kd, 
                   float max_out, float max_iout)
    : PidBase(kp, ki, kd, max_out, max_iout),
      error_max_(1000.0f),
      error_min_(-1000.0f)
{
}

// 设置误差限幅
void SpeedPID::setErrorLimits(float max, float min) {
    if (max >= min) {
        error_max_ = max;
        error_min_ = min;
    } else {
        error_max_ = min;
        error_min_ = max;
    }
}

// 获取误差限幅
float SpeedPID::getErrorMax() const {
    return error_max_;
}

float SpeedPID::getErrorMin() const {
    return error_min_;
}

// 重写Calculate函数以支持误差限幅
float SpeedPID::Calculate(float target, float actual, float dt) {
    float error = target - actual;
    
    // 误差限幅处理
    if (error > error_max_) {
        error = error_max_;
    } else if (error < error_min_) {
        error = error_min_;
    }
    
    // 检测是否切换了目标方向（正/负）
    if ((prevTarget * target) < 0.0f) {
        integral = 0.0f; // 清除历史积分，防止反向时积分抵消不对称
    }
    
    // 微分
    float derivative = 0.0f;
    if (dt > 0.0f) {
        derivative = (error - prevError) / dt;
    }
    
    // P+D输出
    float output = Kp * error + Kd * derivative;
    
    // 积分防饱和
    if (output < maxOutput && output > -maxOutput) {
        integral += Ki * error * dt;
        // 积分限幅
        if (integral > maxIntegral) {
            integral = maxIntegral;
        } else if (integral < -maxIntegral) {
            integral = -maxIntegral;
        }
    }
    
    output += integral;
    
    // 输出限幅
    if (output > maxOutput) {
        output = maxOutput;
    } else if (output < -maxOutput) {
        output = -maxOutput;
    }
    
    // 误差保存
    prevError = error;
    
    // 目标保存，便于判断正负方向
    prevTarget = target;
    
    return output;
}

// C接口封装
extern "C" {
    static SpeedPID speed_pid(0.5f, 0.01f, 0.001f, 5000.0f, 200.0f);

    void speed_pid_clear() {
        speed_pid.Clear();
    }

    int16_t speed_pid_calculate(float target_speed, float actual_speed, float dt) {
        float output = speed_pid.Calculate(target_speed, actual_speed, dt);
        return static_cast<int16_t>(output);
    }
}