//
// PID控制器类实现
// 包含位置式和增量式PID，支持输出限幅、积分限幅、抗积分饱和
//

#include "../PID/PID.h"
#include <cmath>
#include <algorithm>

PID::PID()
    : kp_(0.0f), ki_(0.0f), kd_(0.0f)
    , integral_(0.0f), prev_error_(0.0f), prev_output_(0.0f)
    , output_min_(-1000.0f), output_max_(1000.0f)
    , integral_min_(-1000.0f), integral_max_(1000.0f)
    , dead_zone_(0.0f), dt_(0.001f)  // 默认1ms采样周期
    , mode_(PID_MODE_POSITION)
{
}

PID::PID(float kp, float ki, float kd, float dt)
    : kp_(kp), ki_(ki), kd_(kd), dt_(dt)
    , integral_(0.0f), prev_error_(0.0f), prev_output_(0.0f)
    , output_min_(-1000.0f), output_max_(1000.0f)
    , integral_min_(-1000.0f), integral_max_(1000.0f)
    , dead_zone_(0.0f)
    , mode_(PID_MODE_POSITION)
{
}

void PID::init(float kp, float ki, float kd, float dt)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    dt_ = dt;
    
    // 重置状态
    reset();
}

void PID::reset()
{
    integral_ = 0.0f;
    prev_error_ = 0.0f;
    prev_output_ = 0.0f;
}

void PID::setGains(float kp, float ki, float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PID::setOutputLimits(float min, float max)
{
    if (min <= max) {
        output_min_ = min;
        output_max_ = max;
    }
}

void PID::setIntegralLimits(float min, float max)
{
    if (min <= max) {
        integral_min_ = min;
        integral_max_ = max;
    }
}

void PID::setDeadZone(float dead_zone)
{
    dead_zone_ = std::abs(dead_zone);
}

void PID::setMode(PIDMode mode)
{
    mode_ = mode;
}

void PID::setSampleTime(float dt)
{
    if (dt > 0.0f) {
        // 调整积分项以保持相同的积分效果
        if (dt_ > 0.0f) {
            integral_ *= dt / dt_;
        }
        dt_ = dt;
    }
}

float PID::calculate(float setpoint, float measurement)
{
    float error = setpoint - measurement;
    
    // 死区处理
    if (std::abs(error) <= dead_zone_) {
        error = 0.0f;
    }
    
    float output = 0.0f;
    
    if (mode_ == PID_MODE_POSITION) {
        output = calculatePosition(error);
    } else {
        output = calculateIncremental(error);
    }
    
    // 输出限幅
    output = clamp(output, output_min_, output_max_);
    
    // 保存状态
    prev_error_ = error;
    prev_output_ = output;
    
    return output;
}

float PID::calculatePosition(float error)
{
    // 比例项
    float p_term = kp_ * error;
    
    // 积分项（带限幅）
    integral_ += ki_ * error * dt_;
    integral_ = clamp(integral_, integral_min_, integral_max_);
    
    // 微分项
    float d_term = 0.0f;
    if (dt_ > 0.0f) {
        d_term = kd_ * (error - prev_error_) / dt_;
    }
    
    return p_term + integral_ + d_term;
}

float PID::calculateIncremental(float error)
{
    // 增量式PID公式：Δu = Kp*(e(k)-e(k-1)) + Ki*e(k) + Kd*(e(k)-2e(k-1)+e(k-2))
    // 这里简化为使用prev_error_作为e(k-1)
    
    float delta_output = 0.0f;
    
    // 比例项增量
    delta_output += kp_ * (error - prev_error_);
    
    // 积分项
    delta_output += ki_ * error * dt_;
    
    // 微分项（需要e(k-2)，这里简化为使用prev_error_的变化率）
    if (dt_ > 0.0f) {
        // 简化的微分项，实际应用中可能需要存储e(k-2)
        delta_output += kd_ * (error - 2 * prev_error_ + prev_prev_error_) / dt_;
    }
    
    // 更新历史误差（用于下一次计算）
    prev_prev_error_ = prev_error_;
    
    // 计算输出
    float output = prev_output_ + delta_output;
    
    // 积分抗饱和处理（条件积分）
    if ((output >= output_max_ && error > 0) || (output <= output_min_ && error < 0)) {
        // 输出饱和且误差方向相同，停止积分
        // 在实际增量式PID中，这需要特殊处理
    }
    
    return output;
}

float PID::calculateWithFeedforward(float setpoint, float measurement, float feedforward)
{
    float pid_output = calculate(setpoint, measurement);
    return clamp(pid_output + feedforward, output_min_, output_max_);
}

float PID::clamp(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

float PID::getKp() const { return kp_; }
float PID::getKi() const { return ki_; }
float PID::getKd() const { return kd_; }
float PID::getIntegral() const { return integral_; }
float PID::getPrevError() const { return prev_error_; }
float PID::getOutputMin() const { return output_min_; }
float PID::getOutputMax() const { return output_max_; }
float PID::getDeadZone() const { return dead_zone_; }
float PID::getSampleTime() const { return dt_; }
PID::PIDMode PID::getMode() const { return mode_; }

// 串级PID支持
void PID::setCascadeMode(bool enable, PID* outer_pid)
{
    cascade_mode_ = enable;
    outer_pid_ = outer_pid;
}

float PID::calculateCascade(float outer_output, float measurement)
{
    if (!cascade_mode_ || outer_pid_ == nullptr) {
        return calculate(outer_output, measurement);
    }
    
    // 串级PID：外环输出作为内环的设定值
    return calculate(outer_output, measurement);
}
