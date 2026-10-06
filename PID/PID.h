//
// PID控制器类头文件
// 提供位置式和增量式PID控制，支持输出限幅、积分限幅、抗积分饱和
//

#ifndef SENTRIOMENI2026_PID_H
#define SENTRIOMENI2026_PID_H

class PID {
public:
    // PID模式枚举
    enum PIDMode {
        PID_MODE_POSITION,    // 位置式PID
        PID_MODE_INCREMENTAL  // 增量式PID
    };
    
    // 构造函数
    PID();
    PID(float kp, float ki, float kd, float dt = 0.001f);
    
    // 初始化函数
    void init(float kp, float ki, float kd, float dt = 0.001f);
    
    // 重置PID状态
    void reset();
    
    // 参数设置函数
    void setGains(float kp, float ki, float kd);
    void setOutputLimits(float min, float max);
    void setIntegralLimits(float min, float max);
    void setDeadZone(float dead_zone);
    void setMode(PIDMode mode);
    void setSampleTime(float dt);
    
    // 计算函数
    float calculate(float setpoint, float measurement);
    float calculateWithFeedforward(float setpoint, float measurement, float feedforward);
    
    // 串级PID支持
    void setCascadeMode(bool enable, PID* outer_pid = nullptr);
    float calculateCascade(float outer_output, float measurement);
    
    // 获取当前参数和状态
    float getKp() const;
    float getKi() const;
    float getKd() const;
    float getIntegral() const;
    float getPrevError() const;
    float getOutputMin() const;
    float getOutputMax() const;
    float getDeadZone() const;
    float getSampleTime() const;
    PIDMode getMode() const;
    
private:
    // 核心计算函数
    float calculatePosition(float error);
    float calculateIncremental(float error);
    
    // 辅助函数
    float clamp(float value, float min, float max);
    
    // PID参数
    float kp_;          // 比例系数
    float ki_;          // 积分系数
    float kd_;          // 微分系数
    float dt_;          // 采样时间（秒）
    
    // PID状态
    float integral_;        // 积分项
    float prev_error_;      // 上一次误差
    float prev_prev_error_; // 上上次误差（用于增量式）
    float prev_output_;     // 上一次输出
    
    // 限幅参数
    float output_min_;      // 输出最小值
    float output_max_;      // 输出最大值
    float integral_min_;    // 积分项最小值
    float integral_max_;    // 积分项最大值
    
    // 其他参数
    float dead_zone_;       // 死区大小
    PIDMode mode_;          // PID模式
    
    // 串级PID相关
    bool cascade_mode_;
    PID* outer_pid_;
};

#endif // SENTRIOMENI2026_PID_H
