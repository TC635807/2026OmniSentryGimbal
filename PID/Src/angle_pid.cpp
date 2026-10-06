//
// Created by 20852 on 2025/9/21.
//

#include "angle_pid.h"

int16_t AnglePID::calculate(int32_t target,int32_t feedback, float dt) {
    int32_t fb = handleZeroCross(target, feedback);
    float output = Calculate(static_cast<float>(target), static_cast<float>(fb), dt);
    return static_cast<int16_t>(output);
}

//Phase Unwrapping（相位展开）的一种“最近等价表示”
int32_t AnglePID::handleZeroCross(int32_t target, int32_t current) {
    // 使用最近路径原则，让 current 调整到 target 附近
    // 迭代调整直到误差在 ±4096 范围内
    int32_t error = target - current;
    
    // 循环调整直到误差在可接受范围内
    while (error > 4096) {
        current += 8192;
        error = target - current;
    }
    while (error < -4096) {
        current -= 8192;
        error = target - current;
    }
    
    return current;
}

// C接口封装
extern "C" {
    //static AnglePID angle_pid(2.0f, 0.0f, 0.1f, 5000.0f, 5000.0f);
    static AnglePID angle_pid(15.0f, 0.8f, 0.08f, 5000.0f, 5000.0f);

    void angle_pid_clear() {
        angle_pid.Clear();
    }

    int16_t angle_pid_calculate(int32_t target, int32_t actual_angle, float dt) {
        return angle_pid.calculate(target, actual_angle, dt);
    }
}