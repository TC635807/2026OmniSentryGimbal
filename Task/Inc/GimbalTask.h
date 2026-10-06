//
// GimbalTask - 云台控制任务
// 负责计算当前值和ControlCenter的误差并计算PID
//

#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H


#pragma once
#include "TaskBase.h"

#ifdef __cplusplus
class StartGimbalTask : public TaskBase {
public:
    void run() override;   // 继承并实现 run()
};
#endif
void setYawTarget(float* target, float current, float input);
#ifdef __cplusplus
extern "C" {
#endif

    void StartGimbalTask_Init();

#ifdef __cplusplus
}
#endif

#endif // GIMBAL_TASK_H
