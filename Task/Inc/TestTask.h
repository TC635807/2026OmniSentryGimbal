//
// ControlCenterTask - 控制中心任务
// 负责读取遥控器或USB命令并规定各个电机的目标角度或目标转速
//

#ifndef CONTROL_CENTER_TASK_H
#define CONTROL_CENTER_TASK_H
#pragma once
#include "TaskBase.h"

#ifdef __cplusplus
class StartTestTask : public TaskBase {
public:
    void run() override;   // 继承并实现 run()
};
#endif

#ifdef __cplusplus
extern "C" {
#endif

    void StartTestTask_Init();

#ifdef __cplusplus
}
#endif

#endif // CONTROL_CENTER_TASK_H
