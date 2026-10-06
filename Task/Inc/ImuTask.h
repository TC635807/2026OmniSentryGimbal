//
// Created by TC63 on 2026/2/1.
//

#ifndef SENTRIOMENI2026_IMUTASK_H
#define SENTRIOMENI2026_IMUTASK_H
#pragma once
#include "TaskBase.h"

#ifdef __cplusplus
class StartImuTask : public TaskBase {
public:
    void run() override;   // 继承并实现 run()
};
#endif

#ifdef __cplusplus
extern "C" {
#endif

    void StartImuTask_Init();

#ifdef __cplusplus
}
#endif

#endif //SENTRIOMENI2026_IMUTASK_H