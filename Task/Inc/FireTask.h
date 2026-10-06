//
// Created by TC63 on 2026/2/4.
//

#ifndef SENTRIOMENI2026_FIRETASK_H
#define SENTRIOMENI2026_FIRETASK_H
#include "TaskBase.h"

#ifdef __cplusplus
class StartFireTask : public TaskBase {
public:
    void run() override;   // 继承并实现 run()
};
#endif

#ifdef __cplusplus
extern "C" {
#endif

    void StartFireTask_Init();

#ifdef __cplusplus
}
#endif

#endif //SENTRIOMENI2026_FIRETASK_H