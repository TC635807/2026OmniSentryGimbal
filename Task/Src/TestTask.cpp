//
// ControlCenterTask - 控制中心任务实现
// 负责读取遥控器或USB命令并规定各个电机的目标角度或目标转速
//

#include "TestTask.h"
#include "message_bus.h"
#include "dbus.h"
#include "cmsis_os.h"
#include "bsp_can.h"
// 遥控器通道映射
// ch[0]: 左摇杆左右 (-660~660)
// ch[1]: 左摇杆前后 (-660~660)
// ch[2]: 右摇杆左右 (-660~660)
// ch[3]: 右摇杆前后 (-660~660)
// s1, s2: 开关状态

void StartTestTask::run() {

    for (;;) {

    }
}
#ifdef __cplusplus
extern "C" {
#endif

    static StartTestTask testTask;

    void StartTestTask_Init() {
        testTask.start((char*)"TestTask", 256, osPriorityNormal);
    }
#ifdef __cplusplus
}
#endif