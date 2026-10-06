//
// ControlCenterTask - 控制中心任务实现
// 负责读取遥控器或USB命令并规定各个电机的目标角度或目标转速
//

#include "ControlCenterTask.h"
#include "UsbConnectTask.h"
#include "dbus.h"
#include "message_bus.h"
#include "cmsis_os.h"
#include "bsp_can.h"
#include "stm32f4xx_hal.h"
#include <cmath>
// 遥控器通道映射
// ch[0]: 左摇杆左右 (-660~660)
// ch[1]: 左摇杆前后 (-660~660)
// ch[2]: 右摇杆左右 (-660~660)
// ch[3]: 右摇杆前后 (-660~660)
// s1, s2: 开关状态
extern int16_t yaw_current_out ;
extern int16_t pitch_current_out;
extern  int control_output_motor_1 ;
extern  int control_output_motor_2 ;
extern  int control_output_motor_3 ;
extern  VisionToGimbalData  vision_data;
extern float yaw;  // 引用ImuTask中的全局yaw
extern float pitch;  // 引用ImuTask中的全局yaw
extern float roll;  // 引用ImuTask中的全局yaw
float gimbal_yaw_offset = 0.0f;  // 遥控器累积的偏移量l
uint32_t vision_mode_start_tick = 0;
             float filtered_yaw = 0.0f;
            float filtered_pitch = 0.0f;
//std::array<float, 4> eulerToQuaternion(float yaw, float pitch, float roll);
void StartControlCenterTask::run() {
    // 初始化控制目标
    bsp_can1_lkmotorstartcmd();
    control_target.chassis_vx = 0.0f;
    control_target.chassis_vy = 0.0f;
    control_target.chassis_wz = 0.0f;
    control_target.gimbal_yaw_angle = 1.0f;
    control_target.gimbal_pitch_angle = 0.0f;
    control_target.friction_ready = false;
    control_target.feeder_ready = 0;
    control_target.control_mode = ControlTarget::MODE_RC;
    control_target.emergency_stop = false;

    for (;;) {
        // 读取遥控器数据
        // DBUS数据已在DMA中断中更新到全局变量dbus

        // 根据开关状态判断控制模式

        if (dbus.s1 == 1) {
            control_target.control_mode = ControlTarget::MODE_RC;  // 遥控器模式
        } else if (dbus.s1 == 2) {
            control_target.control_mode = ControlTarget::MODE_RC_follow;
        }else if (dbus.s1 == 3) {
            control_target.control_mode = ControlTarget::MODE_AI;  // USB模式（待实现）
        }
        
        
       
        // ========== 云台控制 ========
        // 映射到control_target中（根据用户要求）
        // 注意：视觉发送的是弧度制，control_target需要角度制
        const float RAD_TO_DEG = 180.0f / 3.14159265358979323846f;

        // 只更新角度，不更新其他控制模式相关字段
        // 根据pkt.mode决定是否更新control_target
        if(vision_data.mode == 1||vision_data.mode == 2) {  // CONTROL模式


            const float FILTER_ALPHA = 0.5f;  // 滤波系数，越小越平滑
//    // 模式刚切换时，以当前 yaw 为基准
//         filtered_yaw = yaw;   // 用当前 IMU 角度重置
        filtered_pitch = pitch;
//             // Vision目标滤波
//            // 低通滤波平滑目标角度
//             float raw_yaw = vision_data.yaw * RAD_TO_DEG;
           float raw_pitch = vision_data.pitch * RAD_TO_DEG;
    
//                 filtered_yaw = filtered_yaw * (1 - FILTER_ALPHA) + raw_yaw * FILTER_ALPHA;
               filtered_pitch = filtered_pitch * (1 - FILTER_ALPHA) + raw_pitch * FILTER_ALPHA;

                //    control_target.gimbal_yaw_angle = filtered_yaw;
                  control_target.gimbal_pitch_angle = filtered_pitch;
                
                   control_target.gimbal_yaw_angle = vision_data.yaw* RAD_TO_DEG;
                //   control_target.gimbal_pitch_angle = vision_data.pitch* RAD_TO_DEG;
                  vision_mode_start_tick = HAL_GetTick();
        }
        else {
            // RC模式下允许遥控器叠加
            if (control_target.control_mode==ControlTarget::MODE_AI) {
                if (HAL_GetTick() - vision_mode_start_tick >= 4000) {
                    control_target.gimbal_yaw_angle-=0.4;
                    control_target.gimbal_pitch_angle = std::sin(2.0 * M_PI * HAL_GetTick() / 500.0) * 12 - 5;
                }
            }
            control_target.gimbal_pitch_angle += dbus.ch[1] * 0.0005f;  // 缩放系数
            control_target.gimbal_yaw_angle -= dbus.ch[0] * 0.0005f;
        }
                control_target.gimbal_yaw_angle_single = remainderf(control_target.gimbal_yaw_angle, 360.0f);

        // ========== 摩擦轮控制 ==========
        if (vision_data.mode==2) {
            if (dbus.s2==1||dbus.s2==3) {
                control_target.friction_ready = true;
            }else {
                control_target.friction_ready = false;
            }
            if ((dbus.s2==1||dbus.s2==3)&&control_target.friction_ready) {
                control_target.feeder_ready= true;
            }else {
                control_target.feeder_ready= false;
            }
        }
        else {
            if (dbus.s2==1||dbus.s2==3) {
                control_target.friction_ready = true;  // 摩擦轮
            } else {
                control_target.friction_ready = false;
            }
            if ((dbus.s2==1&&control_target.friction_ready)||vision_data.mode==2){
                control_target.feeder_ready= true;//拨弹盘
            }else {
                control_target.feeder_ready= false;
            }
        }
        
        yaw_current_out -= control_target.chassis_wz * 1.8f;
        bsp_can2_djimotorcmd(control_output_motor_1, control_output_motor_2,0.,0);
        bsp_can1_djimotorcmdfive2eight(yaw_current_out,control_output_motor_3,0,0);
        bsp_can1_lkmotortorquecmd(pitch_current_out);
        bsp_can1_sendremotecontrolcmd(dbus.ch[2],dbus.ch[3],dbus.ch[4],dbus.s1);
        bsp_can1_sendimudata(yaw,0);
        osDelay(5);  // 1ms卡死
    }

}

#ifdef __cplusplus
extern "C" {
#endif

    static StartControlCenterTask controlcenterTask;

    void StartControlCenterTask_Init() {
        controlcenterTask.start((char*)"StartControlCenterTask", 1024, osPriorityNormal);
    }
#ifdef __cplusplus
}
#endif