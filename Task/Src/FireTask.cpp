//
// FireTask - 开火控制任务实现
// 负责摩擦轮和拨弹盘的控制
//

#include "FireTask.h"
#include "message_bus.h"
#include "Motor.h"
#include "bsp_can.h"
#include "cmsis_os.h"
#include "speed_pid.h"
#include "angle_pid.h"
#include "stm32f4xx_hal.h"
#include <cstdint>
// 引用全局电机变量（在bsp_can.cpp中定义）
extern DJI_motor_info  motor_2, motor_1, motor_6,motor_5;
extern LK_motor_info LK_motor_1;
int control_output_motor_1 = 0.0f;
int control_output_motor_2 = 0.0f;
int control_output_motor_3 = 0.0f;

void StartFireTask::run() {

    /* 初始化 */

    // 增加基础前馈，减轻 PID 负担
    float feed_forward_left = 1000 * 0.5f;
    float feed_forward_right = -1000 * 0.5f;
    
    //实例化电机PID
    //（kp，ki，kd，maxout，minout,errmax,errmin）
    SpeedPID motor_LEFT_PID(20.0f, 0.11f, 0.09f, 8000.0f, -8000.0f, 250.0f, -250.0f);
    SpeedPID motor_RIGHT_PID(20.0f, 0.11f, 0.09f, 8000.0f, -8000.0f, 250.0f, -250.0f);
    
    // 角度环PID - 用于定角度阶段
    AnglePID motor_SUPPLY_ANGLE_PID(5.0f, 0.5f, 0.0f, 5000.0f, 3000.0f);
    
    // 速度环PID - 用于定速度阶段
    SpeedPID motor_SUPPLY_PID(30.0f, 0.5f, 0.0f, 30000.0f, -30000.0f, 2500.0f, -2500.0f);

    // ====== 拨弹盘控制相关变量 ======
    // 拨盘目标总角度（机械输出轴角度）
    static int32_t feeder_target_angle = 0;
    // 每次feeder_ready为true时增加的目标增量（机械轴角度）
    const int32_t FEEDER_ANGLE_INC = 4000;
    
    // 角度环阶段相关
    const int32_t ANGLE_MODE_THRESHOLD = 5000;  // 角度环认为到位的误差阈值
    const int32_t ANGLE_REACH_TIMEOUT = 200;    // 角度到位判定超时时间(ms)
    
    // 速度环阶段相关
    const int32_t FEEDER_SPEED_TARGET = 4500;   // 拨弹速度目标（定速度模式）
    const int32_t SPEED_JAM_THRESHOLD = 1000;   // 速度环堵转判定速度阈值（速度低于此值认为堵转）
    const uint32_t SPEED_JAM_TIMEOUT = 500;     // 速度环堵转判定超时时间(ms)
    const int32_t SPEED_REVERSE_TIME = 100;    // 反转时间(ms)
    
    // 堵转判定阈值（电机total_angle与目标差值过大）
    const int32_t JAM_THRESHOLD = 500000;
    // 反转距离
    const int32_t REVERSE_DISTANCE = 100000;
    
    // 阶段标志：0-停止, 1-角度环阶段, 2-速度环阶段, 3-反转阶段
    static uint8_t feeder_stage = 0;
    // 角度到位计时
    static uint32_t angle_reach_start_time = 0;
    // 速度环开始计时
    static uint32_t speed_mode_start_time = 0;
    // 反转开始计时
    static uint32_t reverse_start_time = 0;
    // 上次feeder_ready状态
    static bool last_feeder_ready = false;
    // 阶段标志：0正转，1反转
    static uint8_t friction_stage = 0;
    //摩擦轮子未到达目标的持续时间
    static uint32_t motor_1_err_time=0;
    //摩擦轮上一次到达目标的时间
    static uint32_t motor_1_last_ok_time=0;
    // 新增差值PID，用于控制两个摩擦轮转速差为零
    SpeedPID diffPID(15.0f, 0.5f, 0.05f, 300.0f, -300.0f, 150.0f, -150.0f);
    float filtered_diff = 0.0f;
    
    for (;;) {
        if (control_target.friction_ready == true) {
               // ====== 摩擦轮控制 ======
            if (motor_1.rotor_speed>=5000) {
                motor_1_last_ok_time=HAL_GetTick();
            }
            motor_1_err_time=HAL_GetTick()-motor_1_last_ok_time;
            if((motor_1_err_time>1000)&&(motor_1_err_time<2000)){    //此处为堵转判定时间，误差时间过长触发
                friction_stage = 1;
            control_output_motor_1 = feed_forward_left + motor_LEFT_PID.Calculate(-2000, motor_1.rotor_speed, 0.01f) ;
            control_output_motor_2 = feed_forward_right + motor_RIGHT_PID.Calculate(2000, motor_2.rotor_speed, 0.01f) ;
            }else if (motor_1_err_time>2000) {   //err时间超过目标时间则恢复正转
                motor_1_last_ok_time=HAL_GetTick() ;
                friction_stage = 0;
            }
            else {   //1s内达到目标转速则正常触发else
            friction_stage = 0;
            
            int16_t diff = motor_1.rotor_speed + motor_2.rotor_speed;
            // 低通滤波
            filtered_diff = 0.2f * diff + 0.8f * filtered_diff;
            // 差值PID控制，目标为0
            float diff_output = diffPID.Calculate(0.0f, filtered_diff, 0.01f);
            control_output_motor_1 = feed_forward_left + motor_LEFT_PID.Calculate(5800, motor_1.rotor_speed, 0.01f) + diff_output;
            control_output_motor_2 = feed_forward_right + motor_RIGHT_PID.Calculate(-5800, motor_2.rotor_speed, 0.01f) - diff_output;
            }



            // ====== 拨弹盘控制 ======
            bool feeder_should_run = (control_target.feeder_ready == true) && (hot_17mm <= 200)&&(motor_1.rotor_speed>5000);
            
            // 检测feeder_ready从false变true的上升沿（新的发射周期开始）
            if (feeder_should_run && !last_feeder_ready) {
                // 新的发射周期，开始角度环阶段
                feeder_stage = 1;
                // 设置角度目标：在当前位置基础上增加一个拨弹增量
                feeder_target_angle = motor_6.total_angle + FEEDER_ANGLE_INC;
                angle_reach_start_time = HAL_GetTick();
            }
            last_feeder_ready = feeder_should_run;
            
            if (feeder_should_run) {
                int32_t angle_error = feeder_target_angle - motor_6.total_angle;
                if (friction_stage==1) {
                    feeder_stage=3;
                }
                if (feeder_stage == 1) {
                    // ====== 阶段1：角度环模式 ======
                    // 计算与实际角度的差值
                    int32_t angle_diff = feeder_target_angle - motor_6.total_angle;
                    
                    // 差值过大则反转（减少目标角度）
                    if (angle_diff > JAM_THRESHOLD) {
                        feeder_target_angle -= REVERSE_DISTANCE;
                    } else {
                        feeder_target_angle += FEEDER_ANGLE_INC;
                    }
                    
                    // 角度环PID得到速度目标
                    int32_t speed_target = motor_SUPPLY_ANGLE_PID.calculate(feeder_target_angle, motor_6.total_angle, 0.01f);
                    
                    // 检查是否角度到位（误差在阈值内）
                    if (angle_error >= -ANGLE_MODE_THRESHOLD && angle_error <= ANGLE_MODE_THRESHOLD) {
                        // 角度到位，切换到速度环阶段
                        feeder_stage = 2;
                    } else if ((HAL_GetTick() - angle_reach_start_time) > ANGLE_REACH_TIMEOUT) {
                        // 超时也切换到速度环，防止一直卡在角度环
                        feeder_stage = 2;
                    } else {
                        // 角度环PID得到电流输出
                        control_output_motor_3 = motor_SUPPLY_PID.Calculate(speed_target, motor_6.rotor_speed, 0.01f);
                    }
                }
                
                if (feeder_stage == 2) {
                    // ====== 阶段2：速度环模式 ======
                    // 记录进入速度环的起始时间
                    if (speed_mode_start_time == 0) {
                        speed_mode_start_time = HAL_GetTick();
                    }
                    
                    // 检测速度环堵转：速度持续低于阈值超过超时时间
                    bool is_speed_jammed = (motor_6.rotor_speed < SPEED_JAM_THRESHOLD) &&
                                           ((HAL_GetTick() - speed_mode_start_time) > SPEED_JAM_TIMEOUT);
                    
                    if (is_speed_jammed) {
                        // 堵转检测到，切换到反转阶段
                        feeder_stage = 3;
                        reverse_start_time = HAL_GetTick();
                    } else {
                        // 正常速度环控制
                        control_output_motor_3 = motor_SUPPLY_PID.Calculate(FEEDER_SPEED_TARGET, motor_6.rotor_speed, 0.01f);
                    }
                }
                
                if (feeder_stage == 3) {
                    // ====== 阶段3：反转阶段 ======
                    // 给定反转速度，松动卡住的弹丸
                    control_output_motor_3 = motor_SUPPLY_PID.Calculate(-FEEDER_SPEED_TARGET, motor_6.rotor_speed, 0.01f);
                    
                    // 反转一定时间后切回速度环
                    if ((HAL_GetTick() - reverse_start_time) > SPEED_REVERSE_TIME) {
                        feeder_stage = 2;
                        speed_mode_start_time = HAL_GetTick();  // 重置速度环计时
                    }
                }
            } else {
                // 停止发射，复位状态
                feeder_stage = 0;
                feeder_target_angle = motor_6.total_angle;
                speed_mode_start_time = 0;
                reverse_start_time = 0;
                control_output_motor_3 = motor_SUPPLY_PID.Calculate(0, motor_6.rotor_speed, 0.01f);
            }

        } else {
            // 摩擦轮未就绪
            control_output_motor_1 = feed_forward_left + motor_LEFT_PID.Calculate(0, motor_1.rotor_speed, 0.01f);
            control_output_motor_2 = feed_forward_right + motor_RIGHT_PID.Calculate(0, motor_2.rotor_speed, 0.01f);
            control_output_motor_3 = motor_SUPPLY_PID.Calculate(0, motor_6.rotor_speed, 0.01f);
            
            // 重置状态
            feeder_stage = 0;
        }
        osDelay(1);
    }
}
#ifdef __cplusplus
extern "C" {
#endif

    static StartFireTask fireTask;

    void StartFireTask_Init() {
        fireTask.start((char*)"StartFireTask", 512, osPriorityNormal);
    }
#ifdef __cplusplus
}
#endif