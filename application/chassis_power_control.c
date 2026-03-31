/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_power_control.c/h
  * @brief      chassis power control.底盘功率控制
  * @note       this is only controling 80 w power, mainly limit motor current set.
  *             if power limit is 40w, reduce the value JUDGE_TOTAL_CURRENT_LIMIT 
  *             and POWER_CURRENT_LIMIT, and chassis max speed (include max_vx_speed, min_vx_speed)
  *             只控制80w功率，主要通过控制电机电流设定值,如果限制功率是40w，减少
  *             JUDGE_TOTAL_CURRENT_LIMIT和POWER_CURRENT_LIMIT的值，还有底盘最大速度
  *             (包括max_vx_speed, min_vx_speed)
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "chassis_power_control.h"
#include "referee.h"
#include "arm_math.h"
#include "detect_task.h"

#define POWER_LIMIT         80.0f
#define WARNING_POWER       40.0f   
#define WARNING_POWER_BUFF  50.0f   

#define NO_JUDGE_TOTAL_CURRENT_LIMIT    64000.0f    //16000 * 4, 
#define BUFFER_TOTAL_CURRENT_LIMIT      16000.0f
#define POWER_TOTAL_CURRENT_LIMIT       20000.0f

/**
  * @brief          limit the power, mainly limit motor current
  * @param[in]      chassis_power_control: chassis data 
  * @retval         none
  */
/**
  * @brief          限制功率，主要限制电机电流
  * @param[in]      chassis_power_control: 底盘数据
  * @retval         none
  */
void chassis_power_control(chassis_move_t *chassis_power_control)
{
    fp32 chassis_power = 0.0f;        // 2026 协议中此值通常需通过计算或暂用 limit 代替
    fp32 chassis_power_buffer = 0.0f;
    fp32 total_current_limit = 0.0f;
    fp32 total_current = 0.0f;
    uint8_t robot_id = get_robot_id();

    // 裁判系统掉线检查
    if(toe_is_error(REFEREE_TOE))
    {
        total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
    }
    // 工程机器人或 ID 为 0（未初始化）不限功率
    else if(robot_id == RED_ENGINEER || robot_id == BLUE_ENGINEER || robot_id == 0)
    {
        total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
    }
    else
    {
        /* 核心修复：解决 Error #167 */
        uint16_t temp_power_limit;
        uint16_t temp_buffer;
        
        // 调用我们重构的接口，传入匹配的 uint16_t 指针
        get_chassis_power_and_buffer(&temp_power_limit, &temp_buffer);
        
        // 转换为逻辑使用的 fp32
        // 注意：2026 协议 0x0202 帧不发实时功率。这里 chassis_power 建议赋值为上限 limit
        chassis_power = (fp32)temp_power_limit; 
        chassis_power_buffer = (fp32)temp_buffer;

        // 功率控制算法逻辑（保持原有比例缩放逻辑）
        // 1. 如果缓冲能量低于警告阈值
        if(chassis_power_buffer < WARNING_POWER_BUFF)
        {
            fp32 power_scale;
            if(chassis_power_buffer > 5.0f)
            {
                power_scale = chassis_power_buffer / WARNING_POWER_BUFF;
            }
            else
            {
                power_scale = 5.0f / WARNING_POWER_BUFF;
            }
            total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT * power_scale;
        }
        else
        {
            // 2. 如果当前功率（或上限）超过警告功率
            if(chassis_power > WARNING_POWER)
            {
                fp32 power_scale;
                if(chassis_power < POWER_LIMIT)
                {
                    power_scale = (POWER_LIMIT - chassis_power) / (POWER_LIMIT - WARNING_POWER);
                }
                else
                {
                    power_scale = 0.0f;
                }
                total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT * power_scale;
            }
            else
            {
                total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT;
            }
        }
    }

    // 计算当前四个电机的总设定电流
    total_current = 0.0f;
    for(uint8_t i = 0; i < 4; i++)
    {
        total_current += fabs(chassis_power_control->motor_speed_pid[i].out);
    }

    // 如果总电流超过限制，进行等比例缩放
    if(total_current > total_current_limit && total_current > 0.001f)
    {
        fp32 current_scale = total_current_limit / total_current;
        for(uint8_t i = 0; i < 4; i++)
        {
            chassis_power_control->motor_speed_pid[i].out *= current_scale;
        }
    }
}
