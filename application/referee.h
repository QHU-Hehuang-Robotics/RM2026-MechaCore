#ifndef REFEREE_H
#define REFEREE_H

#include "main.h"
#include "protocol.h"

/* 机器人 ID 定义 - 解决 chassis_power_control.c 的 Error #20 */
typedef enum {
    RED_HERO = 1,
    RED_ENGINEER = 2,
    RED_STANDARD_1 = 3,
    RED_STANDARD_2 = 4,
    RED_STANDARD_3 = 5,
    RED_AERIAL = 6,
    RED_SENTRY = 7,
    BLUE_HERO = 11,
    BLUE_ENGINEER = 12,
    BLUE_STANDARD_1 = 13,
    BLUE_STANDARD_2 = 14,
    BLUE_STANDARD_3 = 15,
    BLUE_AERIAL = 16,
    BLUE_SENTRY = 17,
} robot_id_t;

/* 2026 协议命令码 */
#define GAME_STATUS_CMD_ID                  0x0001
#define GAME_ROBOT_HP_CMD_ID                0x0003
#define ROBOT_STATUS_CMD_ID                 0x0201
#define POWER_HEAT_DATA_CMD_ID              0x0202
#define SHOOT_DATA_CMD_ID                   0x0207
#define STUDENT_INTERACTIVE_DATA_CMD_ID     0x0301

/* 2026 协议结构体 */
typedef __packed struct {
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} game_status_t;

typedef __packed struct {
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
} robot_status_t;

typedef __packed struct {
    uint16_t reserved1;
    uint16_t reserved2;
    float reserved3;
    uint16_t buffer_energy;
    uint16_t shooter_17mm_barrel_heat;
    uint16_t shooter_42mm_barrel_heat;
} power_heat_data_t;

/* 接口函数 */
extern void init_referee_struct_data(void);
extern void referee_data_solve(uint8_t *frame);
extern uint8_t get_robot_id(void);

/* 兼容性接口声明 - 解决 Error #167 和 shoot.c 报错 */
extern void get_chassis_power_and_buffer(uint16_t *power_limit, uint16_t *buffer);
extern void get_shoot_heat0_limit_and_heat0(uint16_t *limit, uint16_t *heat);

#endif
