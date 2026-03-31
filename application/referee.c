#include "referee.h"
#include "string.h"
#include "CRC8_CRC16.h"

game_status_t game_state;
robot_status_t robot_state;
power_heat_data_t power_heat;

void init_referee_struct_data(void) {
    memset(&game_state, 0, sizeof(game_status_t));
    memset(&robot_state, 0, sizeof(robot_status_t));
    memset(&power_heat, 0, sizeof(power_heat_data_t));
}

void referee_data_solve(uint8_t *frame) {
    uint16_t cmd_id = 0;
    uint8_t index = 5; 

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += 2; 

    switch (cmd_id) {
        case GAME_STATUS_CMD_ID:
            memcpy(&game_state, frame + index, sizeof(game_status_t));
            break;
        case ROBOT_STATUS_CMD_ID:
            memcpy(&robot_state, frame + index, sizeof(robot_status_t));
            break;
        case POWER_HEAT_DATA_CMD_ID:
            memcpy(&power_heat, frame + index, sizeof(power_heat_data_t));
            break;
        default: break;
    }
}

uint8_t get_robot_id(void) {
    return robot_state.robot_id;
}

/* 兼容性接口实现 */
void get_chassis_power_and_buffer(uint16_t *power_limit, uint16_t *buffer) {
    if (power_limit) *power_limit = robot_state.chassis_power_limit;
    if (buffer) *buffer = power_heat.buffer_energy;
}

void get_shoot_heat0_limit_and_heat0(uint16_t *limit, uint16_t *heat) {
    if (limit) *limit = robot_state.shooter_barrel_heat_limit;
    if (heat) *heat = power_heat.shooter_17mm_barrel_heat;
}
