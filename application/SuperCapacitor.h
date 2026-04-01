#ifndef SUPERCAPACITOR_H
#define SUPERCAPACITOR_H

#include "struct_typedef.h"
#include "can.h"

#define ERROR_UNDER_VOLTAGE 0b00000001
#define ERROR_OVER_VOLTAGE 0b00000010
#define ERROR_BUCK_BOOST 0b00000100
#define ERROR_SHORT_CIRCUIT 0b00001000
#define ERROR_HIGH_TEMPERATURE 0b00010000
#define ERROR_NO_POWER_INPUT 0b00100000
#define ERROR_CAPACITOR 0b01000000

#define Cap_CAN hcan1

#define CAN_SuperCapacitor_ID 0x051


typedef struct __attribute__((packed)) {
    uint8_t enable_dcdc    : 1;  // DCDC输出使能 (1:开启, 0:关闭)
    uint8_t system_restart : 1;  // 系统重启请求 (1:触发)
    uint8_t reserved0      : 6;  // 保留位，凑足1字节
  
    uint16_t referee_power_limit;   // 裁判系统限制功率 (W)
    
    uint16_t referee_energy_buffer; // 裁判系统能量缓冲 (J)

    uint8_t reserved1[3];
} PowerControl_t;

typedef struct __attribute__((packed)) {
    
    uint8_t status_error_code;   // 高位(bit7)表示输出状态，低7位表示错误码

    float chassis_power_f32;     // 当前底盘实际测量功率 (W)

    uint16_t active_power_limit; // 模块当前实际执行的功率限制 (W)

    uint8_t cap_energy_percent;  // 电容剩余能量 (0-255映射0-100%)
} PowerStatus_t;

void Cap_PostStatusFeedback(PowerControl_t Cap_PowerControl);

#endif
