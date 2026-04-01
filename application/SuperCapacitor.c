#include "SuperCapacitor.h"

static CAN_TxHeaderTypeDef Cap_tx_message;

void Cap_PostStatusFeedback(PowerControl_t Cap_PowerControl)
{
    uint32_t Cap_mail_box;
    Cap_tx_message.StdId = CAN_SuperCapacitor_ID;
    Cap_tx_message.IDE = CAN_ID_STD;
    Cap_tx_message.RTR = CAN_RTR_DATA;
    Cap_tx_message.DLC = 0x08;
    HAL_CAN_AddTxMessage(&Cap_CAN, &Cap_tx_message, (uint8_t *)&Cap_PowerControl, &Cap_mail_box);
}

