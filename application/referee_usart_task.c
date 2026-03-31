#include "referee_usart_task.h"
#include "fifo.h"
#include "CRC8_CRC16.h"
#include "referee.h"
#include "protocol.h"
#include "cmsis_os.h"

static unpack_data_t referee_unpack_obj;
fifo_s_t referee_fifo;
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
static void referee_unpack_fifo_data(void);

void referee_usart_task(void const * argument) {
    init_referee_struct_data();
    fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);
    // 串口初始化逻辑 (保持原样)
    
    while(1) {
        referee_unpack_fifo_data();
        osDelay(1); // 提高处理频率，应对 2026 高频数据
    }
}

static void referee_unpack_fifo_data(void) {
    uint8_t byte = 0;
    unpack_data_t *p_obj = &referee_unpack_obj;

    while (fifo_s_used(&referee_fifo)) {
        byte = fifo_s_get(&referee_fifo);
        switch(p_obj->unpack_step) {
            case STEP_HEADER_SOF:
                if(byte == 0xA5) {
                    p_obj->unpack_step = STEP_LENGTH_LOW;
                    p_obj->protocol_packet[p_obj->index++] = byte;
                } else p_obj->index = 0;
                break;
            
            case STEP_LENGTH_LOW:
                p_obj->data_len = byte;
                p_obj->protocol_packet[p_obj->index++] = byte;
                p_obj->unpack_step = STEP_LENGTH_HIGH;
                break;
            
            case STEP_LENGTH_HIGH:
                p_obj->data_len |= (byte << 8);
                p_obj->protocol_packet[p_obj->index++] = byte;
                // 2026 协议最大帧长检查
                if(p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - 9)) p_obj->unpack_step = STEP_FRAME_SEQ;
                else { p_obj->unpack_step = STEP_HEADER_SOF; p_obj->index = 0; }
                break;

            case STEP_FRAME_SEQ:
                p_obj->protocol_packet[p_obj->index++] = byte;
                p_obj->unpack_step = STEP_HEADER_CRC8;
                break;

            case STEP_HEADER_CRC8:
                p_obj->protocol_packet[p_obj->index++] = byte;
                if (verify_CRC8_check_sum(p_obj->protocol_packet, 5)) { // 校验帧头
                    p_obj->unpack_step = STEP_DATA_CRC16;
                } else { p_obj->unpack_step = STEP_HEADER_SOF; p_obj->index = 0; }
                break;

            case STEP_DATA_CRC16:
                if (p_obj->index < (p_obj->data_len + 9)) {
                    p_obj->protocol_packet[p_obj->index++] = byte;
                }
                if (p_obj->index >= (p_obj->data_len + 9)) {
                    // 整包 CRC16 校验
                    if (verify_CRC16_check_sum(p_obj->protocol_packet, p_obj->index)) {
                        referee_data_solve(p_obj->protocol_packet);
                    }
                    p_obj->unpack_step = STEP_HEADER_SOF;
                    p_obj->index = 0;
                }
                break;
            default: p_obj->unpack_step = STEP_HEADER_SOF; p_obj->index = 0; break;
        }
    }
}
