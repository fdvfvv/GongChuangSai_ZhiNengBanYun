#ifndef __X42S_H
#define __X42S_H

#include "stm32f4xx_hal.h"

#define X42S_BAUD           115200U
#define X42S_FIXED_CRC      0x6BU

#define X42S_CMD_ENABLE     0xF3U
#define X42S_CMD_SPEED      0xF6U
#define X42S_CMD_POS        0xFDU
#define X42S_CMD_STOP       0xFEU
#define X42S_CMD_SYNC       0xFFU
#define X42S_CMD_RD_POS     0x36U
#define X42S_CMD_RD_SPEED   0x35U
#define X42S_CMD_RD_STATUS  0x3AU
#define X42S_CMD_MOD_ID     0xAEU

#define X42S_RSP_OK         0x02U
#define X42S_DIR_CW         0x00U
#define X42S_DIR_CCW        0x01U
#define X42S_MODE_REL_LAST  0x00U
#define X42S_MODE_ABS       0x01U
#define X42S_MODE_REL_CUR   0x02U
#define X42S_SYNC_NOW       0x00U
#define X42S_SYNC_HOLD      0x01U

#define X42S_MOTOR_ADDR_1   1U
#define X42S_MOTOR_ADDR_2   2U
#define X42S_MOTOR_ADDR_3   3U
#define X42S_MOTOR_ADDR_4   4U
#define X42S_PULSES_PER_REV 3200U
#define X42S_ST_ENABLED     0x01U /* 0x3A bit0: 电机已使能 */

void x42s_init(USART_TypeDef *instance);
uint8_t x42s_enable(uint8_t addr, uint8_t enable, uint8_t sync);
uint8_t x42s_set_speed(uint8_t addr, uint8_t dir, uint16_t rpm,
                       uint8_t acc, uint8_t sync);
uint8_t x42s_move_pulses(uint8_t addr, uint8_t dir, uint16_t rpm,
                         uint8_t acc, uint32_t pulses, uint8_t mode, uint8_t sync);
uint8_t x42s_stop(uint8_t addr, uint8_t sync);
uint8_t x42s_sync_trigger(void);
uint8_t x42s_read_position(uint8_t addr, float *angle);
uint8_t x42s_read_speed(uint8_t addr, int16_t *rpm);
uint8_t x42s_read_status(uint8_t addr, uint8_t *status);
uint8_t x42s_modify_id(uint8_t addr, uint8_t new_id);

#endif
