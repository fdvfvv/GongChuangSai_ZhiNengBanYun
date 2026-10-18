#ifndef __ZDT_CAN_H
#define __ZDT_CAN_H

#include "stm32f4xx_hal.h"

/* Emm 固件 CAN 通讯协议 (裸寄存器 bxCAN 实现, 不依赖 HAL_CAN):
 * 扩展帧, ExtId = (电机地址 << 8) | 分包序号
 * 数据区: [0]=功能码, [1..]=参数, 末字节 0x6B
 * CAN 波特率 500Kbps (出厂默认, 与官方例程一致)
 * 引脚: RoboMaster A 板 CAN1 = PD0(RX) / PD1(TX), AF9, PCLK1=16MHz
 *       分频=4, TS1=5TQ, TS2=2TQ -> 16M/4/8TQ = 500K
 * 注意: BTR 寄存器 BRP 字段 = 分频系数 - 1, 500K 时 BRP 字段应写 3 */

#define ZDT_CMD_ENABLE    0xF3U
#define ZDT_CMD_SPEED     0xF6U
#define ZDT_CMD_POS       0xFDU
#define ZDT_CMD_STOP      0xFEU
#define ZDT_CMD_RD_SPEED  0x35U
#define ZDT_CMD_RD_STATUS 0x3AU
#define ZDT_CMD_CLR_CLOG  0x0EU
#define ZDT_FIXED_CRC     0x6BU

#define ZDT_DIR_CW        0x00U
#define ZDT_DIR_CCW       0x01U

/* 波特率 BTR 编码 (PCLK1=16MHz, TS1=5TQ, TS2=2TQ, SJW=1TQ, 总 8TQ)
 * 波特率 = 16M / ((BRP字段+1) x 8TQ)  -- BRP 字段值 = 分频系数 - 1 */
#define ZDT_BTR_1M    0x00140001U  /* 分频=2  */
#define ZDT_BTR_500K  0x00140003U  /* 分频=4 -> 16M/4/8 = 500K */
#define ZDT_BTR_250K  0x00140007U  /* 分频=8  */
#define ZDT_BTR_200K  0x00140009U  /* 分频=10 */
#define ZDT_BTR_125K  0x0014000FU  /* 分频=16 */
#define ZDT_BTR_100K  0x00140013U  /* 分频=20 */

#define ZDT_MOTOR_ADDR_1  1U
#define ZDT_MOTOR_ADDR_2  2U
#define ZDT_MOTOR_ADDR_3  3U
#define ZDT_MOTOR_ADDR_4  4U

uint8_t zdt_can_init(uint32_t btr);
uint8_t zdt_can_send_cmd(uint8_t addr, const uint8_t *payload, uint8_t len);
uint8_t zdt_can_enable(uint8_t addr, uint8_t enable);
uint8_t zdt_can_set_speed(uint8_t addr, uint8_t dir, uint16_t rpm, uint8_t acc);
uint8_t zdt_can_stop(uint8_t addr);
uint8_t zdt_can_clear_clog(uint8_t addr);
uint8_t zdt_can_read_speed(uint8_t addr, int16_t *rpm);
uint8_t zdt_can_read_status(uint8_t addr, uint8_t *status);

/* 诊断: 两个 CAN 各自能否退出初始化(总线隐性), 及最终选定的总线 */
extern volatile uint32_t g_can1_ready; /* CAN1(PA11/PA12) 就绪 */
extern volatile uint32_t g_can2_ready; /* CAN2(PB5/PB13)  就绪 */
extern volatile uint32_t g_can_active; /* 0=无, 1=CAN1, 2=CAN2 */

#endif
