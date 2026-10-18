#include "zdt_can.h"

/* RoboMaster A 板 (STM32F427): CAN1 = PD0(RX) / PD1(TX), AF9
 * PCLK1=16MHz -> 500Kbps: 分频=4(BRP字段=3), TS1=5TQ, TS2=2TQ, SJW=1TQ (总 8TQ)
 * BTR = SJW(25:24) | TS2(22:20) | TS1(19:16) | BRP(9:0) = 0x00140003
 * 历史 bug: 曾误写 0x00140004(分频=5) -> 实际 400K, 电机无 ACK */

/* 诊断: 两个 CAN 各自能否退出初始化(总线隐性), 及最终选定的总线 */
volatile uint32_t g_can1_ready = 0;  /* CAN1(PA11/PA12) 就绪 */
volatile uint32_t g_can2_ready = 0;  /* CAN2(PB5/PB13)  就绪 */
volatile uint32_t g_can_active = 0;  /* 0=无, 1=CAN1, 2=CAN2 */
static CAN_TypeDef *s_can = NULL;    /* 当前使用的 CAN 控制器 */

/* 发一帧扩展帧: 邮箱0, 返回 1=已投递。
 * 超时用硬件计数循环(不依赖 SysTick 中断): 16MHz 下 4000000 次迭代 ≈ 30ms,
 * 避免 uwTick 停走时死循环卡死整个程序。 */
static uint8_t can_tx(CAN_TypeDef *can, uint32_t extid,
                      const uint8_t *data, uint8_t dlc)
{
  for (volatile uint32_t spin = 0U; spin < 800000U; ++spin)
  {
    if ((can->TSR & CAN_TSR_TME0) != 0U) break;
  }
  if ((can->TSR & CAN_TSR_TME0) == 0U) return 0U; /* 邮箱0仍被占用(总线无ACK重传中) */

  can->sTxMailBox[0].TIR = (extid << 3) | (1U << 2); /* IDE=扩展帧 */
  can->sTxMailBox[0].TDTR = (uint32_t)(dlc & 0x0FU);
  can->sTxMailBox[0].TDLR = ((uint32_t)data[0]) | ((uint32_t)data[1] << 8) |
                            ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
  can->sTxMailBox[0].TDHR = ((uint32_t)data[4]) | ((uint32_t)data[5] << 8) |
                            ((uint32_t)data[6] << 16) | ((uint32_t)data[7] << 24);
  can->sTxMailBox[0].TIR |= 1U; /* TXRQ 置起 */
  return 1U;
}

/* 收一帧 (非阻塞), 返回 1=收到 */
static uint8_t can_rx(CAN_TypeDef *can, uint32_t *extid, uint8_t *data,
                      uint8_t *dlc)
{
  uint32_t lo, hi;

  if ((can->RF0R & CAN_RF0R_FMP0_Msk) == 0U) return 0U;

  *extid = can->sFIFOMailBox[0].RIR >> 3;
  *dlc = (uint8_t)(can->sFIFOMailBox[0].RDTR & 0x0FU);
  lo = can->sFIFOMailBox[0].RDLR;
  hi = can->sFIFOMailBox[0].RDHR;
  data[0] = (uint8_t)lo;         data[1] = (uint8_t)(lo >> 8);
  data[2] = (uint8_t)(lo >> 16); data[3] = (uint8_t)(lo >> 24);
  data[4] = (uint8_t)hi;         data[5] = (uint8_t)(hi >> 8);
  data[6] = (uint8_t)(hi >> 16); data[7] = (uint8_t)(hi >> 24);

  can->RF0R |= CAN_RF0R_RFOM0; /* 释放 FIFO0 */
  return 1U;
}

/* 初始化一路 CAN 的 GPIO + 控制器, 尝试退出初始化模式。
 * 总线异常(悬空/接反/短路)时 INAK 永远为 1 -> 返回 0 */
static uint8_t can_init_one(CAN_TypeDef *can, GPIO_TypeDef *port,
                            uint16_t rx_pin, uint16_t tx_pin, uint8_t af,
                            uint32_t btr)
{
  GPIO_InitTypeDef gpio = {0};

  if (can == CAN1) __HAL_RCC_CAN1_CLK_ENABLE();
  else             __HAL_RCC_CAN2_CLK_ENABLE();

  /* 使能 GPIO 时钟 */
  if (port == GPIOA)      __HAL_RCC_GPIOA_CLK_ENABLE();
  else if (port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
  else if (port == GPIOD) __HAL_RCC_GPIOD_CLK_ENABLE();

  gpio.Pin = rx_pin | tx_pin;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_PULLUP;   /* RX 上拉: 悬空时维持隐性, 避免误判显性 */
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = af;
  HAL_GPIO_Init(port, &gpio);

  /* 进初始化模式 */
  can->MCR |= CAN_MCR_INRQ;
  for (volatile uint32_t spin = 0U; spin < 100000U; ++spin) /* ≈1ms @16MHz */
  {
    if ((can->MSR & CAN_MSR_INAK) != 0U) break;
  }

  can->BTR = btr; /* 波特率由调用方指定 */

  /* 自动离线恢复: 总线干扰导致 bus-off 后自动回到总线,
   * 否则 TEC 计满 255 后控制器永久停发, 电机就收不到命令了 */
  can->MCR |= CAN_MCR_ABOM;

  /* 退出睡眠 + 退出初始化(正常模式)。
   * bxCAN 退出初始化需先监测到总线空闲(11个连续隐性位),
   * 总线异常(悬空/接反/短路)时永远出不来 -> 循环重试。 */
  for (uint32_t retry = 0U; retry < 100U; ++retry)
  {
    can->MCR &= ~(CAN_MCR_SLEEP | CAN_MCR_INRQ);
    for (volatile uint32_t spin = 0U; spin < 100000U; ++spin) /* ≈1ms */
    {
      if ((can->MSR & CAN_MSR_INAK) == 0U) break;
    }
    if ((can->MSR & CAN_MSR_INAK) == 0U) return 1U; /* 成功退出 */
    can->MCR |= CAN_MCR_INRQ;                       /* 失败则重来 */
  }
  return 0U;
}

/* 配置一路 CAN 的验收滤波器: 全通, 32位位宽, 掩码模式, FIFO0 */
static void can_filter(CAN_TypeDef *can, uint32_t bank)
{
  CAN1->FMR |= CAN_FMR_FINIT;              /* 滤波器初始化模式 */
  CAN1->FS1R |= (1U << bank);              /* 32-bit scale */
  CAN1->FM1R &= ~(1U << bank);             /* mask mode */
  CAN1->FFA1R &= ~(1U << bank);            /* FIFO0 */
  CAN1->sFilterRegister[bank].FR1 = 0U;    /* ID 全通 */
  CAN1->sFilterRegister[bank].FR2 = 0U;    /* mask 全通 */
  CAN1->FA1R |= (1U << bank);              /* activate */
  CAN1->FMR &= ~CAN_FMR_FINIT;
  (void)can;
}

uint8_t zdt_can_init(uint32_t btr)
{
  /* 只用 CAN1 (A 板 PD0/PD1), 简单直接 */
  g_can1_ready = can_init_one(CAN1, GPIOD, GPIO_PIN_0, GPIO_PIN_1,
                              GPIO_AF9_CAN1, btr);

  if (g_can1_ready)
  {
    can_filter(CAN1, 0U);
    s_can = CAN1;
    g_can_active = 1U;
  }
  else
  {
    s_can = NULL;
    g_can_active = 0U;
  }
  return (s_can != NULL) ? 1U : 0U;
}

/* Emm 命令 -> CAN 帧:
 * ExtId = (addr<<8)|packNum, data[0]=功能码, data[1..]=payload 后续字节 */
uint8_t zdt_can_send_cmd(uint8_t addr, const uint8_t *payload, uint8_t len)
{
  uint8_t data[8] = {0};
  uint8_t pack = 0U;
  uint8_t i = 0U;
  uint8_t remain = (uint8_t)(len - 1U); /* 去掉首字节功能码 */

  if (s_can == NULL) return 0U;

  while (i < remain)
  {
    uint8_t k = (uint8_t)(remain - i);
    uint8_t l;

    data[0] = payload[0];
    if (k < 8U)
    {
      for (l = 0U; l < k; ++l, ++i) { data[l + 1U] = payload[i + 1U]; }
    }
    else
    {
      for (l = 0U; l < 7U; ++l, ++i) { data[l + 1U] = payload[i + 1U]; }
      k = 7U;
    }
    if (!can_tx(s_can, ((uint32_t)addr << 8) | pack, data, (uint8_t)(k + 1U)))
    {
      return 0U;
    }
    ++pack;
  }
  return 1U;
}

/* 等一帧匹配 addr+功能码 的响应, 超时由硬件计数循环控制(不依赖 SysTick) */
static uint8_t can_recv(uint8_t addr, uint8_t code, uint8_t *data,
                        uint8_t *dlc, uint32_t spin_budget)
{
  if (s_can == NULL) return 0U;

  /* 100ms @16MHz ≈ 1.2M 次迭代 */
  for (volatile uint32_t spin = 0U; spin < spin_budget; ++spin)
  {
    uint32_t extid;
    uint8_t buf[8];
    uint8_t n;

    if (can_rx(s_can, &extid, buf, &n))
    {
      if (((extid >> 8) == addr) && (buf[0] == code))
      {
        for (uint8_t j = 0U; j < n && j < 8U; ++j) { data[j] = buf[j]; }
        *dlc = n;
        return 1U;
      }
    }
  }
  return 0U;
}

uint8_t zdt_can_enable(uint8_t addr, uint8_t enable)
{
  const uint8_t payload[] = {ZDT_CMD_ENABLE, 0xABU, enable ? 1U : 0U, 0x00U,
                             ZDT_FIXED_CRC};
  uint8_t data[8];
  uint8_t dlc = 0U;

  if (!zdt_can_send_cmd(addr, payload, sizeof(payload))) return 0U;
  return can_recv(addr, ZDT_CMD_ENABLE, data, &dlc, 300000U);
}

uint8_t zdt_can_set_speed(uint8_t addr, uint8_t dir, uint16_t rpm, uint8_t acc)
{
  const uint8_t payload[] = {ZDT_CMD_SPEED, dir & 1U, (uint8_t)(rpm >> 8),
                             (uint8_t)rpm, acc, 0x00U, ZDT_FIXED_CRC};
  uint8_t data[8];
  uint8_t dlc = 0U;

  if (!zdt_can_send_cmd(addr, payload, sizeof(payload))) return 0U;
  return can_recv(addr, ZDT_CMD_SPEED, data, &dlc, 300000U);
}

uint8_t zdt_can_stop(uint8_t addr)
{
  const uint8_t payload[] = {ZDT_CMD_STOP, 0x98U, 0x00U, ZDT_FIXED_CRC};
  uint8_t data[8];
  uint8_t dlc = 0U;

  if (!zdt_can_send_cmd(addr, payload, sizeof(payload))) return 0U;
  return can_recv(addr, ZDT_CMD_STOP, data, &dlc, 300000U);
}

/* 解除堵转保护 (Emm: addr 0x0E 0x52 0x6B) */
uint8_t zdt_can_clear_clog(uint8_t addr)
{
  const uint8_t payload[] = {ZDT_CMD_CLR_CLOG, 0x52U, ZDT_FIXED_CRC};
  uint8_t data[8];
  uint8_t dlc = 0U;

  if (!zdt_can_send_cmd(addr, payload, sizeof(payload))) return 0U;
  return can_recv(addr, ZDT_CMD_CLR_CLOG, data, &dlc, 300000U);
}

/* 回传: data[0]=0x35, data[1]=符号, data[2..3]=转速大端, data[4]=0x6B */
uint8_t zdt_can_read_speed(uint8_t addr, int16_t *rpm)
{
  const uint8_t payload[] = {ZDT_CMD_RD_SPEED, ZDT_FIXED_CRC};
  uint8_t data[8];
  uint8_t dlc = 0U;
  uint16_t vel;

  if (!zdt_can_send_cmd(addr, payload, sizeof(payload))) return 0U;
  if (!can_recv(addr, ZDT_CMD_RD_SPEED, data, &dlc, 300000U)) return 0U;
  if (dlc < 5U) return 0U;

  vel = (uint16_t)(((uint16_t)data[2] << 8) | data[3]);
  *rpm = (data[1] != 0U) ? -(int16_t)vel : (int16_t)vel;
  return 1U;
}

/* 回传: data[0]=0x3A, data[1]=状态, data[2]=0x6B */
uint8_t zdt_can_read_status(uint8_t addr, uint8_t *status)
{
  const uint8_t payload[] = {ZDT_CMD_RD_STATUS, ZDT_FIXED_CRC};
  uint8_t data[8];
  uint8_t dlc = 0U;

  if (!zdt_can_send_cmd(addr, payload, sizeof(payload))) return 0U;
  if (!can_recv(addr, ZDT_CMD_RD_STATUS, data, &dlc, 300000U)) return 0U;
  if (dlc < 3U) return 0U;

  *status = data[1];
  return 1U;
}
