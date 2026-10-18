#include "x42s.h"

static USART_TypeDef *x42s_uart;

void x42s_init(USART_TypeDef *instance)
{
  x42s_uart = instance;
}

static uint8_t x42s_send(const uint8_t *data, uint8_t length)
{
  if (x42s_uart == NULL)
  {
    return 0U;
  }
  for (uint8_t i = 0U; i < length; ++i)
  {
    uint32_t start = HAL_GetTick();
    while ((x42s_uart->SR & USART_SR_TXE) == 0U)
    {
      if ((HAL_GetTick() - start) > 100U) return 0U;
    }
    x42s_uart->DR = data[i];
  }
  return 1U;
}

static uint8_t x42s_receive(uint8_t *data, uint8_t length)
{
  if (x42s_uart == NULL) return 0U;
  for (uint8_t i = 0U; i < length; ++i)
  {
    uint32_t start = HAL_GetTick();
    while ((x42s_uart->SR & USART_SR_RXNE) == 0U)
    {
      if ((HAL_GetTick() - start) > 50U) return 0U;
    }
    data[i] = (uint8_t)x42s_uart->DR;
  }
  return data[length - 1U] == X42S_FIXED_CRC;
}

/* Emm 响应帧长不固定(读状态4B/读转速6B等): 收首字节后 50ms 静默判帧结束 */
static uint8_t x42s_capture(uint8_t *data, uint8_t max)
{
  uint8_t n = 0;
  uint32_t t0, last;

  if (x42s_uart == NULL) return 0U;
  t0 = HAL_GetTick();
  while ((x42s_uart->SR & USART_SR_RXNE) == 0U)
  {
    if ((HAL_GetTick() - t0) > 100U) return 0U;
  }
  last = HAL_GetTick();
  while ((HAL_GetTick() - last) < 50U)
  {
    if ((x42s_uart->SR & USART_SR_RXNE) != 0U)
    {
      if (n < max) { data[n] = (uint8_t)x42s_uart->DR; }
      else { (void)x42s_uart->DR; }
      ++n;
      last = HAL_GetTick();
    }
  }
  return n;
}

static uint8_t x42s_command(uint8_t addr, uint8_t code,
                            const uint8_t *payload, uint8_t payload_len)
{
  uint8_t frame[16];
  uint8_t response[4];
  uint8_t length = 0U;

  frame[length++] = addr;
  frame[length++] = code;
  for (uint8_t i = 0U; i < payload_len; ++i)
  {
    frame[length++] = payload[i];
  }
  frame[length++] = X42S_FIXED_CRC;

  if (!x42s_send(frame, length) ||
      !x42s_receive(response, sizeof(response)))
  {
    return 0U;
  }
  /* Emm 协议命令回传: 地址 + 功能码 + 命令状态 + 0x6B。
   * 状态字不是固定值(含使能/到位/堵转等标志位)，只校验地址与功能码。 */
  return response[0] == addr && response[1] == code;
}

uint8_t x42s_enable(uint8_t addr, uint8_t enable, uint8_t sync)
{
  const uint8_t payload[] = {0xABU, enable ? 1U : 0U, sync & 1U};
  return x42s_command(addr, X42S_CMD_ENABLE, payload, sizeof(payload));
}

uint8_t x42s_set_speed(uint8_t addr, uint8_t dir, uint16_t rpm,
                       uint8_t acc, uint8_t sync)
{
  const uint8_t payload[] = {
    dir & 1U, (uint8_t)(rpm >> 8), (uint8_t)rpm, acc, sync & 1U
  };
  return x42s_command(addr, X42S_CMD_SPEED, payload, sizeof(payload));
}

uint8_t x42s_move_pulses(uint8_t addr, uint8_t dir, uint16_t rpm,
                         uint8_t acc, uint32_t pulses, uint8_t mode, uint8_t sync)
{
  const uint8_t payload[] = {
    dir & 1U, (uint8_t)(rpm >> 8), (uint8_t)rpm, acc,
    (uint8_t)(pulses >> 24), (uint8_t)(pulses >> 16),
    (uint8_t)(pulses >> 8), (uint8_t)pulses, mode & 3U, sync & 1U
  };
  return x42s_command(addr, X42S_CMD_POS, payload, sizeof(payload));
}

uint8_t x42s_stop(uint8_t addr, uint8_t sync)
{
  const uint8_t payload[] = {0x98U, sync & 1U};
  return x42s_command(addr, X42S_CMD_STOP, payload, sizeof(payload));
}

uint8_t x42s_sync_trigger(void)
{
  const uint8_t frame[] = {0x00U, X42S_CMD_SYNC, 0x66U, X42S_FIXED_CRC};
  uint8_t response[4];

  if (!x42s_send(frame, sizeof(frame)))
  {
    return 0U;
  }
  return x42s_receive(response, sizeof(response));
}

uint8_t x42s_read_position(uint8_t addr, float *angle)
{
  const uint8_t frame[] = {addr, X42S_CMD_RD_POS, X42S_FIXED_CRC};
  uint8_t response[16];
  uint8_t n;
  uint32_t position;

  if (!x42s_send(frame, sizeof(frame)))
  {
    return 0U;
  }
  n = x42s_capture(response, sizeof(response));
  if (n < 8U || response[0] != addr || response[1] != X42S_CMD_RD_POS)
  {
    return 0U;
  }
  position = ((uint32_t)response[3] << 24) |
             ((uint32_t)response[4] << 16) |
             ((uint32_t)response[5] << 8) | response[6];
  *angle = (float)(uint16_t)position * 360.0F / 65536.0F;
  return 1U;
}

/* 修改电机地址(备用, 默认不调用)。
 * 警告: 总线上若有多个电机同为 addr, 这条命令会把它们全改成 id!
 * 改地址必须一次只接一个电机上电单独改。 */
uint8_t x42s_modify_id(uint8_t addr, uint8_t new_id)
{
  const uint8_t payload[] = {0x4BU, 1U, new_id}; /* 0x4B 辅助码 + 存储标志 + 新ID */
  return x42s_command(addr, X42S_CMD_MOD_ID, payload, sizeof(payload));
}

uint8_t x42s_read_speed(uint8_t addr, int16_t *rpm)
{
  const uint8_t frame[] = {addr, X42S_CMD_RD_SPEED, X42S_FIXED_CRC};
  uint8_t response[16];
  uint8_t n;
  int16_t speed;

  /* 回传: 地址 + 0x35 + 状态 + 转速(2B, RPM, 有符号) + 0x6B, 共 6 字节 */
  if (!x42s_send(frame, sizeof(frame)))
  {
    return 0U;
  }
  n = x42s_capture(response, sizeof(response));
  if (n < 5U || response[0] != addr || response[1] != X42S_CMD_RD_SPEED)
  {
    return 0U;
  }
  speed = (int16_t)(((uint16_t)response[3] << 8) | response[4]);
  *rpm = speed;
  return 1U;
}

uint8_t x42s_read_status(uint8_t addr, uint8_t *status)
{
  const uint8_t frame[] = {addr, X42S_CMD_RD_STATUS, X42S_FIXED_CRC};
  uint8_t response[4];

  if (!x42s_send(frame, sizeof(frame)) ||
      !x42s_receive(response, sizeof(response)) ||
      response[0] != addr || response[1] != X42S_CMD_RD_STATUS)
  {
    return 0U;
  }
  *status = response[2];
  return 1U;
}
