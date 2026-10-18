#include "main.h"
#include "zdt_can.h"

/* ============================================================
 * RoboMaster A 板 (STM32F427) + 4 个张大头闭环步进电机 (Emm 固件)
 * CAN1 = PD0(RX)/PD1(TX), 500K, 扩展帧, ExtId=(addr<<8)|pack
 *
 * v5 自愈版: 解决"转着转着停了"
 *   - 开 ABOM 自动离线恢复(总线干扰 bus-off 后自动回总线)
 *   - 每 2 秒全量重发: 解除堵转保护 -> 使能 -> 速度
 *     覆盖电机掉使能/驱动器重启/堵转保护/丢命令等所有停转场景
 *   (v4 修正了 BTR BRP 字段应为分频-1 的波特率 bug, 400K->500K)
 *
 * 流程: CAN 初始化 -> 重试探测 4 台电机(等电机上电启动)
 *       -> LED 报在线台数 -> 使能 + 500RPM 速度模式 -> 心跳
 *
 * LED(PF14) 闪烁编码:
 *   上电后快闪 N 次(1~4) = N 台电机应答(随后电机开始转)
 *   慢闪 1 次循环 = CAN 初始化失败(控制器无法退出初始化)
 *   慢闪 LEC 码循环 = 总线错误: 3=ACK错误(无节点应答/接反),
 *                     1/2/6=Stuff/Form/CRC(波特率不匹配),
 *                     4/5=位错误(线路干扰)
 *   0.5s 心跳 = 命令已发完, 电机应已转动
 * ============================================================ */

/* 运行速度(RPM): 整数, 范围 0~3000, 想改速度只改这一个数 */
#define ZDT_RUN_RPM  1U

static void MX_GPIO_Init(void);
static void SystemClock_Config(void);

/* 无限循环报错误码: 闪 n 次 -> 停 700ms -> 重复 */
static void led_report(uint8_t n)
{
  while (1)
  {
    for (uint8_t i = 0U; i < n; ++i)
    {
      GPIOF->BSRR = GPIO_PIN_14;
      HAL_Delay(150U);
      GPIOF->BSRR = (uint32_t)GPIO_PIN_14 << 16;
      HAL_Delay(150U);
    }
    HAL_Delay(700U);
  }
}

/* 有限次快闪: 报告在线电机台数 */
static void led_blink_count(uint8_t n)
{
  for (uint8_t i = 0U; i < n; ++i)
  {
    GPIOF->BSRR = GPIO_PIN_14;
    HAL_Delay(120U);
    GPIOF->BSRR = (uint32_t)GPIO_PIN_14 << 16;
    HAL_Delay(180U);
  }
  HAL_Delay(500U);
}

int main(void)
{
  uint8_t status = 0U;
  uint8_t ok = 0U;

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();

  /* 1. CAN 初始化 (500K) */
  if (!zdt_can_init(ZDT_BTR_500K))
  {
    led_report(1U); /* 码1: CAN 无法退出初始化 */
  }
  HAL_Delay(500U); /* 等闭环驱动器上电启动 */

  /* 2. 重试探测 4 台电机: 最多 10 轮 x 500ms, 任意一台应答即认为链路通 */
  for (uint8_t round = 0U; round < 10U && ok == 0U; ++round)
  {
    ok = 0U;
    for (uint8_t addr = 1U; addr <= 4U; ++addr)
    {
      if (zdt_can_read_status(addr, &status))
      {
        ++ok;
      }
    }
    if (ok == 0U)
    {
      HAL_Delay(500U);
    }
  }

  if (ok == 0U)
  {
    /* 全部无应答: 读 CAN1->ESR 报精确错误码
     * LEC: 1=Stuff 2=Form 3=ACK 4=Bit1 5=Bit0 6=CRC 7=无错误 */
    volatile uint32_t esr = CAN1->ESR;
    uint32_t tec = (esr >> 16) & 0xFFU;
    uint32_t lec = (esr >> 4) & 0x07U;

    if (lec != 0U && lec != 7U)
    {
      led_report((uint8_t)lec); /* 直接报 LEC 错误码 */
    }
    led_report((tec >= 96U) ? 3U : 2U); /* 3=无ACK, 2=有ACK但无回包 */
  }

  /* 3. 链路通: 快闪报告在线台数 */
  led_blink_count(ok);

  /* 4. 解除堵转保护 + 使能 + 速度模式正转 (速度见 ZDT_RUN_RPM) */
  for (uint8_t addr = 1U; addr <= 4U; ++addr)
  {
    zdt_can_clear_clog(addr);
    zdt_can_enable(addr, 1U);
  }
  HAL_Delay(100U);
  for (uint8_t addr = 1U; addr <= 4U; ++addr)
  {
    zdt_can_set_speed(addr, ZDT_DIR_CW, ZDT_RUN_RPM, 50U);
  }

  /* 5. 心跳 + 每 2 秒自愈重发(解除堵转保护->使能->速度),
   *    电机掉使能/重启/堵转保护/丢命令都会在 2 秒内自动恢复 */
  while (1)
  {
    GPIOF->BSRR = GPIO_PIN_14;
    HAL_Delay(500U);
    GPIOF->BSRR = (uint32_t)GPIO_PIN_14 << 16;
    HAL_Delay(500U);

    static uint8_t tick = 0U;
    if (++tick >= 2U)
    {
      tick = 0U;
      for (uint8_t addr = 1U; addr <= 4U; ++addr)
      {
        zdt_can_clear_clog(addr);
        zdt_can_enable(addr, 1U);
        zdt_can_set_speed(addr, ZDT_DIR_CW, ZDT_RUN_RPM, 50U);
      }
    }
  }
}

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOF_CLK_ENABLE();
  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = GPIO_PIN_14;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &gpio);
  GPIOF->BSRR = (uint32_t)GPIO_PIN_14 << 16;
}

static void SystemClock_Config(void)
{
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  osc.HSIState = RCC_HSI_ON;
  osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  osc.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK) while (1) {}
  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV1;
  clk.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_0) != HAL_OK) while (1) {}
}
