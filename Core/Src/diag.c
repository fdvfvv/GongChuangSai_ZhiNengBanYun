/**
 * @file    diag.c
 * @brief   电机 CAN 诊断固件
 *
 * 流程：
 *   1. CAN1 500k 初始化
 *   2. 扫描地址 1~8，LED 快闪报告“有几个地址应答”
 *   3. 慢闪等待 KEY
 *   4. 按 KEY 后，对每个应答的地址：
 *        - LED 闪“地址号”次（地址 1 闪 1 次，地址 2 闪 2 次……）
 *        - 该电机以 20 RPM 转 2 秒，LED 常亮
 *        - 停 0.5 秒
 *   5. 全部跑完，LED 再闪“应答数量”次，等待下一次 KEY
 *
 * 不要求 4 台都在线，适合排查“哪台电机/哪根线有问题”。
 */

#include "diag.h"
#include "app_config.h"
#include "zdt_can.h"
#include "main.h"

/* 你的四台地址是 1~4；只扫 1~4，少地址时不会额外制造无 ACK 帧 */
#define DIAG_ADDR_MAX  4U
#define DIAG_TEST_RPM  20U

/* 给 ST-Link 看诊断结果：bit0~bit7 对应地址 1~8 */
volatile uint16_t g_diag_found_mask;
volatile uint8_t  g_diag_count;
volatile uint8_t  g_diag_scan_done;
volatile uint8_t  g_diag_testing_addr;

static void diag_led_on(void)
{
    GPIOF->BSRR = GPIO_PIN_14;
}

static void diag_led_off(void)
{
    GPIOF->BSRR = (uint32_t)GPIO_PIN_14 << 16;
}

static void diag_blink_count(uint8_t n)
{
    for (uint8_t i = 0U; i < n; ++i)
    {
        diag_led_on();
        HAL_Delay(150U);
        diag_led_off();
        HAL_Delay(200U);
    }
    HAL_Delay(500U);
}

static void diag_fatal(uint8_t n)
{
    while (1)
    {
        diag_blink_count(n);
    }
}

static uint8_t diag_button_pressed(void)
{
    static uint8_t stable = 0U;
    static uint8_t last_raw = 0U;
    static uint32_t last_change = 0U;

    uint8_t raw = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_RESET) ? 1U : 0U;
    uint32_t now = HAL_GetTick();

    if (raw != last_raw)
    {
        last_raw = raw;
        last_change = now;
    }

    if ((now - last_change) > 20U && raw != stable)
    {
        stable = raw;
        if (stable != 0U) return 1U;
    }
    return 0U;
}

static void diag_wait_key(void)
{
    uint32_t last = HAL_GetTick();

    while (1)
    {
        if (diag_button_pressed()) return;

        uint32_t now = HAL_GetTick();
        if ((now - last) >= 500U)
        {
            last = now;
            if ((GPIOF->ODR & GPIO_PIN_14) != 0U) diag_led_off();
            else diag_led_on();
        }
        HAL_Delay(5U);
    }
}

void diag_run(void)
{
    uint8_t found[DIAG_ADDR_MAX + 1U] = {0};
    uint8_t count = 0U;

    diag_led_off();
    g_diag_found_mask = 0U;
    g_diag_count = 0U;
    g_diag_scan_done = 0U;
    g_diag_testing_addr = 0U;

    if (!zdt_can_init(ZDT_BTR_500K))
    {
        diag_fatal(1U);   /* 1 = CAN1 初始化失败 */
    }

    HAL_Delay(500U);

    /* 扫描 1~8，有应答就记下来 */
    for (uint8_t addr = 1U; addr <= DIAG_ADDR_MAX; ++addr)
    {
        uint8_t status = 0U;
        if (zdt_can_read_status(addr, &status))
        {
            found[addr] = 1U;
            g_diag_found_mask |= (uint16_t)(1U << addr);
            ++count;
        }
    }
    g_diag_count = count;
    g_diag_scan_done = 1U;

    /* 先闪“有几个地址应答”，再进入等待 */
    diag_blink_count(count);
    if (count == 0U)
    {
        diag_fatal(2U);   /* 2 = 1~8 号一个都没应答 */
    }

    while (1)
    {
        diag_wait_key();

        for (uint8_t addr = 1U; addr <= DIAG_ADDR_MAX; ++addr)
        {
            if (found[addr] == 0U) continue;

            /* 闪“地址号”次，让操作者知道现在测的是几号 */
            diag_blink_count(addr);
            HAL_Delay(400U);

            g_diag_testing_addr = addr;
            diag_led_on();
            (void)zdt_can_clear_clog(addr);
            (void)zdt_can_enable(addr, 1U);
            (void)zdt_can_set_speed(addr, ZDT_DIR_CW, DIAG_TEST_RPM, 20U);

            HAL_Delay(2000U);

            (void)zdt_can_set_speed(addr, ZDT_DIR_CW, 0U, 20U);
            diag_led_off();
            g_diag_testing_addr = 0U;
            HAL_Delay(500U);
        }

        /* 一轮结束，再闪总数，等待下一次 KEY */
        diag_blink_count(count);
    }
}
