/**
 * @file    main.c
 * @brief   RoboMaster A 板 + 4×ZDT 闭环步进 + DM-IMU 陀螺仪闭环搬运车
 *
 * 硬件：
 *   CAN1 (PD0/PD1, 500 kbps) : 4 台张大头 ZDT 闭环步进（Emm 固件，扩展帧）
 *   CAN2 (PB12/PB13, 1 Mbps) : DM-IMU-L1 六轴陀螺仪（经典标准帧）
 *   PF14 : 板载绿色 LED
 *   PB2  : 板载 KEY 按键（按下为低）
 *
 * 上电流程：
 *   1. CAN1 初始化，探测 4 台电机，LED 快闪报告在线台数
 *   2. CAN1 使能 + 解除堵转保护
 *   3. CAN2 初始化，配置 DM-IMU 主动上报（欧拉角 + 角速度，100Hz）
 *   4. 等待 KEY 按下 -> 延时 -> 按 g_demo_route 跑轨迹
 *   5. 运行中再按 KEY = 急停；停止后再按 = 重新跑
 *
 * LED：
 *   快闪 N 次 = N 台电机在线 / 状态码
 *   慢闪 = 空闲等待按键
 *   运行中 2Hz 闪 = 正在跑轨迹
 *   故障码 3/4/5 循环闪 = IMU 未找到 / 运行中 IMU 掉线 / 轨迹超时
 */

#include "main.h"
#include "app_config.h"
#include "zdt_can.h"
#include "dm_imu.h"
#include "chassis.h"
#include "trajectory.h"
#include "diag.h"

#include <math.h>

static void MX_GPIO_Init(void);
static void SystemClock_Config(void);

/* ==========================================================================
 * 演示轨迹：0.6 m 正方形，四边直线 + 四个 90° 左转
 *
 * 想跑圆环就把下面整张表换成：
 *   static const traj_segment_t g_demo_route[] =
 *   {
 *       { TRAJ_SEG_ARC, 360.0f, 0.22f, 0.45f },   // 左转一圈，半径 0.45m
 *   };
 * ========================================================================== */
static const traj_segment_t g_demo_route[] =
{
    { TRAJ_SEG_STRAIGHT, 0.60f, 0.22f, 0.0f },
    { TRAJ_SEG_TURN,     90.0f, 50.0f, 0.0f },
    { TRAJ_SEG_STRAIGHT, 0.60f, 0.22f, 0.0f },
    { TRAJ_SEG_TURN,     90.0f, 50.0f, 0.0f },
    { TRAJ_SEG_STRAIGHT, 0.60f, 0.22f, 0.0f },
    { TRAJ_SEG_TURN,     90.0f, 50.0f, 0.0f },
    { TRAJ_SEG_STRAIGHT, 0.60f, 0.22f, 0.0f },
    { TRAJ_SEG_TURN,     90.0f, 50.0f, 0.0f },
    { TRAJ_SEG_WAIT,     0.50f, 0.0f,  0.0f },
};

#define DEMO_ROUTE_COUNT (sizeof(g_demo_route) / sizeof(g_demo_route[0]))

static uint8_t g_runtime_fault;   /* 4=IMU 掉线, 5=轨迹超时 */

/* 与 chassis.c 使用同一组地址宏，改 app_config.h 一处即可 */
static const uint8_t g_motor_addr[4] =
{
    MOTOR_ADDR_FL, MOTOR_ADDR_FR, MOTOR_ADDR_RL, MOTOR_ADDR_RR
};

/* ------------------------------------------------------------------------- */
/* LED / 按键                                                                */
/* ------------------------------------------------------------------------- */

static void led_on(void)
{
    GPIOF->BSRR = GPIO_PIN_14;
}

static void led_off(void)
{
    GPIOF->BSRR = (uint32_t)GPIO_PIN_14 << 16;
}

static void led_blink_count(uint8_t n)
{
    for (uint8_t i = 0U; i < n; ++i)
    {
        led_on();
        HAL_Delay(120U);
        led_off();
        HAL_Delay(180U);
    }
    HAL_Delay(400U);
}

/* 启动阶段的致命错误：无限循环报码，必须复位 */
static void led_report(uint8_t n)
{
    while (1)
    {
        for (uint8_t i = 0U; i < n; ++i)
        {
            led_on();
            HAL_Delay(150U);
            led_off();
            HAL_Delay(150U);
        }
        HAL_Delay(800U);
    }
}

static uint8_t button_raw_pressed(void)
{
    return (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_RESET) ? 1U : 0U;
}

/* 20ms 消抖，返回 1 = 发生一次“按下”事件 */
static uint8_t button_pressed_event(void)
{
    static uint8_t stable = 0U;
    static uint8_t last_raw = 0U;
    static uint32_t last_change = 0U;

    uint8_t raw = button_raw_pressed();
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

static void wait_for_start(void)
{
    uint32_t last = HAL_GetTick();

    while (1)
    {
        dm_imu_poll();

        if (button_pressed_event()) break;

        uint32_t now = HAL_GetTick();
        if ((now - last) >= 500U)
        {
            last = now;
            if ((GPIOF->ODR & GPIO_PIN_14) != 0U) led_off();
            else led_on();
        }
        HAL_Delay(5U);
    }

    led_off();
}

/* ------------------------------------------------------------------------- */
/* 电机                                                                      */
/* ------------------------------------------------------------------------- */

static uint8_t motor_probe(void)
{
    uint8_t status = 0U;
    uint8_t ok = 0U;

    /* 四轮车必须四台都在线，少一台轨迹会跑歪，宁可上电就报错 */
    for (uint8_t round = 0U; round < 10U; ++round)
    {
        ok = 0U;
        for (uint8_t i = 0U; i < 4U; ++i)
        {
            if (zdt_can_read_status(g_motor_addr[i], &status)) ++ok;
        }
        if (ok >= 4U) break;
        HAL_Delay(300U);
    }
    return ok;
}

static void motor_prepare(void)
{
    for (uint8_t i = 0U; i < 4U; ++i)
    {
        (void)zdt_can_clear_clog(g_motor_addr[i]);
        (void)zdt_can_enable(g_motor_addr[i], 1U);
    }
    HAL_Delay(100U);
}

#if APP_WHEEL_TEST_MODE
/* 单轮 30RPM 点动，用来确认“地址 -> 车轮位置”和“正方向”。
 * 建议把车架空再开这个模式。 */
static void wheel_test(void)
{
    for (uint8_t i = 0U; i < 4U; ++i)
    {
        (void)zdt_can_set_speed(g_motor_addr[i], ZDT_DIR_CW, 30U, 20U);
        HAL_Delay(2000U);
        (void)zdt_can_set_speed(g_motor_addr[i], ZDT_DIR_CW, 0U, 20U);
        HAL_Delay(500U);
    }

    while (1)
    {
        led_blink_count(4U);
    }
}
#endif

/* ------------------------------------------------------------------------- */
/* 主程序                                                                    */
/* ------------------------------------------------------------------------- */

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    led_off();

#if APP_DIAG_MODE
    /* 诊断固件：扫描电机地址并逐个点动，不初始化 IMU/不跑轨迹 */
    diag_run();
#endif

    /* 1. CAN1 初始化（500 kbps，4 台 ZDT 电机） */
    if (!zdt_can_init(ZDT_BTR_500K)) led_report(1U);
    HAL_Delay(500U);

    /* 2. 探测电机。四轮必须四台都在线 */
    uint8_t motors_online = motor_probe();
    led_blink_count(motors_online);
    if (motors_online < 4U) led_report(2U);

    /* 3. 使能 + 解除堵转保护 */
    motor_prepare();

    /* 底盘初始化：PID 增益、轮速斜坡、四轮 0 速度。必须在发车前调用 */
    chassis_init();

#if APP_WHEEL_TEST_MODE
    wheel_test();
#endif

    /* 4. DM-IMU：找不到就循环重试，LED 3 快闪 */
    while (!dm_imu_init())
    {
        led_blink_count(3U);
        HAL_Delay(800U);
    }
    led_blink_count(2U);       /* 2 = 电机 + IMU 都就绪 */
    dm_imu_zero_yaw();         /* 上电位置作为 0° */

    /* 5. 等待发车 */
#if APP_REQUIRE_START_BUTTON
    wait_for_start();
#else
    HAL_Delay(2000U);
#endif

    if (!dm_imu_is_online()) led_report(3U);

    HAL_Delay(APP_START_DELAY_MS);

    /* 延时期间 IMU 帧会在 FIFO 里堆积，发车前再收一次并确认在线 */
    dm_imu_poll();
    if (!dm_imu_is_online()) led_report(3U);

    dm_imu_zero_yaw();         /* 发车瞬间再归零，把等待期间的漂移清掉 */
    traj_start(g_demo_route, (uint8_t)DEMO_ROUTE_COUNT);

    uint32_t next_control = HAL_GetTick();
    uint32_t last_control = HAL_GetTick();
    uint32_t next_led = HAL_GetTick();
    uint32_t next_service = HAL_GetTick();

    while (1)
    {
        dm_imu_poll();

        uint32_t now = HAL_GetTick();

        /* ---- 100Hz 控制 ---- */
        if ((int32_t)(now - next_control) >= 0)
        {
            next_control += APP_CONTROL_PERIOD_MS;
            if ((int32_t)(now - next_control) > 100)
            {
                next_control = now + APP_CONTROL_PERIOD_MS;   /* 掉帧后重新对齐 */
            }

            float dt = (float)(now - last_control) * 0.001f;
            last_control = now;
            if (dt <= 0.0f) dt = (float)APP_CONTROL_PERIOD_MS * 0.001f;
            if (dt > 0.05f) dt = 0.05f;

            /* 电机 2 秒自愈：驱动器掉使能/堵转保护后自动恢复。
             * 放在速度命令之前发，保证本周期最后写进去的是新速度。 */
            if (ZDT_SELF_HEAL_MS > 0U &&
                (int32_t)(now - next_service) >= 0)
            {
                next_service = now + ZDT_SELF_HEAL_MS;
                for (uint8_t i = 0U; i < 4U; ++i)
                {
                    (void)zdt_can_clear_clog_nowait(g_motor_addr[i]);
                    (void)zdt_can_enable_nowait(g_motor_addr[i], 1U);
                }
            }

            if (traj_is_running())
            {
                if (!dm_imu_is_online()) traj_abort();
                else traj_update(dt);
            }
            chassis_update(dt);
        }

        /* ---- 运行故障：锁存、闪码、等按键复位 ---- */
        if (g_runtime_fault == 0U)
        {
            if (chassis_faulted())
            {
                g_runtime_fault = 4U;
                traj_abort();
                chassis_stop();
            }
            else if (traj_state() == TRAJ_STATE_FAULT)
            {
                g_runtime_fault = 5U;
                chassis_stop();
            }
        }

        if (g_runtime_fault != 0U)
        {
            /* 4 或 5 次短闪 / 4 秒 */
            uint8_t phase = (uint8_t)((now / 250U) % 16U);
            if (phase < (uint8_t)(g_runtime_fault * 2U) && (phase & 1U) == 0U)
            {
                led_on();
            }
            else
            {
                led_off();
            }

            if (button_pressed_event() && dm_imu_is_online())
            {
                chassis_clear_fault();
                g_runtime_fault = 0U;
                dm_imu_zero_yaw();
                traj_start(g_demo_route, (uint8_t)DEMO_ROUTE_COUNT);
            }

            HAL_Delay(5U);
            continue;
        }

        /* ---- 按键：运行中急停；停止后重新发车 ---- */
        if (button_pressed_event())
        {
            if (traj_is_running())
            {
                traj_abort();
                chassis_stop();
            }
            else if (dm_imu_is_online())
            {
                chassis_clear_fault();
                dm_imu_zero_yaw();
                traj_start(g_demo_route, (uint8_t)DEMO_ROUTE_COUNT);
            }
        }

        /* ---- 心跳灯：运行 2Hz，空闲 1Hz ---- */
        uint32_t period = traj_is_running() ? 250U : 500U;
        if ((now - next_led) >= period)
        {
            next_led = now;
            if ((GPIOF->ODR & GPIO_PIN_14) != 0U) led_off();
            else led_on();
        }

        HAL_Delay(1U);
    }
}

/* ------------------------------------------------------------------------- */
/* 底层初始化                                                                */
/* ------------------------------------------------------------------------- */

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PF14 绿色 LED */
    gpio.Pin = GPIO_PIN_14;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOF, &gpio);
    led_off();

    /* PB2 板载 KEY，按下接地 */
    gpio.Pin = GPIO_PIN_2;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);
}

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /* 与原工程一致：HSI 16MHz，CAN1 的 500K 分频按 PCLK1=16MHz 计算。
     * 如果以后改 HSE/PLL，务必同步修改 zdt_can.h 里的 BTR 和 dm_imu.c 的 BTR。 */
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
