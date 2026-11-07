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
#include "trace.h"
#include "diag.h"

#include <math.h>

static void MX_GPIO_Init(void);
static void SystemClock_Config(void);

/* ==========================================================================
 * 当前演示轨迹：
 *   前进 20 cm 直线 -> 原地右转 90° -> 再前进 20 cm 直线 -> 停
 *
 * 想跑圆环就把下面整张表换成：
 *   static const traj_segment_t g_demo_route[] =
 *   {
 *       { TRAJ_SEG_ARC, 360.0f, 0.22f, 0.45f },   // 左转一圈，半径 0.45m
 *   };
 * ========================================================================== */
static const traj_segment_t g_demo_route[] =
{
    { TRAJ_SEG_STRAIGHT, 0.20f, 0.18f, 0.0f },   /* 前进 20 cm，0.18 m/s */
    { TRAJ_SEG_TURN,    -90.0f, 60.0f, 0.0f },   /* 右转 90°，60°/s */
    { TRAJ_SEG_STRAIGHT, 0.20f, 0.18f, 0.0f },   /* 再前进 20 cm */
    { TRAJ_SEG_WAIT,      0.30f, 0.0f,  0.0f },  /* 停 0.3 s */
};

#define DEMO_ROUTE_COUNT (sizeof(g_demo_route) / sizeof(g_demo_route[0]))

static uint8_t g_runtime_fault;   /* 4=IMU 掉线, 5=轨迹超时 */

/* 启动阶段诊断：ST-Link 直接读，定位卡在哪一步
 * 0=上电 1=CAN1就绪 2=电机探测完 3=电机使能完 4=IMU就绪 5=等KEY 6=已发车 */
volatile uint8_t g_boot_stage;
volatile uint8_t g_motors_online;

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

#if APP_REQUIRE_START_BUTTON
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
#endif /* APP_REQUIRE_START_BUTTON */

/* 发车前所有等待都必须"边等边收 IMU"。
 * 踩过的坑：直接用 HAL_Delay(2000) 等待时没人调 dm_imu_poll()，
 * IMU 超过 IMU_TIMEOUT_MS(300ms) 就被判离线，后面那道
 * if(!dm_imu_is_online()) led_report(3) 会直接进死循环闪灯，
 * 表现为板子停在 boot_stage=5 永不发车。 */
static void delay_with_imu(uint32_t ms)
{
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < ms)
    {
        dm_imu_poll();
        HAL_Delay(2U);
    }
}


#if APP_REQUIRE_START_BUTTON
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
#endif

/* ------------------------------------------------------------------------- */
/* 电机                                                                      */
/* ------------------------------------------------------------------------- */

/* 单轮探测：扫 1~4 一次，返回应答台数。重试由 main 里“等到 4 台”的外层循环负责。 */
static uint8_t motor_probe(void)
{
    uint8_t status = 0U;
    uint8_t count = 0U;

    for (uint8_t i = 0U; i < 4U; ++i)
    {
        if (zdt_can_read_status(g_motor_addr[i], &status))
        {
            ++count;
        }
    }
    return count;
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

#if APP_MOTION_TEST_MODE
/* 四模式方向标定：每种模式 30RPM 转 2.5 秒，记录 yaw 变化量 x10。
 * 用来判断“哪种四轮组合能让车体真的原地旋转”，以及转向的正负。 */
volatile int16_t g_mtest_dyaw[4];   /* 模式 0..3 的 yaw 变化 x10 */
volatile int16_t g_mtest_rpm[4][4]; /* [模式][电机] 运转中回读的实际转速 */
volatile uint8_t g_mtest_phase;     /* 当前正在跑的相位 */
volatile uint8_t g_mtest_clr[4];    /* 解除堵转保护有没有应答 */
volatile uint8_t g_mtest_en[4];     /* 使能有没有应答 */
volatile uint8_t g_mtest_st[4];     /* 读到的状态字节 */

static void drive_pattern(const int8_t p[4], uint16_t rpm_mag)
{
    /* 测试用：逐台等待应答，确保四台都真的收到（连发会丢后两帧） */
    for (uint8_t i = 0U; i < 4U; ++i)
    {
        uint8_t dir = (p[i] >= 0) ? ZDT_DIR_CW : ZDT_DIR_CCW;
        (void)zdt_can_set_speed(g_motor_addr[i], dir, rpm_mag, ZDT_ACC);
    }
}

/* 按住一个组合 ms 毫秒，期间每 100ms 重发一次命令，
 * 和真实控制环一样靠重发兜住丢帧 */
static void hold_pattern(const int8_t p[4], uint16_t rpm_mag, uint32_t ms)
{
    uint32_t start = HAL_GetTick();
    uint32_t next = start;

    while ((HAL_GetTick() - start) < ms)
    {
        uint32_t now = HAL_GetTick();
        if ((int32_t)(now - next) >= 0)
        {
            next = now + 100U;
            drive_pattern(p, rpm_mag);
        }
        dm_imu_poll();
        HAL_Delay(2U);
    }
}

static void motion_test(void)
{
    /* 顺序 FL/FR/RL/RR */
    static const int8_t pat[4][4] =
    {
        {+1, +1, +1, +1},   /* 0 全正：应直行 */
        {+1, -1, +1, -1},   /* 1 左+右-：理想麦轮应原地右转 */
        {+1, +1, -1, -1},   /* 2 前+后- */
        {+1, -1, -1, +1},   /* 3 对角 */
    };
    const int8_t zero[4] = {0, 0, 0, 0};

    /* 逐台：解除堵转保护 -> 使能 -> 读状态，把应答都记下来 */
    for (uint8_t i = 0U; i < 4U; ++i)
    {
        uint8_t st = 0U;
        g_mtest_clr[i] = zdt_can_clear_clog(g_motor_addr[i]);
        g_mtest_en[i] = zdt_can_enable(g_motor_addr[i], 1U);
        g_mtest_st[i] = zdt_can_read_status(g_motor_addr[i], &st) ? st : 0xFFU;
    }
    delay_with_imu(300U);

    for (uint8_t p = 0U; p < 4U; ++p)
    {
        g_mtest_phase = p;
        float y0 = g_imu.yaw_deg;

        hold_pattern(pat[p], 30U, 2500U);

        /* 运转中回读实际转速：这是判断 nowait 发送到底有没有生效的铁证 */
        for (uint8_t i = 0U; i < 4U; ++i)
        {
            int16_t r = 0;
            if (zdt_can_read_speed(g_motor_addr[i], &r))
            {
                g_mtest_rpm[p][i] = r;
            }
            else
            {
                g_mtest_rpm[p][i] = -32768;
            }
        }

        drive_pattern(zero, 0U);
        delay_with_imu(800U);

        float d = g_imu.yaw_deg - y0;
        while (d > 180.0f)  d -= 360.0f;
        while (d < -180.0f) d += 360.0f;
        g_mtest_dyaw[p] = (int16_t)(d * 10.0f);
    }

    while (1)
    {
        led_blink_count(4U);
    }
}
#endif

#if APP_WHEEL_TEST_MODE
/* 电机链路自检结果，用 ST-Link 读 RAM 即可看到每一步是否通 */
volatile uint8_t g_mt_status_ok[4];   /* 0x3A 读状态有没有回应 */
volatile uint8_t g_mt_status_val[4];  /* 状态字节 */
volatile uint8_t g_mt_enable_ok[4];   /* F3 AB 01 使能有没有回应 */
volatile uint8_t g_mt_read0_ok[4];    /* 运转前读转速是否成功 */
volatile uint8_t g_mt_read1_ok[4];    /* 运转中读转速是否成功 */
volatile int16_t g_mt_rpm_before[4];  /* 运转前实际转速 */
volatile int16_t g_mt_rpm_during[4];  /* 运转中实际转速（关键：不为 0 说明电机真在转） */
volatile uint8_t g_mt_step;           /* 当前测到第几台 */

/* 逐台：读状态 -> 使能 -> 读转速 -> 给 60RPM 转 1 秒 -> 读转速 -> 停。
 * 建议把车架空（轮子离地）再开这个模式。 */
static void wheel_test(void)
{
    for (uint8_t i = 0U; i < 4U; ++i)
    {
        uint8_t st = 0U;
        int16_t rpm = 0;

        g_mt_step = i;
        g_mt_status_ok[i] = zdt_can_read_status(g_motor_addr[i], &st);
        g_mt_status_val[i] = st;
        g_mt_enable_ok[i] = zdt_can_enable(g_motor_addr[i], 1U);

        g_mt_read0_ok[i] = zdt_can_read_speed(g_motor_addr[i], &rpm);
        g_mt_rpm_before[i] = rpm;

        (void)zdt_can_set_speed(g_motor_addr[i], ZDT_DIR_CW, 60U, 10U);
        HAL_Delay(1000U);

        rpm = 0;
        g_mt_read1_ok[i] = zdt_can_read_speed(g_motor_addr[i], &rpm);
        g_mt_rpm_during[i] = rpm;

        (void)zdt_can_set_speed(g_motor_addr[i], ZDT_DIR_CW, 0U, 10U);
        HAL_Delay(300U);
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

#if APP_IMU_MONITOR_MODE
/* 只监控 IMU：记录 yaw 极值、峰值角速度、在线计数，用来单独验证陀螺仪 */
volatile float   g_mon_yaw_min;
volatile float   g_mon_yaw_max;
volatile float   g_mon_rate_max;
volatile uint32_t g_mon_online_cnt;
volatile uint32_t g_mon_loops;

static void imu_monitor(void)
{
    while (1)
    {
        dm_imu_poll();

        float y = g_imu.yaw_deg;
        if (y < g_mon_yaw_min) g_mon_yaw_min = y;
        if (y > g_mon_yaw_max) g_mon_yaw_max = y;

        float r = g_imu.yaw_rate_dps;
        if (r < 0.0f) r = -r;
        if (r > g_mon_rate_max) g_mon_rate_max = r;

        if (dm_imu_is_online()) ++g_mon_online_cnt;
        ++g_mon_loops;

        HAL_Delay(2U);
    }
}
#endif

#if APP_AIR_TEST_MODE
/* 架空观察轮子转向：反复做“四轮同向”和“左右反向”，慢速长时，方便用眼睛看清。
 * 走的是真实底盘通路（含 INVERT 修正和轮转发帧），所以看到的就是跑车时的样子。
 * 阶段 A（LED 闪 1 次）：直行，四轮应同向，轮子底部往车尾方向推。
 * 阶段 B（LED 闪 2 次）：原地右转，左侧两个轮子一个方向、右侧两个反方向。 */
static void air_wheel_test(void)
{
    for (uint8_t round = 0U; round < 3U; ++round)
    {
        /* --- 阶段 A：直行 5 秒 --- */
        led_blink_count(1U);
        delay_with_imu(400U);      /* 闪灯是阻塞的，补收 IMU 帧，否则会被判离线 */
        chassis_clear_fault();
        chassis_set_target(0.10f, 0.0f, 0.0f, 0.0f, 0U, HEADING_MAX_WZ_DPS);
        for (uint16_t k = 0U; k < 500U; ++k)
        {
            dm_imu_poll();
            chassis_update(0.01f);
            HAL_Delay(10U);
        }
        chassis_stop();
        delay_with_imu(2000U);

        /* --- 阶段 B：原地右转 5 秒 --- */
        led_blink_count(2U);
        delay_with_imu(400U);      /* 同上：补收 IMU，避免闪灯期间被判离线 */
        chassis_clear_fault();
        chassis_set_target(0.0f, 0.0f, -40.0f, 0.0f, 0U, HEADING_MAX_WZ_DPS);
        for (uint16_t k = 0U; k < 500U; ++k)
        {
            dm_imu_poll();
            chassis_update(0.01f);
            HAL_Delay(10U);
        }
        chassis_stop();
        delay_with_imu(2000U);
    }

    /* 做完就一直慢闪，方便判断已经结束 */
    while (1)
    {
        led_blink_count(2U);
    }
}
#endif

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    led_off();
    g_boot_stage = 0U;
    g_motors_online = 0U;

    /* 冷上电时 A 板电源轨、CAN 收发器、ZDT 驱动器都比 MCU 慢几秒。
     * 先干等 2.5s，等整机供电稳定，避免“只上电不动、按复位才动”。 */
    HAL_Delay(2500U);

#if APP_DIAG_MODE
    /* 诊断固件：扫描电机地址并逐个点动，不初始化 IMU/不跑轨迹 */
    diag_run();
#endif

    /* 1. 等 CAN1 能退出初始化：冷上电时 CAN 收发器/驱动器供电比 MCU 慢，
     * 第一次 zdt_can_init 会失败。一直重试直到成功，期间 LED 单闪。 */
    while (!zdt_can_init(ZDT_BTR_500K))
    {
        led_blink_count(1U);
        HAL_Delay(400U);
    }
    g_boot_stage = 1U;
    HAL_Delay(500U);

    /* 2. 等四台电机都应答：没凑齐就重试，直到四台都在线，期间 LED 双闪。
     * 这等价于“等系统就绪再自动发车”，不用再手动按复位键。 */
    uint8_t motors_online = 0U;
    while ((motors_online = motor_probe()) < 4U)
    {
        led_blink_count(2U);
        HAL_Delay(400U);
        (void)zdt_can_init(ZDT_BTR_500K);   /* 清掉可能的 bus-off 再试 */
    }
    g_motors_online = motors_online;
    g_boot_stage = 2U;
    led_blink_count(motors_online);   /* 4 快闪 */

    /* 3. 使能 + 解除堵转保护 */
    motor_prepare();
    g_boot_stage = 3U;

    /* 底盘初始化：PID 增益、轮速斜坡、四轮 0 速度。必须在发车前调用 */
    chassis_init();

#if APP_WHEEL_TEST_MODE
    wheel_test();
#endif

#if APP_MOTION_TEST_MODE
    motion_test();
#endif

    /* 4. DM-IMU：找不到就循环重试，LED 3 快闪 */
    while (!dm_imu_init())
    {
        led_blink_count(3U);
        HAL_Delay(800U);
    }
    g_boot_stage = 4U;
    led_blink_count(2U);       /* 2 = 电机 + IMU 都就绪 */
    dm_imu_zero_yaw();         /* 上电位置作为 0° */

#if APP_IMU_MONITOR_MODE
    imu_monitor();             /* 只测陀螺仪，不跑车 */
#endif

#if APP_AIR_TEST_MODE
    air_wheel_test();          /* 架空观察四轮转向，不跑轨迹 */
#endif

    /* 5. 等待发车 */
#if APP_REQUIRE_START_BUTTON
    wait_for_start();
#else
    delay_with_imu(500U);   /* 一上电就走，只留 0.5s 让 IMU 稳定 */
#endif
    g_boot_stage = 5U;

    if (!dm_imu_is_online()) led_report(3U);

    delay_with_imu(APP_START_DELAY_MS);

    /* 延时期间 IMU 帧会在 FIFO 里堆积，发车前再收一次并确认在线 */
    dm_imu_poll();
    if (!dm_imu_is_online()) led_report(3U);

    dm_imu_zero_yaw();         /* 发车瞬间再归零，把等待期间的漂移清掉 */
    motor_prepare();           /* 每次发车前重新解除堵转 + 使能，防止上一轮留下的保护状态 */
    trace_begin();             /* 黑匣子：从发车时刻开始记 100Hz 数据 */
    traj_start(g_demo_route, (uint8_t)DEMO_ROUTE_COUNT);
    g_boot_stage = 6U;

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
            trace_tick();     /* 黑匣子：记录本周期真实下发的轮速与航向 */
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

#if APP_REQUIRE_START_BUTTON
            if (button_pressed_event() && dm_imu_is_online())
            {
                chassis_clear_fault();
                g_runtime_fault = 0U;
                dm_imu_zero_yaw();
                traj_start(g_demo_route, (uint8_t)DEMO_ROUTE_COUNT);
            }
#endif

            HAL_Delay(5U);
            continue;
        }

        /* ---- 按键：运行中急停；停止后重新发车 ---- */
#if APP_REQUIRE_START_BUTTON
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
#endif

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
