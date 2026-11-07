/**
 * @file    dm_imu.c
 * @brief   DM-IMU-L1 驱动：CAN2 (PB12/PB13) + bxCAN 裸寄存器实现
 *
 * 为什么不用 HAL_CAN：
 *   - 原工程没有开 CAN HAL 模块，保持零依赖；
 *   - bxCAN 的 CAN1 已经被电机占用，这里只操作 CAN2；
 *   - 滤波器是 CAN1/CAN2 共享的 28 个 bank，CAN2 用 bank 14 之后。
 */

#include "dm_imu.h"
#include "app_config.h"

#include <math.h>

/* PCLK1 = 16 MHz（本工程 HSI 16MHz 直连），TS1=5TQ, TS2=2TQ, SJW=1TQ，总 8TQ
 * 1 Mbps : 分频 2 -> BTR = 0x00140001
 * 500 kbps: 分频 4 -> BTR = 0x00140003 */
#define DM_IMU_BTR_1M     0x00140001U
#define DM_IMU_BTR_500K   0x00140003U

#define DM_IMU_REG_GYRO_DATA       2U
#define DM_IMU_REG_EULER_DATA      3U
#define DM_IMU_REG_SET_ZERO         5U
#define DM_IMU_REG_SET_DELAY        10U
#define DM_IMU_REG_CHANGE_ACTIVE    11U
#define DM_IMU_REG_DATA_OUTPUT      15U
#define DM_IMU_REG_SAVE_PARAM       254U

#define DM_IMU_OUT_EULER_GYRO       0x06U   /* bit1 欧拉角 + bit2 角速度 */
#define DM_IMU_ACTIVE_DELAY_MS      10U     /* 100 Hz 主动上报 */

#define DM_IMU_CAN2_FILTER_BANK     14U     /* CAN2 从 bank 14 开始 */

dm_imu_t g_imu;

static float s_yaw_prev;
static float s_yaw_cont;
static float s_yaw_offset;
static float s_rate_lpf;
static uint8_t s_have_yaw;

/* ------------------------------------------------------------------------- */
/* 低层 bxCAN2                                                               */
/* ------------------------------------------------------------------------- */

static uint8_t can2_enter_init(void)
{
    CAN2->MCR &= ~CAN_MCR_SLEEP;
    CAN2->MCR |= CAN_MCR_INRQ;
    for (volatile uint32_t spin = 0U; spin < 100000U; ++spin)
    {
        if ((CAN2->MSR & CAN_MSR_INAK) != 0U) return 1U;
    }
    return 0U;
}

static uint8_t can2_leave_init(void)
{
    for (uint32_t retry = 0U; retry < 100U; ++retry)
    {
        CAN2->MCR &= ~(CAN_MCR_SLEEP | CAN_MCR_INRQ);
        for (volatile uint32_t spin = 0U; spin < 100000U; ++spin)
        {
            if ((CAN2->MSR & CAN_MSR_INAK) == 0U) return 1U;
        }
        CAN2->MCR |= CAN_MCR_INRQ;
    }
    return 0U;
}

/* CAN2 滤波器：bank 14，32 位掩码模式，全部通过，进 FIFO0。
 * 只在 CAN2 上挂 IMU，软件里再按标准 ID 过滤，简单可靠。 */
static void can2_filter_init(void)
{
    __HAL_RCC_CAN1_CLK_ENABLE();   /* FMR/FS1R/FA1R 在 CAN1 的寄存器空间 */

    CAN1->FMR |= CAN_FMR_FINIT;
    CAN1->FMR = (CAN1->FMR & ~CAN_FMR_CAN2SB) |
                (DM_IMU_CAN2_FILTER_BANK << CAN_FMR_CAN2SB_Pos);

    CAN1->FA1R &= ~(1U << DM_IMU_CAN2_FILTER_BANK);
    CAN1->FS1R |= (1U << DM_IMU_CAN2_FILTER_BANK);   /* 32-bit */
    CAN1->FM1R &= ~(1U << DM_IMU_CAN2_FILTER_BANK);  /* mask mode */
    CAN1->FFA1R &= ~(1U << DM_IMU_CAN2_FILTER_BANK); /* FIFO0 */
    CAN1->sFilterRegister[DM_IMU_CAN2_FILTER_BANK].FR1 = 0U;
    CAN1->sFilterRegister[DM_IMU_CAN2_FILTER_BANK].FR2 = 0U;
    CAN1->FA1R |= (1U << DM_IMU_CAN2_FILTER_BANK);

    CAN1->FMR &= ~CAN_FMR_FINIT;
}

static uint8_t can2_init(uint32_t btr)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_CAN2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* A 板 CAN2: PB12 = RX, PB13 = TX, AF9 */
    gpio.Pin = GPIO_PIN_12 | GPIO_PIN_13;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;   /* RX 上拉，悬空时保持隐性 */
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &gpio);

    if (!can2_enter_init()) return 0U;

    CAN2->BTR = btr;
    /* ABOM 自动恢复；NART 禁止无限重传，IMU 没接时也不会把 CAN2 拖进 bus-off */
    CAN2->MCR |= CAN_MCR_ABOM | CAN_MCR_NART;

    if (!can2_leave_init()) return 0U;

    can2_filter_init();
    return 1U;
}

/* 发一帧标准数据帧，返回 1 = 已投递。超时用短自旋，避免拖慢控制循环。 */
static uint8_t can2_tx_std(uint16_t std_id, const uint8_t *data, uint8_t dlc)
{
    for (volatile uint32_t spin = 0U; spin < 20000U; ++spin)
    {
        if ((CAN2->TSR & CAN_TSR_TME0) != 0U) break;
    }
    if ((CAN2->TSR & CAN_TSR_TME0) == 0U) return 0U;

    CAN2->sTxMailBox[0].TIR = ((uint32_t)std_id << 21);   /* 标准帧, IDE=0 */
    CAN2->sTxMailBox[0].TDTR = (uint32_t)(dlc & 0x0FU);
    CAN2->sTxMailBox[0].TDLR = ((uint32_t)data[0]) |
                               ((uint32_t)data[1] << 8) |
                               ((uint32_t)data[2] << 16) |
                               ((uint32_t)data[3] << 24);
    CAN2->sTxMailBox[0].TDHR = ((uint32_t)data[4]) |
                               ((uint32_t)data[5] << 8) |
                               ((uint32_t)data[6] << 16) |
                               ((uint32_t)data[7] << 24);
    CAN2->sTxMailBox[0].TIR |= 1U;   /* TXRQ */
    return 1U;
}

/* ------------------------------------------------------------------------- */
/* IMU 命令                                                                  */
/* ------------------------------------------------------------------------- */

/* 命令帧：CC reg ac DD data[4]（数据小端），标准 ID = 从机 ID */
static void imu_write_reg(uint8_t reg, uint32_t value)
{
    uint8_t buf[8];
    buf[0] = 0xCCU;
    buf[1] = reg;
    buf[2] = 0x01U;   /* write */
    buf[3] = 0xDDU;
    buf[4] = (uint8_t)(value);
    buf[5] = (uint8_t)(value >> 8);
    buf[6] = (uint8_t)(value >> 16);
    buf[7] = (uint8_t)(value >> 24);
    (void)can2_tx_std(IMU_SLAVE_ID, buf, 8U);
}

/* 请求单帧数据：CC reg 00 DD 00 00 00 00 */
static void imu_read_reg(uint8_t reg)
{
    uint8_t buf[8] = {0xCCU, reg, 0x00U, 0xDDU, 0U, 0U, 0U, 0U};
    (void)can2_tx_std(IMU_SLAVE_ID, buf, 8U);
}

static void imu_send_active_config(void)
{
    /* 不主动改通信接口/波特率：能收到数据说明已经在这条 CAN 上，
     * 改 COM/BAUD 反而可能让设备重启或失联。只设上报周期、输出类型、主动模式。 */
    imu_write_reg(DM_IMU_REG_SET_DELAY, DM_IMU_ACTIVE_DELAY_MS);
    HAL_Delay(5U);
    imu_write_reg(DM_IMU_REG_DATA_OUTPUT, DM_IMU_OUT_EULER_GYRO);
    HAL_Delay(5U);
    imu_write_reg(DM_IMU_REG_CHANGE_ACTIVE, 1U);
    HAL_Delay(10U);

#if IMU_SAVE_CONFIG
    imu_write_reg(DM_IMU_REG_SAVE_PARAM, 0U);
    HAL_Delay(20U);
#endif
}

/* ------------------------------------------------------------------------- */
/* 数据解析                                                                  */
/* ------------------------------------------------------------------------- */

static float u16_to_float(uint16_t raw, float min, float max)
{
    return ((float)raw) * (max - min) / 65535.0f + min;
}

static void imu_update_euler(const uint8_t *d)
{
    uint16_t pitch_raw = (uint16_t)((uint16_t)d[2] | ((uint16_t)d[3] << 8));
    uint16_t yaw_raw   = (uint16_t)((uint16_t)d[4] | ((uint16_t)d[5] << 8));
    uint16_t roll_raw  = (uint16_t)((uint16_t)d[6] | ((uint16_t)d[7] << 8));

    g_imu.pitch_deg = u16_to_float(pitch_raw, -90.0f, 90.0f);
    g_imu.roll_deg  = u16_to_float(roll_raw, -180.0f, 180.0f);

    float yaw = u16_to_float(yaw_raw, -180.0f, 180.0f);

    if (!s_have_yaw)
    {
        s_yaw_prev = yaw;
        s_yaw_cont = yaw;
        s_yaw_offset = yaw;
        s_have_yaw = 1U;
    }
    else
    {
        float delta = yaw - s_yaw_prev;
        if (delta > 180.0f)       delta -= 360.0f;
        else if (delta < -180.0f) delta += 360.0f;
        s_yaw_cont += delta;
        s_yaw_prev = yaw;
    }

    g_imu.yaw_deg = IMU_YAW_SIGN * (s_yaw_cont - s_yaw_offset);
    g_imu.last_yaw_ms = HAL_GetTick();
}

static void imu_update_gyro(const uint8_t *d)
{
    uint16_t gx_raw = (uint16_t)((uint16_t)d[2] | ((uint16_t)d[3] << 8));
    uint16_t gy_raw = (uint16_t)((uint16_t)d[4] | ((uint16_t)d[5] << 8));
    uint16_t gz_raw = (uint16_t)((uint16_t)d[6] | ((uint16_t)d[7] << 8));

    (void)u16_to_float(gx_raw, -34.88f, 34.88f);
    (void)u16_to_float(gy_raw, -34.88f, 34.88f);
    float gz = u16_to_float(gz_raw, -34.88f, 34.88f);   /* rad/s */

    float rate_dps = gz * 57.2957795f * IMU_YAW_SIGN;
    s_rate_lpf += IMU_RATE_LPF_ALPHA * (rate_dps - s_rate_lpf);
    g_imu.yaw_rate_dps = s_rate_lpf;
}

static void imu_update_accel(const uint8_t *d)
{
    uint16_t ax_raw = (uint16_t)((uint16_t)d[2] | ((uint16_t)d[3] << 8));
    uint16_t ay_raw = (uint16_t)((uint16_t)d[4] | ((uint16_t)d[5] << 8));
    uint16_t az_raw = (uint16_t)((uint16_t)d[6] | ((uint16_t)d[7] << 8));

    g_imu.temperature_c = (float)d[1];
    g_imu.accel_mps2[0] = u16_to_float(ax_raw, -235.2f, 235.2f);
    g_imu.accel_mps2[1] = u16_to_float(ay_raw, -235.2f, 235.2f);
    g_imu.accel_mps2[2] = u16_to_float(az_raw, -235.2f, 235.2f);
}

/* ------------------------------------------------------------------------- */
/* 公共接口                                                                  */
/* ------------------------------------------------------------------------- */

uint8_t dm_imu_init(void)
{
#if IMU_CAN_BAUD_1M
    const uint32_t bauds[2] = {DM_IMU_BTR_1M, DM_IMU_BTR_500K};
#else
    const uint32_t bauds[2] = {DM_IMU_BTR_500K, DM_IMU_BTR_1M};
#endif
    uint8_t tries = IMU_AUTO_BAUD ? 2U : 1U;

    g_imu.init_retry_count++;
    g_imu.online = 0U;
    g_imu.configured = 0U;
    g_imu.rx_count = 0U;
    g_imu.last_rx_ms = 0U;
    g_imu.last_yaw_ms = 0U;
    g_imu.yaw_deg = 0.0f;
    g_imu.yaw_rate_dps = 0.0f;
    s_have_yaw = 0U;
    s_yaw_prev = 0.0f;
    s_yaw_cont = 0.0f;
    s_yaw_offset = 0.0f;
    s_rate_lpf = 0.0f;

    for (uint8_t i = 0U; i < tries; ++i)
    {
        if (!can2_init(bauds[i]))
        {
            continue;
        }

        HAL_Delay(50U);          /* 等 IMU 上电/总线稳定 */
        imu_send_active_config();

        uint32_t start = HAL_GetTick();
        while ((HAL_GetTick() - start) < 500U)
        {
            dm_imu_poll();
            if (g_imu.rx_count > 0U && g_imu.last_yaw_ms != 0U)
            {
                g_imu.configured = 1U;
                g_imu.baud_index = i;
                g_imu.online = 1U;
                return 1U;
            }
            HAL_Delay(2U);
        }

        /* 有些固件/配置下“主动上报”不生效，但请求-应答一定可用。
         * 这里退回 100Hz 请求欧拉角 + 角速度，避免一直卡在 LED 3。 */
        uint32_t req_start = HAL_GetTick();
        while ((HAL_GetTick() - req_start) < 600U)
        {
            imu_read_reg(DM_IMU_REG_EULER_DATA);
            imu_read_reg(DM_IMU_REG_GYRO_DATA);
            for (uint8_t k = 0U; k < 5U; ++k)
            {
                dm_imu_poll();
                HAL_Delay(2U);
            }
            if (g_imu.last_yaw_ms != 0U)
            {
                g_imu.configured = 2U;   /* 2 = 请求-应答模式 */
                g_imu.baud_index = i;
                g_imu.online = 1U;
                return 1U;
            }
        }
    }

    /* 没找到也把 CAN2 留在首选波特率，主循环可以继续重试 */
    (void)can2_init(bauds[0]);
    imu_send_active_config();
    g_imu.baud_index = 0U;
    return 0U;
}

void dm_imu_poll(void)
{
    /* 请求-应答模式：每 10ms 主动要一帧欧拉角 + 角速度。
     * 主动上报模式(configured==1)不额外发请求。 */
    if (g_imu.configured == 2U)
    {
        static uint32_t last_req_ms;
        uint32_t now = HAL_GetTick();
        if ((now - last_req_ms) >= 10U)
        {
            last_req_ms = now;
            imu_read_reg(DM_IMU_REG_EULER_DATA);
            imu_read_reg(DM_IMU_REG_GYRO_DATA);
        }
    }

    for (uint8_t n = 0U; n < 8U; ++n)
    {
        if ((CAN2->RF0R & CAN_RF0R_FMP0_Msk) == 0U) break;

        uint32_t rir  = CAN2->sFIFOMailBox[0].RIR;
        uint32_t rdtr = CAN2->sFIFOMailBox[0].RDTR;
        uint32_t lo   = CAN2->sFIFOMailBox[0].RDLR;
        uint32_t hi   = CAN2->sFIFOMailBox[0].RDHR;

        CAN2->RF0R |= CAN_RF0R_RFOM0;   /* 释放 FIFO0 */

        if ((rir & CAN_RI0R_IDE) != 0U)   /* 只收标准帧 */
        {
            g_imu.bad_frame_count++;
            continue;
        }

        uint16_t id = (uint16_t)((rir >> 21) & 0x7FFU);
        if (id != IMU_MASTER_ID && id != IMU_SLAVE_ID)
        {
            g_imu.bad_frame_count++;
            continue;
        }

        uint8_t dlc = (uint8_t)(rdtr & 0x0FU);
        if (dlc < 8U)
        {
            g_imu.bad_frame_count++;
            continue;
        }

        uint8_t d[8];
        d[0] = (uint8_t)lo;         d[1] = (uint8_t)(lo >> 8);
        d[2] = (uint8_t)(lo >> 16); d[3] = (uint8_t)(lo >> 24);
        d[4] = (uint8_t)hi;         d[5] = (uint8_t)(hi >> 8);
        d[6] = (uint8_t)(hi >> 16); d[7] = (uint8_t)(hi >> 24);

        g_imu.last_rx_ms = HAL_GetTick();
        g_imu.rx_count++;

        switch (d[0])
        {
            case 1U: imu_update_accel(d); break;
            case 2U: imu_update_gyro(d);  break;
            case 3U: imu_update_euler(d); break;
            default: break;   /* 四元数帧暂不使用 */
        }
    }

    uint32_t age = dm_imu_last_rx_age_ms();
    uint32_t yaw_age = (g_imu.last_yaw_ms == 0U)
                           ? 0xFFFFFFFFU
                           : (HAL_GetTick() - g_imu.last_yaw_ms);
    /* 只有欧拉角新鲜才算“可用来锁航向”；加速度帧不能掩盖 yaw 停更 */
    g_imu.online = (age < IMU_TIMEOUT_MS && yaw_age < IMU_TIMEOUT_MS) ? 1U : 0U;
}

void dm_imu_zero_yaw(void)
{
    s_yaw_offset = s_yaw_cont;
    g_imu.yaw_deg = 0.0f;
    imu_write_reg(DM_IMU_REG_SET_ZERO, 0U);
}

uint8_t dm_imu_is_online(void)
{
    return g_imu.online;
}

uint32_t dm_imu_last_rx_age_ms(void)
{
    if (g_imu.rx_count == 0U) return 0xFFFFFFFFU;
    return HAL_GetTick() - g_imu.last_rx_ms;
}
