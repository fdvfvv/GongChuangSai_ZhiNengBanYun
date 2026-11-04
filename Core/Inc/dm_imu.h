/**
 * @file    dm_imu.h
 * @brief   DM-IMU-L1 六轴惯性测量单元驱动（STM32F427 bxCAN / CAN2）
 *
 * 协议来源：DM-IMU-L1 使用说明书 V1.3 附录 C
 *   - 经典标准帧，8 字节，从机 ID 默认 1，主机 ID 默认 0x11
 *   - 主动上报帧 DATA[0]：1=加速度+温度，2=角速度，3=欧拉角，4=四元数
 *   - 多字节小端；欧拉角/角速度按 uint16 线性映射
 *
 * 本驱动只使用 欧拉角帧(3) 和 角速度帧(2)，并把偏航角展开成连续角度，
 * 这样小车可以连续左转/右转超过 ±180°。
 */

#ifndef DM_IMU_H
#define DM_IMU_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

typedef struct
{
    float yaw_deg;          /* 连续偏航角，上电时置 0，可超过 ±180 */
    float yaw_rate_dps;     /* 低通后的 Z 轴角速度，deg/s，逆时针为正 */
    float pitch_deg;
    float roll_deg;
    float accel_mps2[3];
    float temperature_c;

    uint32_t rx_count;      /* 收到并解析的有效数据帧数 */
    uint32_t last_rx_ms;    /* 最近一次有效帧的 HAL_GetTick() */
    uint32_t last_yaw_ms;   /* 最近一次欧拉角帧的时间（航向控制必须用这个） */
    uint32_t bad_frame_count;
    uint32_t init_retry_count;

    uint8_t online;         /* 1 = 最近 IMU_TIMEOUT_MS 内有数据 */
    uint8_t configured;     /* 1 = 已成功发出主动上报配置命令 */
    uint8_t baud_index;     /* 0=1Mbps, 1=500kbps */
} dm_imu_t;

extern dm_imu_t g_imu;

/**
 * @brief  初始化 CAN2 并配置 IMU 主动上报。
 *         会依次尝试 1 Mbps / 500 kbps（可用 app_config.h 关闭）。
 * @retval 1 = 已收到有效数据；0 = 初始化失败/未收到数据
 */
uint8_t dm_imu_init(void);

/**
 * @brief  轮询 CAN2 FIFO，解析所有已到的 IMU 帧。建议 100~1000Hz 调用。
 */
void dm_imu_poll(void);

/**
 * @brief  把当前偏航角记为 0°（软件零位，同时尝试发 IMU 航向归零命令）。
 */
void dm_imu_zero_yaw(void);

/** @brief 1 = IMU 在线（最近 IMU_TIMEOUT_MS 内有帧） */
uint8_t dm_imu_is_online(void);

/** @brief 距最近一帧的毫秒数，没收到过返回 0xFFFFFFFF */
uint32_t dm_imu_last_rx_age_ms(void);

#endif /* DM_IMU_H */
