/* 主机测试用的 dm_imu.h 替身：避免把 STM32 HAL 头文件带进 gcc 主机编译 */
#ifndef DM_IMU_H
#define DM_IMU_H

#include <stdint.h>

typedef struct
{
    float yaw_deg;
    float yaw_rate_dps;
    float pitch_deg;
    float roll_deg;
    float accel_mps2[3];
    float temperature_c;
    uint32_t rx_count;
    uint32_t last_rx_ms;
    uint32_t last_yaw_ms;
    uint32_t bad_frame_count;
    uint32_t init_retry_count;
    uint8_t online;
    uint8_t configured;
    uint8_t baud_index;
} dm_imu_t;

extern dm_imu_t g_imu;

uint8_t dm_imu_is_online(void);

#endif
