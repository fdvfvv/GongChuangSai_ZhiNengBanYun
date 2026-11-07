/**
 * @file    trace.c
 * @brief   运行数据黑匣子实现
 */

#include "trace.h"
#include "app_config.h"
#include "chassis.h"
#include "dm_imu.h"
#include "trajectory.h"

#include "main.h"

#if TRACE_ENABLE

trace_rec_t g_trace[TRACE_CAP];
volatile uint32_t g_trace_n;
volatile uint32_t g_trace_t0;

static int16_t f2i10(float v)
{
    float x = v * 10.0f;
    if (x > 32767.0f) x = 32767.0f;
    if (x < -32768.0f) x = -32768.0f;
    return (int16_t)x;
}

void trace_begin(void)
{
    g_trace_n = 0U;
    g_trace_t0 = HAL_GetTick();
}

void trace_tick(void)
{
    uint32_t n = g_trace_n;
    if (n >= TRACE_CAP) return;

    trace_rec_t *r = &g_trace[n];
    uint32_t t = HAL_GetTick() - g_trace_t0;

    r->t_ms = (uint16_t)(t > 65535U ? 65535U : t);
    r->yaw10 = f2i10(g_imu.yaw_deg);
    r->rate10 = f2i10(g_imu.yaw_rate_dps);
    r->wz10 = f2i10(g_chassis_state.wz_cmd_dps);
    for (uint8_t i = 0U; i < 4U; ++i)
    {
        r->rpm10[i] = f2i10(g_chassis_state.wheel_rpm_cmd[i]);
    }
    r->idx = (int8_t)traj_segment_index();
    r->state = (int8_t)traj_state();
    r->fault = (int8_t)g_chassis_state.fault;
    r->stamp = (int8_t)(n & 0x7F);

    g_trace_n = n + 1U;
}

#endif
