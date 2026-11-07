/**
 * @file    trace.h
 * @brief   运行数据黑匣子：把控制环数据存进 RAM，跑完后用 ST-Link 一次性导出
 *
 * 为什么要它：用 OpenOCD 在线采样时，每次读内存都会短暂 halt 内核，
 * 会干扰 100Hz 控制环；而且车一动 ST-Link 线容易松，数据就断了。
 * 让固件自己按 100Hz 记录，跑完静止后再 dump，数据完整且不干扰控制。
 *
 * 导出（ST-Link 连着时）:
 *   openocd telnet 4444 ->  dump_image trace.bin <g_trace 地址> <g_trace_n * 20>
 */

#ifndef TRACE_H
#define TRACE_H

#include <stdint.h>

#define TRACE_ENABLE   1
#define TRACE_CAP      1500U   /* 1500 条 @100Hz = 15 秒 */

/* 一条记录 20 字节，字段尽量用定点数以省 RAM */
typedef struct
{
    uint16_t t_ms;      /* 距 trace_begin 的毫秒数 */
    int16_t  yaw10;     /* yaw_deg * 10 */
    int16_t  rate10;    /* yaw_rate_dps * 10 */
    int16_t  wz10;      /* 航向环输出角速度 * 10 */
    int16_t  rpm10[4];  /* 四轮命令 RPM * 10，顺序 FL/FR/RL/RR */
    int8_t   idx;       /* 轨迹段索引 */
    int8_t   state;     /* 轨迹状态，见 traj_state_t */
    int8_t   fault;     /* 底盘故障锁存（1=IMU 掉线） */
    int8_t   stamp;     /* 段内重入计数，用于定位异常切段 */
} trace_rec_t;

#if TRACE_ENABLE

extern trace_rec_t g_trace[TRACE_CAP];
extern volatile uint32_t g_trace_n;   /* 已记录条数 */
extern volatile uint32_t g_trace_t0;  /* 起始时刻 HAL_GetTick() */

void trace_begin(void);
void trace_tick(void);

#else

#define trace_begin()  do {} while (0)
#define trace_tick()   do {} while (0)

#endif

#endif /* TRACE_H */
