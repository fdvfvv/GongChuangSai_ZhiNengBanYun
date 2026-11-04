/**
 * @file    trajectory.h
 * @brief   基于陀螺仪航向的轨迹执行器：直线、原地转向、圆弧
 *
 * 说明：
 *   - 直线段距离是“按轮速积分”的里程，没有编码器反馈，长时间会有几厘米误差；
 *     航向由 DM-IMU 闭环，横向不会越跑越偏。
 *   - 转向/圆弧的角度由 DM-IMU 闭环，转到目标角度才结束，不靠延时。
 *   - 每个段都有超时保护，IMU 异常或机械卡死不会一直转。
 */

#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <stdint.h>

typedef enum
{
    TRAJ_SEG_STRAIGHT = 0,   /* a=距离 m, b=速度 m/s */
    TRAJ_SEG_TURN,           /* a=角度 deg（左正右负）, b=最大角速度 deg/s */
    TRAJ_SEG_ARC,            /* a=角度 deg（左正右负）, b=线速度 m/s, c=半径 m */
    TRAJ_SEG_WAIT            /* a=等待秒数 */
} traj_seg_type_t;

typedef struct
{
    uint8_t type;
    float a;
    float b;
    float c;
} traj_segment_t;

typedef enum
{
    TRAJ_STATE_IDLE = 0,
    TRAJ_STATE_RUNNING,
    TRAJ_STATE_DONE,
    TRAJ_STATE_ABORTED,
    TRAJ_STATE_FAULT
} traj_state_t;

void traj_start(const traj_segment_t *route, uint8_t count);
void traj_update(float dt);
void traj_abort(void);

uint8_t traj_is_running(void);
uint8_t traj_is_done(void);
traj_state_t traj_state(void);
uint8_t traj_segment_index(void);

#endif /* TRAJECTORY_H */
