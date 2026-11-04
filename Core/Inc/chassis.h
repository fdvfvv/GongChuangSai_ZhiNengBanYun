/**
 * @file    chassis.h
 * @brief   四麦轮底盘：航向闭环 + 轮速分配 + 电机下发
 *
 * 控制链路：
 *   trajectory 给出 (vx, vy, wz 前馈, 目标 yaw)
 *     -> PID 用 DM-IMU 的 yaw / yaw_rate 修正 wz
 *     -> 麦轮逆运动学得到四轮线速度
 *     -> 换算 RPM、斜坡限幅、按地址发给 ZDT 闭环步进
 */

#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdint.h>

typedef struct
{
    float vx;               /* m/s，前进为正 */
    float vy;               /* m/s，左移为正 */
    float wz_ff;            /* deg/s，前馈角速度，逆时针为正 */
    float target_yaw_deg;   /* 目标连续航向 */
    float wz_limit_dps;     /* 本段允许的最大角速度，防止原地转向超速 */
    uint8_t heading_enable; /* 1 = 用 PID 锁航向 */
    uint8_t active;         /* 1 = 正在输出 */
} chassis_cmd_t;

typedef struct
{
    float wheel_rpm_cmd[4];  /* 斜坡后的电机命令，顺序 FL/FR/RL/RR */
    float wheel_mps[4];      /* 逆运动学结果，顺序 FL/FR/RL/RR */
    float yaw_error_deg;
    float wz_cmd_dps;
    uint8_t fault;           /* 1 = IMU 掉线/底盘故障，已锁存 */
} chassis_state_t;

extern chassis_cmd_t g_chassis_cmd;
extern chassis_state_t g_chassis_state;

void chassis_init(void);

/**
 * @brief 设置底盘目标。heading_enable=1 时 target_yaw_deg 有效。
 * @param wz_limit_dps 本段最大角速度；<=0 时用 HEADING_MAX_WZ_DPS
 */
void chassis_set_target(float vx, float vy, float wz_ff_dps,
                        float target_yaw_deg, uint8_t heading_enable,
                        float wz_limit_dps);

/** @brief 立即停车（四轮 0 RPM），并清掉当前目标 */
void chassis_stop(void);

/** @brief 100Hz 调用：PID -> 逆运动学 -> 斜坡 -> CAN 下发 */
void chassis_update(float dt);

/** @brief 清除 IMU 掉线锁存，允许重新发车 */
void chassis_clear_fault(void);

uint8_t chassis_faulted(void);

#endif /* CHASSIS_H */
