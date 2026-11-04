/**
 * @file    chassis.c
 * @brief   四麦轮底盘闭环控制实现
 */

#include "chassis.h"
#include "app_config.h"
#include "dm_imu.h"
#include "chassis_math.h"
#include "zdt_can.h"

#include <math.h>

chassis_cmd_t g_chassis_cmd;
chassis_state_t g_chassis_state;

static pid_t s_heading_pid;
static float s_rpm_cmd[4];
static uint8_t s_fault;

static const uint8_t s_motor_addr[4] =
{
    MOTOR_ADDR_FL, MOTOR_ADDR_FR, MOTOR_ADDR_RL, MOTOR_ADDR_RR
};

static const float s_motor_invert[4] =
{
    MOTOR_FL_INVERT ? -1.0f : 1.0f,
    MOTOR_FR_INVERT ? -1.0f : 1.0f,
    MOTOR_RL_INVERT ? -1.0f : 1.0f,
    MOTOR_RR_INVERT ? -1.0f : 1.0f
};

static void send_wheel_rpm(const float rpm[4])
{
    for (uint8_t i = 0U; i < 4U; ++i)
    {
        float r = rpm[i];
        if (fabsf(r) < 0.5f) r = 0.0f;

        uint16_t mag = (uint16_t)(fabsf(r) + 0.5f);
        if (mag > 3000U) mag = 3000U;

        uint8_t dir = (r >= 0.0f) ? ZDT_DIR_CW : ZDT_DIR_CCW;
        (void)zdt_can_set_speed_nowait(s_motor_addr[i], dir, mag, ZDT_ACC);
    }
}

static void stop_now(void)
{
    const float zero[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    for (uint8_t i = 0U; i < 4U; ++i)
    {
        s_rpm_cmd[i] = 0.0f;
        g_chassis_state.wheel_rpm_cmd[i] = 0.0f;
    }
    send_wheel_rpm(zero);
}

void chassis_init(void)
{
    g_chassis_cmd.vx = 0.0f;
    g_chassis_cmd.vy = 0.0f;
    g_chassis_cmd.wz_ff = 0.0f;
    g_chassis_cmd.target_yaw_deg = 0.0f;
    g_chassis_cmd.wz_limit_dps = HEADING_MAX_WZ_DPS;
    g_chassis_cmd.heading_enable = 0U;
    g_chassis_cmd.active = 0U;

    g_chassis_state.fault = 0U;
    g_chassis_state.yaw_error_deg = 0.0f;
    g_chassis_state.wz_cmd_dps = 0.0f;
    for (uint8_t i = 0U; i < 4U; ++i)
    {
        g_chassis_state.wheel_rpm_cmd[i] = 0.0f;
        g_chassis_state.wheel_mps[i] = 0.0f;
        s_rpm_cmd[i] = 0.0f;
    }

    s_heading_pid.kp = HEADING_KP;
    s_heading_pid.ki = HEADING_KI;
    s_heading_pid.kd = HEADING_KD;
    s_heading_pid.i_limit = HEADING_I_LIMIT_DEG;
    s_heading_pid.out_limit = HEADING_MAX_WZ_DPS;
    pid_reset(&s_heading_pid);

    s_fault = 0U;
    stop_now();
}

void chassis_set_target(float vx, float vy, float wz_ff_dps,
                        float target_yaw_deg, uint8_t heading_enable,
                        float wz_limit_dps)
{
    if (s_fault) return;

    g_chassis_cmd.vx = vx;
    g_chassis_cmd.vy = vy;
    g_chassis_cmd.wz_ff = wz_ff_dps;
    g_chassis_cmd.target_yaw_deg = target_yaw_deg;
    g_chassis_cmd.wz_limit_dps = (wz_limit_dps > 0.0f) ? wz_limit_dps
                                                       : HEADING_MAX_WZ_DPS;
    g_chassis_cmd.heading_enable = heading_enable ? 1U : 0U;
    g_chassis_cmd.active = 1U;
}

void chassis_stop(void)
{
    g_chassis_cmd.vx = 0.0f;
    g_chassis_cmd.vy = 0.0f;
    g_chassis_cmd.wz_ff = 0.0f;
    g_chassis_cmd.wz_limit_dps = HEADING_MAX_WZ_DPS;
    g_chassis_cmd.heading_enable = 0U;
    g_chassis_cmd.active = 0U;
    g_chassis_state.yaw_error_deg = 0.0f;
    g_chassis_state.wz_cmd_dps = 0.0f;
    pid_reset(&s_heading_pid);
    stop_now();
}

void chassis_update(float dt)
{
    if (dt <= 0.0f) return;
    if (dt > 0.05f) dt = 0.05f;   /* 偶尔卡顿也不要把斜坡算飞 */

    /* IMU 掉线：立刻停轮并锁存故障，必须重新按键/复位才能再动 */
    if (!dm_imu_is_online())
    {
        if (!s_fault)
        {
            s_fault = 1U;
            g_chassis_state.fault = 1U;
            g_chassis_cmd.active = 0U;
            stop_now();
        }
        return;
    }

    if (s_fault) return;
    if (!g_chassis_cmd.active)
    {
        return;
    }

    float wz_limit = g_chassis_cmd.wz_limit_dps;
    if (wz_limit <= 0.0f || wz_limit > HEADING_MAX_WZ_DPS)
    {
        wz_limit = HEADING_MAX_WZ_DPS;
    }

    float wz_dps = g_chassis_cmd.wz_ff;
    g_chassis_state.yaw_error_deg = 0.0f;

    if (g_chassis_cmd.heading_enable)
    {
        float err = wrap180f(g_chassis_cmd.target_yaw_deg - g_imu.yaw_deg);
        g_chassis_state.yaw_error_deg = err;

        /* 小误差软死区：避免静摩擦下舵机式来回抖 */
        float err_p = err;
        if (fabsf(err) < HEADING_DEADBAND_DEG)
        {
            err_p = 0.0f;
        }
        else
        {
            err_p = (err > 0.0f) ? (err - HEADING_DEADBAND_DEG)
                                 : (err + HEADING_DEADBAND_DEG);
        }

        /* 输出限幅跟随本段限速，避免原地转向段积分饱和 */
        s_heading_pid.out_limit = wz_limit;
        float rate_err = g_imu.yaw_rate_dps - g_chassis_cmd.wz_ff;
        wz_dps = pid_update(&s_heading_pid, err_p, rate_err,
                            g_chassis_cmd.wz_ff, dt);
    }
    else
    {
        pid_reset(&s_heading_pid);
    }

    wz_dps = clampf(wz_dps, -wz_limit, wz_limit);
    g_chassis_state.wz_cmd_dps = wz_dps;

    float wz_rad = wz_dps / CHASSIS_RAD2DEG_F;
    float mps[4];

#if CHASSIS_MECANUM
    float k = 0.5f * (WHEEL_BASE_M + WHEEL_TRACK_M);
    mecanum_inverse(g_chassis_cmd.vx, g_chassis_cmd.vy, wz_rad, k, mps);
#else
    differential_inverse(g_chassis_cmd.vx, wz_rad, 0.5f * WHEEL_TRACK_M, mps);
#endif

    for (uint8_t i = 0U; i < 4U; ++i)
    {
        g_chassis_state.wheel_mps[i] = mps[i];

        float rpm = wheel_mps_to_rpm(mps[i], WHEEL_DIAMETER_M, WHEEL_GEAR_RATIO);
        rpm *= s_motor_invert[i];
        rpm = clampf(rpm, -MAX_WHEEL_RPM, MAX_WHEEL_RPM);

        s_rpm_cmd[i] = slewf(s_rpm_cmd[i], rpm, WHEEL_ACCEL_RPM_S * dt);
        g_chassis_state.wheel_rpm_cmd[i] = s_rpm_cmd[i];
    }

    send_wheel_rpm(s_rpm_cmd);
}

void chassis_clear_fault(void)
{
    s_fault = 0U;
    g_chassis_state.fault = 0U;
    pid_reset(&s_heading_pid);
    for (uint8_t i = 0U; i < 4U; ++i) s_rpm_cmd[i] = 0.0f;
}

uint8_t chassis_faulted(void)
{
    return s_fault;
}
