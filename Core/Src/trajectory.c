/**
 * @file    trajectory.c
 * @brief   直线 / 原地转向 / 圆弧 轨迹执行器
 */

#include "trajectory.h"
#include "app_config.h"
#include "chassis.h"
#include "chassis_math.h"
#include "dm_imu.h"

#include <math.h>

static const traj_segment_t *s_route;
static uint8_t s_count;
static uint8_t s_idx;
static traj_state_t s_state;

static float s_ref_yaw;         /* 整条轨迹的连续航向基准 */
static float s_seg_ref_start;   /* 本段开始时的航向基准 */
static float s_distance;        /* 直线已走距离 */
static float s_elapsed_s;       /* 本段已用时间 */
static float s_settle_s;        /* 转向到位稳定时间 */
static uint8_t s_arc_settling;  /* 圆弧走完后正在原地修最后一点角度 */

static void enter_segment(void)
{
    if (s_idx >= s_count)
    {
        chassis_stop();
        s_state = TRAJ_STATE_DONE;
        return;
    }

    const traj_segment_t *seg = &s_route[s_idx];

    s_seg_ref_start = s_ref_yaw;
    s_distance = 0.0f;
    s_elapsed_s = 0.0f;
    s_settle_s = 0.0f;
    s_arc_settling = 0U;

    if (seg->type == (uint8_t)TRAJ_SEG_TURN)
    {
        s_ref_yaw += seg->a;   /* 绝对目标角，误差不会逐段累积 */
    }
}

static void advance_segment(void)
{
    s_idx++;
    enter_segment();
}

static void traj_fault(void)
{
    chassis_stop();
    s_state = TRAJ_STATE_FAULT;
}

void traj_start(const traj_segment_t *route, uint8_t count)
{
    s_route = route;
    s_count = count;
    s_idx = 0U;
    s_settle_s = 0.0f;

    if (route == 0 || count == 0U)
    {
        chassis_stop();
        s_state = TRAJ_STATE_DONE;
        return;
    }

    s_ref_yaw = g_imu.yaw_deg;
    s_state = TRAJ_STATE_RUNNING;
    enter_segment();
}

void traj_update(float dt)
{
    if (s_state != TRAJ_STATE_RUNNING || s_route == 0 || dt <= 0.0f) return;

    const traj_segment_t *seg = &s_route[s_idx];
    s_elapsed_s += dt;

    switch (seg->type)
    {
        case (uint8_t)TRAJ_SEG_STRAIGHT:
        {
            float v = trapezoid_speed(seg->a, TRAJ_LINEAR_ACCEL_MPS2, seg->b, s_distance);
            chassis_set_target(v, 0.0f, 0.0f, s_ref_yaw, 1U, HEADING_MAX_WZ_DPS);
            s_distance += v * dt;

            float timeout_s = (seg->b > 0.01f)
                                  ? (seg->a / seg->b * 3.0f + 3.0f)
                                  : 5.0f;
            if (s_distance >= seg->a - 0.0005f)
            {
                s_distance = seg->a;
                advance_segment();
            }
            else if (s_elapsed_s > timeout_s)
            {
                traj_fault();
            }
            break;
        }

        case (uint8_t)TRAJ_SEG_TURN:
        {
            float turn_limit = fabsf(seg->b);
            if (turn_limit < 1.0f) turn_limit = HEADING_MAX_WZ_DPS;

            chassis_set_target(0.0f, 0.0f, 0.0f, s_ref_yaw, 1U, turn_limit);

            float err = wrap180f(s_ref_yaw - g_imu.yaw_deg);
            if (fabsf(err) < TRAJ_TURN_TOL_DEG &&
                fabsf(g_imu.yaw_rate_dps) < TRAJ_TURN_RATE_TOL_DPS)
            {
                s_settle_s += dt;
            }
            else
            {
                s_settle_s = 0.0f;
            }

            float timeout_ms = (float)TRAJ_TURN_TIMEOUT_MS;
            if (seg->b > 1.0f)
            {
                timeout_ms += fabsf(seg->a) / seg->b * 1000.0f;
            }

            if ((s_settle_s * 1000.0f) >= (float)TRAJ_SETTLE_MS)
            {
                advance_segment();
            }
            else if ((s_elapsed_s * 1000.0f) > timeout_ms)
            {
                traj_fault();
            }
            break;
        }

        case (uint8_t)TRAJ_SEG_ARC:
        {
            float total = fabsf(seg->a);
            if (total < 0.01f || seg->c <= 0.0f)
            {
                advance_segment();
                break;
            }

            float sign = (seg->a >= 0.0f) ? 1.0f : -1.0f;
            float omega_max = fabsf(seg->b / seg->c) * CHASSIS_RAD2DEG_F;
            if (omega_max < 1.0f) omega_max = 1.0f;
            float timeout_s = total / omega_max * 3.0f + 5.0f;
            if (timeout_s < (float)TRAJ_ARC_TIMEOUT_MS / 1000.0f)
            {
                timeout_s = (float)TRAJ_ARC_TIMEOUT_MS / 1000.0f;
            }

            if (!s_arc_settling)
            {
                float traveled = sign * (s_ref_yaw - s_seg_ref_start);
                if (traveled < 0.0f) traveled = 0.0f;
                if (traveled > total) traveled = total;

                float omega = trapezoid_speed(total, TRAJ_ANGULAR_ACCEL_DPS2,
                                              omega_max, traveled);

                s_ref_yaw += sign * omega * dt;

                /* 线速度始终向前；左弧/右弧只改角速度方向。
                 * 若写成 sign*omega*c，右弧会变成倒车。 */
                float vx = (omega / CHASSIS_RAD2DEG_F) * seg->c;
                float arc_limit = omega_max + 15.0f;
                if (arc_limit > HEADING_MAX_WZ_DPS) arc_limit = HEADING_MAX_WZ_DPS;
                chassis_set_target(vx, 0.0f, sign * omega, s_ref_yaw, 1U, arc_limit);

                float new_traveled = sign * (s_ref_yaw - s_seg_ref_start);
                if (new_traveled >= total - 0.001f)
                {
                    /* 圆弧走完：先停车，再用原地转向把最后一点角度修准 */
                    s_ref_yaw = s_seg_ref_start + seg->a;
                    s_arc_settling = 1U;
                    s_settle_s = 0.0f;
                }
            }
            else
            {
                chassis_set_target(0.0f, 0.0f, 0.0f, s_ref_yaw, 1U,
                                   HEADING_MAX_WZ_DPS);

                float err = wrap180f(s_ref_yaw - g_imu.yaw_deg);
                if (fabsf(err) < TRAJ_TURN_TOL_DEG &&
                    fabsf(g_imu.yaw_rate_dps) < TRAJ_TURN_RATE_TOL_DPS)
                {
                    s_settle_s += dt;
                }
                else
                {
                    s_settle_s = 0.0f;
                }

                if ((s_settle_s * 1000.0f) >= (float)TRAJ_SETTLE_MS)
                {
                    advance_segment();
                }
            }

            if (s_state == TRAJ_STATE_RUNNING && s_elapsed_s > timeout_s)
            {
                traj_fault();
            }
            break;
        }

        case (uint8_t)TRAJ_SEG_WAIT:
        {
            chassis_stop();
            if (s_elapsed_s >= seg->a) advance_segment();
            break;
        }

        default:
            advance_segment();
            break;
    }
}

void traj_abort(void)
{
    if (s_state == TRAJ_STATE_RUNNING)
    {
        chassis_stop();
        s_state = TRAJ_STATE_ABORTED;
    }
}

uint8_t traj_is_running(void)
{
    return (s_state == TRAJ_STATE_RUNNING) ? 1U : 0U;
}

uint8_t traj_is_done(void)
{
    return (s_state == TRAJ_STATE_DONE ||
            s_state == TRAJ_STATE_ABORTED ||
            s_state == TRAJ_STATE_FAULT) ? 1U : 0U;
}

traj_state_t traj_state(void)
{
    return s_state;
}

uint8_t traj_segment_index(void)
{
    return s_idx;
}
