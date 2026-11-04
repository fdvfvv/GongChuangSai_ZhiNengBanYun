/**
 * @file    test_trajectory.c
 * @brief   主机端轨迹状态机测试：用简化“理想底盘”闭环验证
 *          直线 -> 90°转向 -> 90°圆弧 -> 等待 能正常走完且不超时。
 */

#include "trajectory.h"
#include "chassis.h"
#include "chassis_math.h"
#include "app_config.h"
#include "dm_imu.h"

#include <math.h>
#include <stdio.h>

dm_imu_t g_imu;

static float g_cmd_vx;
static float g_cmd_wz_ff;
static float g_cmd_target;
static float g_cmd_limit;
static uint8_t g_cmd_active;
static float g_max_vx;
static float g_min_vx;
static int g_stop_count;
static int g_failures;

#define CHECK_TRUE(cond, msg)                                                   \
    do                                                                          \
    {                                                                           \
        if (!(cond))                                                            \
        {                                                                       \
            printf("FAIL: %s (%s:%d)\n", (msg), __FILE__, __LINE__);            \
            ++g_failures;                                                       \
        }                                                                       \
    } while (0)

#define CHECK_NEAR(actual, expected, tol, msg)                                  \
    do                                                                          \
    {                                                                           \
        float a_ = (float)(actual);                                             \
        float e_ = (float)(expected);                                           \
        if (fabsf(a_ - e_) > (float)(tol))                                      \
        {                                                                       \
            printf("FAIL: %s actual=%f expected=%f\n", (msg), a_, e_);          \
            ++g_failures;                                                       \
        }                                                                       \
    } while (0)

uint8_t dm_imu_is_online(void)
{
    return 1U;
}

void chassis_set_target(float vx, float vy, float wz_ff_dps,
                        float target_yaw_deg, uint8_t heading_enable,
                        float wz_limit_dps)
{
    (void)vy;
    (void)heading_enable;
    g_cmd_vx = vx;
    g_cmd_wz_ff = wz_ff_dps;
    g_cmd_target = target_yaw_deg;
    g_cmd_limit = wz_limit_dps;
    g_cmd_active = 1U;

    if (vx > g_max_vx) g_max_vx = vx;
    if (vx < g_min_vx) g_min_vx = vx;
}

void chassis_stop(void)
{
    g_cmd_active = 0U;
    ++g_stop_count;
}

void chassis_update(float dt) { (void)dt; }
void chassis_clear_fault(void) {}
uint8_t chassis_faulted(void) { return 0U; }

/* 简化被控对象：航向一阶跟随 PID 输出，速度积分成 yaw */
static void plant_step(float dt)
{
    if (!g_cmd_active) return;

    float err = wrap180f(g_cmd_target - g_imu.yaw_deg);
    float wz = g_cmd_wz_ff + HEADING_KP * err - HEADING_KD * g_imu.yaw_rate_dps;

    float limit = g_cmd_limit > 0.0f ? g_cmd_limit : HEADING_MAX_WZ_DPS;
    wz = clampf(wz, -limit, limit);

    g_imu.yaw_rate_dps = wz;
    g_imu.yaw_deg += wz * dt;
}

static void reset_sim(void)
{
    g_imu.yaw_deg = 0.0f;
    g_imu.yaw_rate_dps = 0.0f;
    g_cmd_vx = 0.0f;
    g_cmd_wz_ff = 0.0f;
    g_cmd_target = 0.0f;
    g_cmd_limit = HEADING_MAX_WZ_DPS;
    g_cmd_active = 0U;
    g_max_vx = 0.0f;
    g_min_vx = 0.0f;
    g_stop_count = 0;
}

static void run_route(void)
{
    static const traj_segment_t route[] =
    {
        { TRAJ_SEG_STRAIGHT, 0.60f, 0.22f, 0.0f },
        { TRAJ_SEG_TURN,     90.0f, 50.0f, 0.0f },
        { TRAJ_SEG_ARC,      90.0f, 0.20f, 0.50f },
        { TRAJ_SEG_WAIT,      0.10f, 0.0f,  0.0f },
    };

    const float dt = 0.01f;
    reset_sim();
    traj_start(route, (uint8_t)(sizeof(route) / sizeof(route[0])));

    int steps = 0;
    while (traj_is_running() && steps < 4000)
    {
        traj_update(dt);
        plant_step(dt);
        ++steps;
    }

    CHECK_TRUE(!traj_is_running(), "route finished");
    CHECK_TRUE(traj_state() == TRAJ_STATE_DONE, "route state DONE");
    CHECK_TRUE(steps < 4000, "route finished before timeout");
    CHECK_NEAR(g_imu.yaw_deg, 180.0f, 3.0f, "final heading 180deg");
    CHECK_TRUE(g_stop_count > 0, "chassis_stop called at end");
}

static void test_abort(void)
{
    static const traj_segment_t route[] =
    {
        { TRAJ_SEG_STRAIGHT, 5.0f, 0.20f, 0.0f },
    };

    reset_sim();
    traj_start(route, 1U);
    for (int i = 0; i < 10; ++i)
    {
        traj_update(0.01f);
        plant_step(0.01f);
    }
    traj_abort();
    CHECK_TRUE(traj_state() == TRAJ_STATE_ABORTED, "abort state");
    CHECK_TRUE(traj_is_done(), "abort is done");
    CHECK_TRUE(g_stop_count > 0, "abort stops chassis");
}

static void test_turn_180(void)
{
    static const traj_segment_t route[] =
    {
        { TRAJ_SEG_TURN, 180.0f, 50.0f, 0.0f },
    };

    const float dt = 0.01f;
    reset_sim();
    traj_start(route, 1U);

    int steps = 0;
    while (traj_is_running() && steps < 1500)
    {
        traj_update(dt);
        plant_step(dt);
        ++steps;
    }

    CHECK_TRUE(traj_state() == TRAJ_STATE_DONE, "180 turn done");
    CHECK_NEAR(g_imu.yaw_deg, 180.0f, 3.0f, "180 turn goes left");
    CHECK_TRUE(g_imu.yaw_deg > 0.0f, "180 turn not reversed");
}

static void test_right_arc(void)
{
    static const traj_segment_t route[] =
    {
        { TRAJ_SEG_ARC, -90.0f, 0.20f, 0.50f },
    };

    const float dt = 0.01f;
    reset_sim();
    traj_start(route, 1U);

    int steps = 0;
    while (traj_is_running() && steps < 2000)
    {
        traj_update(dt);
        plant_step(dt);
        ++steps;
    }

    CHECK_TRUE(traj_state() == TRAJ_STATE_DONE, "right arc done");
    CHECK_NEAR(g_imu.yaw_deg, -90.0f, 3.0f, "right arc heading -90");
    CHECK_TRUE(g_max_vx > 0.05f, "right arc moves forward");
    CHECK_TRUE(g_min_vx > -0.01f, "right arc never reverses");
}

int main(void)
{
    run_route();
    test_turn_180();
    test_right_arc();
    test_abort();

    if (g_failures == 0)
    {
        printf("All trajectory tests passed.\n");
        return 0;
    }

    printf("%d trajectory test(s) failed.\n", g_failures);
    return 1;
}
