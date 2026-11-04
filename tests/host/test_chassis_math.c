/**
 * @file    test_chassis_math.c
 * @brief   主机端单元测试：不接硬件，验证角度/麦轮/速度规划/PID 的数学正确性
 *
 * 运行：
 *   powershell -ExecutionPolicy Bypass -File tests/host/run_tests.ps1
 */

#include "chassis_math.h"

#include <math.h>
#include <stdio.h>

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
            printf("FAIL: %s  actual=%f expected=%f tol=%f (%s:%d)\n",          \
                   (msg), a_, e_, (float)(tol), __FILE__, __LINE__);            \
            ++g_failures;                                                       \
        }                                                                       \
    } while (0)

static void test_wrap180(void)
{
    CHECK_NEAR(wrap180f(0.0f), 0.0f, 1e-5f, "wrap 0");
    CHECK_NEAR(wrap180f(179.0f), 179.0f, 1e-5f, "wrap 179");
    CHECK_NEAR(wrap180f(181.0f), -179.0f, 1e-5f, "wrap 181");
    CHECK_NEAR(wrap180f(-181.0f), 179.0f, 1e-5f, "wrap -181");
    CHECK_NEAR(wrap180f(540.0f), 180.0f, 1e-5f, "wrap 540");
    CHECK_NEAR(wrap180f(-540.0f), -180.0f, 1e-5f, "wrap -540");
    CHECK_NEAR(wrap180f(180.0f), 180.0f, 1e-5f, "wrap +180");
    CHECK_NEAR(wrap180f(-180.0f), -180.0f, 1e-5f, "wrap -180");
}

static void test_slew(void)
{
    CHECK_NEAR(slewf(0.0f, 100.0f, 10.0f), 10.0f, 1e-5f, "slew up");
    CHECK_NEAR(slewf(0.0f, -100.0f, 10.0f), -10.0f, 1e-5f, "slew down");
    CHECK_NEAR(slewf(95.0f, 100.0f, 10.0f), 100.0f, 1e-5f, "slew clamp");
    CHECK_NEAR(clampf(5.0f, -1.0f, 1.0f), 1.0f, 1e-5f, "clamp high");
}

static void test_mecanum(void)
{
    float w[4];
    float k = 0.22f;

    /* 纯前进：四轮同速同向 */
    mecanum_inverse(0.5f, 0.0f, 0.0f, k, w);
    for (int i = 0; i < 4; ++i)
    {
        CHECK_NEAR(w[i], 0.5f, 1e-6f, "mecanum forward equal");
    }

    /* 纯左移：FL/RL 反向，FR/RR 同向 */
    mecanum_inverse(0.0f, 0.5f, 0.0f, k, w);
    CHECK_NEAR(w[0], -0.5f, 1e-6f, "strafe left FL");
    CHECK_NEAR(w[1], 0.5f, 1e-6f, "strafe left FR");
    CHECK_NEAR(w[2], 0.5f, 1e-6f, "strafe left RL");
    CHECK_NEAR(w[3], -0.5f, 1e-6f, "strafe left RR");

    /* 纯逆时针自转：左侧两轮向后，右侧两轮向前 */
    mecanum_inverse(0.0f, 0.0f, 1.0f, k, w);
    CHECK_NEAR(w[0], -k, 1e-5f, "rotate CCW FL");
    CHECK_NEAR(w[1], k, 1e-5f, "rotate CCW FR");
    CHECK_NEAR(w[2], -k, 1e-5f, "rotate CCW RL");
    CHECK_NEAR(w[3], k, 1e-5f, "rotate CCW RR");

    /* 差速：左转时右轮比左轮快 */
    differential_inverse(0.5f, 1.0f, 0.11f, w);
    CHECK_TRUE(w[0] < w[1], "differential left turn right faster");
    CHECK_NEAR(w[0], w[2], 1e-6f, "differential FL==RL");
    CHECK_NEAR(w[1], w[3], 1e-6f, "differential FR==RR");
}

static void test_rpm(void)
{
    /* 80mm 轮，1 m/s -> 60/(pi*0.08) = 238.732 RPM */
    CHECK_NEAR(wheel_mps_to_rpm(1.0f, 0.08f, 1.0f), 238.7324f, 0.01f, "rpm 80mm");
    CHECK_NEAR(wheel_mps_to_rpm(0.0f, 0.08f, 1.0f), 0.0f, 1e-6f, "rpm zero");
}

static void test_trapezoid(void)
{
    const float d = 1.0f;
    const float a = 0.35f;
    const float vmax = 0.5f;

    CHECK_TRUE(trapezoid_speed(d, a, vmax, 0.0f) < 0.02f, "trap start small");
    CHECK_TRUE(trapezoid_speed(d, a, vmax, 0.1f) <= vmax, "trap <= vmax");
    CHECK_TRUE(trapezoid_speed(d, a, vmax, d) == 0.0f, "trap end");
    CHECK_TRUE(trapezoid_speed(d, a, vmax, d * 0.5f) > trapezoid_speed(d, a, vmax, 0.05f),
               "trap accelerates");

    /* 数值积分：应能在有限时间内走完 1m，且不超过 vmax */
    float s = 0.0f;
    float dt = 0.001f;
    int steps = 0;
    while (s < d && steps < 20000)
    {
        float v = trapezoid_speed(d, a, vmax, s);
        CHECK_TRUE(v <= vmax + 1e-5f, "trap integrate <= vmax");
        s += v * dt;
        ++steps;
    }
    CHECK_TRUE(s >= d, "trap integrate reaches distance");
    CHECK_TRUE(steps < 10000, "trap integrate reasonable time");
}

static void test_pid(void)
{
    pid_t p = {2.0f, 0.1f, 0.05f, 15.0f, 120.0f, 0.0f};

    pid_reset(&p);
    CHECK_NEAR(pid_update(&p, 0.0f, 0.0f, 0.0f, 0.01f), 0.0f, 1e-6f, "pid zero");

    /* 只看 P 项时把 I 置 0，避免积分项干扰 */
    pid_t pp = {2.0f, 0.0f, 0.0f, 15.0f, 120.0f, 0.0f};
    CHECK_NEAR(pid_update(&pp, 10.0f, 0.0f, 0.0f, 0.01f), 20.0f, 1e-4f, "pid P");

    pid_reset(&p);
    float out = pid_update(&p, 100.0f, 0.0f, 0.0f, 0.01f);
    CHECK_NEAR(out, 120.0f, 1e-4f, "pid output clamp");

    pid_reset(&p);
    out = pid_update(&p, 0.0f, 100.0f, 0.0f, 0.01f);
    CHECK_NEAR(out, -5.0f, 1e-4f, "pid D damping");

    /* 积分限幅：持续大误差不应让积分无限增长 */
    pid_reset(&p);
    for (int i = 0; i < 1000; ++i) (void)pid_update(&p, 50.0f, 0.0f, 0.0f, 0.01f);
    CHECK_TRUE(p.integral <= p.i_limit + 1e-4f, "pid integral limit high");
    CHECK_TRUE(p.integral >= -p.i_limit - 1e-4f, "pid integral limit low");
}

int main(void)
{
    test_wrap180();
    test_slew();
    test_mecanum();
    test_rpm();
    test_trapezoid();
    test_pid();

    if (g_failures == 0)
    {
        printf("All chassis_math tests passed.\n");
        return 0;
    }

    printf("%d test(s) failed.\n", g_failures);
    return 1;
}
