/**
 * @file    chassis_math.c
 * @brief   纯数学工具实现（无 HAL 依赖，可主机编译测试）
 */

#include "chassis_math.h"

#include <math.h>

float wrap180f(float deg)
{
    /* 注意：不能用 fmod(deg + 180, 360) - 180，那样 +180 会变成 -180，
     * 原地转 180° 时方向会反。这里保留输入在 ±180 边界上的符号。 */
    float r = fmodf(deg, 360.0f);
    if (r > 180.0f) r -= 360.0f;
    else if (r < -180.0f) r += 360.0f;
    return r;
}

float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

float slewf(float current, float target, float max_delta)
{
    float d = target - current;
    if (d > max_delta) d = max_delta;
    else if (d < -max_delta) d = -max_delta;
    return current + d;
}

void mecanum_inverse(float vx, float vy, float wz, float k, float out[4])
{
    /* 80mm 麦轮，辊子 45°，X 型布局：
     *   FL = vx - vy - k*wz
     *   FR = vx + vy + k*wz
     *   RL = vx + vy - k*wz
     *   RR = vx - vy + k*wz
     * 逆时针自转时左侧轮向后、右侧轮向前。 */
    out[0] = vx - vy - k * wz;
    out[1] = vx + vy + k * wz;
    out[2] = vx + vy - k * wz;
    out[3] = vx - vy + k * wz;
}

void differential_inverse(float vx, float wz, float half_track, float out[4])
{
    float left  = vx - wz * half_track;
    float right = vx + wz * half_track;
    out[0] = left;    /* FL */
    out[1] = right;   /* FR */
    out[2] = left;    /* RL */
    out[3] = right;   /* RR */
}

float wheel_mps_to_rpm(float mps, float diameter_m, float gear_ratio)
{
    if (diameter_m <= 0.0f) return 0.0f;
    return mps * 60.0f / (CHASSIS_PI_F * diameter_m) * gear_ratio;
}

float trapezoid_speed(float distance, float accel, float vmax, float traveled)
{
    if (distance <= 0.0f || accel <= 0.0f) return 0.0f;
    if (traveled >= distance) return 0.0f;

    /* 离散积分从 s=0 起步时 sqrt(2*a*0)=0 会永远不动。
     * 给一个很小的“起步距离”，相当于给一个很小的初速，不影响整体平滑度。 */
    if (traveled < 0.0001f) traveled = 0.0001f;

    float v_accel = sqrtf(2.0f * accel * traveled);
    float v_decel = sqrtf(2.0f * accel * (distance - traveled));
    float v = v_accel < v_decel ? v_accel : v_decel;
    return v < vmax ? v : vmax;
}

void pid_reset(pid_t *pid)
{
    if (pid != 0) pid->integral = 0.0f;
}

float pid_update(pid_t *pid, float err, float rate, float ff, float dt)
{
    if (pid == 0 || dt <= 0.0f) return ff;

    pid->integral += err * dt;
    pid->integral = clampf(pid->integral, -pid->i_limit, pid->i_limit);

    float out = ff + pid->kp * err + pid->ki * pid->integral - pid->kd * rate;
    return clampf(out, -pid->out_limit, pid->out_limit);
}
