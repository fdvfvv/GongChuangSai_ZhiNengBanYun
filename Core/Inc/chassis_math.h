/**
 * @file    chassis_math.h
 * @brief   纯数学工具：角度展开、麦轮逆运动学、速度规划、PID
 *
 * 这个文件不依赖 STM32 HAL，方便在电脑上用 gcc 跑单元测试
 * （见 tests/host/test_chassis_math.c）。
 */

#ifndef CHASSIS_MATH_H
#define CHASSIS_MATH_H

#ifdef __cplusplus
extern "C" {
#endif

#define CHASSIS_PI_F       3.14159265358979323846f
#define CHASSIS_RAD2DEG_F  57.2957795130823208768f

/** @brief 把角度折到 (-180, 180] */
float wrap180f(float deg);

float clampf(float x, float lo, float hi);

/** @brief 以 max_delta 为步长把 current 推向 target */
float slewf(float current, float target, float max_delta);

/**
 * @brief 麦轮逆运动学：车体速度 -> 四个轮子的线速度
 * @param vx   前进速度 m/s
 * @param vy   左移速度 m/s（+Y 向左）
 * @param wz   自转角速度 rad/s（逆时针为正）
 * @param k    等效半径 = (轴距 + 轮距) / 2，单位 m
 * @param out  输出顺序：FL, FR, RL, RR，单位 m/s
 */
void mecanum_inverse(float vx, float vy, float wz, float k, float out[4]);

/**
 * @brief 差速底盘逆运动学（四驱：左侧两轮同速，右侧两轮同速）
 * @param half_track 轮距的一半，单位 m
 * @param out        输出顺序：FL, FR, RL, RR，单位 m/s
 */
void differential_inverse(float vx, float wz, float half_track, float out[4]);

/** @brief 轮子线速度 m/s -> 电机 RPM */
float wheel_mps_to_rpm(float mps, float diameter_m, float gear_ratio);

/**
 * @brief 梯形/三角形速度规划：根据“总距离、已走距离”给出当前允许速度。
 *        起步为 0，终点为 0，中间不超过 vmax。
 */
float trapezoid_speed(float distance, float accel, float vmax, float traveled);

typedef struct
{
    float kp;
    float ki;
    float kd;
    float i_limit;
    float out_limit;
    float integral;
} pid_t;

void pid_reset(pid_t *pid);

/**
 * @brief 位置式 PID（带积分限幅、输出限幅、角速度阻尼）
 * @param err   目标 - 实际（度）
 * @param rate  实际角速度（度/秒），用于 D 项
 * @param ff    前馈量
 * @param dt    周期（秒）
 */
float pid_update(pid_t *pid, float err, float rate, float ff, float dt);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_MATH_H */
