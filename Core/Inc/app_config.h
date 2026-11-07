/**
 * @file    app_config.h
 * @brief   智能搬运小车 · 陀螺仪闭环与轨迹参数总表
 *
 * 所有需要按实车调整的参数都集中在这里。改完重新编译即可，
 * 不需要到各个 .c 文件里找宏定义。
 *
 * 坐标系约定（与 DM-IMU 一致）：
 *   +X = 车头前进方向
 *   +Y = 车体左侧
 *   +Z = 车体上方
 *   yaw 正方向 = 从 +Z 向下看逆时针（左转）为正
 */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* ==========================================================================
 * 1. DM-IMU 陀螺仪
 * ========================================================================== */

/* DM-IMU 接在 A 板 CAN2：PB12 = CAN2_RX，PB13 = CAN2_TX（AF9）。
 * 注意：A 板丝印上的 CAN2 就是这两个脚，不是 PB5/PB13。 */
#define IMU_CAN_BAUD_1M         1U      /* 1 = 先按 1 Mbps 找 IMU */
#define IMU_AUTO_BAUD           1U      /* 1 = 1 Mbps 找不到时自动再试 500 kbps */
#define IMU_SLAVE_ID            1U      /* 上位机里设置的“从机 ID”，出厂 1 */
#define IMU_MASTER_ID           0x11U   /* 上位机里设置的“主机 ID”，出厂 0x11 */
#define IMU_YAW_SIGN            (+1.0f) /* 2026-11-07 实测标定：IMU 虽倒装(roll≈179°)但原始 yaw 仍是左转为正，保持 +1 */
#define IMU_TIMEOUT_MS          500U    /* 超过这么久没收到帧就判离线并停车（放宽点，避免偶发一帧丢判离线） */
#define IMU_RATE_LPF_ALPHA      0.35f   /* 角速度低通系数，0~1，越小越平滑但越滞后 */
#define IMU_SAVE_CONFIG         0U      /* 1 = 把主动上报配置写进 IMU Flash（少用，避免磨损） */

/* ==========================================================================
 * 2. 底盘与电机
 * ========================================================================== */

/* 80 mm 麦轮（资料里是 80mm 麦轮 + D6 联轴器直连 ZDT X42S） */
#define WHEEL_DIAMETER_M        0.080f
#define WHEEL_BASE_M            0.220f  /* 前轮轴到后轮轴的距离，按实车量 */
#define WHEEL_TRACK_M           0.220f  /* 左轮中心到右轮中心的距离，按实车量 */
#define WHEEL_GEAR_RATIO        1.0f    /* 电机轴 : 车轮轴，直连填 1.0 */

#define MAX_WHEEL_RPM           260.0f  /* 单轮最大转速，防止飞车 */
#define WHEEL_ACCEL_RPM_S       600.0f  /* 轮速斜坡，越大越冲，越小越柔 */
#define ZDT_ACC                 50U     /* 电机内部加速度参数，与原工程一致 */
#define ZDT_SELF_HEAL_MS        2000U   /* 每 2 秒重发 解除堵转+使能，沿用原工程自愈思路 */

/* 电机地址 -> 车轮位置。若转向/平移方向不对，先改下面的 INVERT。 */
#define MOTOR_ADDR_FL           1U
#define MOTOR_ADDR_FR           2U
#define MOTOR_ADDR_RL           3U
#define MOTOR_ADDR_RR           4U

/* 0 = 正转命令让该轮向前；1 = 反一下。
 * 麦轮左右镜像安装时，通常一侧需要取反，实车用“单轮点动”确认。 */
/* 2026-11-07 三次实车 + 黑匣子数据反推（不依赖肉眼）：
 * 设 f_i = 原始正命令下该轮的物理方向，由
 *   第2次(FL/RL取反) 前进段 yaw 恒 0  =>  f1+f2+f3+f4 = 0
 *   第1次(不取反)   前进段 yaw 掉到-22 =>  f1-f2+f3-f4 ≠ 0
 * 唯一解 f=(+,-,+,-)：**右侧两轮是镜像件**（右轮原始正命令是后退）。
 * 故只取反 FR、RR。校验：全轮前进时四轮物理同向；右转段 FL/FR 同向、
 * RL/RR 同向且两侧相反。 */
#define MOTOR_FL_INVERT         0
#define MOTOR_FR_INVERT         1
#define MOTOR_RL_INVERT         0
#define MOTOR_RR_INVERT         1

/* 1 = 麦轮全向底盘；0 = 普通四驱差速底盘（左两轮同速、右两轮同速） */
#define CHASSIS_MECANUM         1

/* ==========================================================================
 * 3. 航向闭环 PID
 * ========================================================================== */

#define HEADING_KP              2.20f   /* 每 1° 航向误差给多少 °/s 的修正 */
#define HEADING_KI              0.15f   /* 消除长期漂移，先给很小 */
#define HEADING_KD              0.08f   /* 用陀螺仪角速度做阻尼，抑制过冲 */
#define HEADING_I_LIMIT_DEG     15.0f   /* 积分限幅，防止打舵越积越大 */
#define HEADING_MAX_WZ_DPS      120.0f  /* 最大原地/修正角速度 */
#define HEADING_DEADBAND_DEG    0.35f   /* 小于这个误差不再修，避免来回抖 */

/* ==========================================================================
 * 4. 轨迹
 * ========================================================================== */

#define TRAJ_LINEAR_ACCEL_MPS2  0.35f   /* 直线加/减速度，越小越平稳 */
#define TRAJ_ANGULAR_ACCEL_DPS2 90.0f   /* 圆弧角加速度 */
#define TRAJ_TURN_TOL_DEG       1.5f    /* 原地转到这个误差内算到位 */
#define TRAJ_TURN_RATE_TOL_DPS  4.0f    /* 同时角速度要小于这个值 */
#define TRAJ_SETTLE_MS          120U    /* 到位后稳定这么久才切下一段 */
#define TRAJ_TURN_TIMEOUT_MS    6000U   /* 单次原地转向超时 */
#define TRAJ_ARC_TIMEOUT_MS     30000U  /* 单段圆弧超时 */

/* ==========================================================================
 * 5. 启动与调试
 * ========================================================================== */

/* 2026-11-07 调试：PB2 在 STM32F427 上是 BOOT1，A 板该脚有外部下拉，
 * 实测恒读 0(被当成常按)，按键事件不可用 -> 暂时改成上电延时自动发车。
 * 按键改到可用引脚后再改回 1。 */
#define APP_REQUIRE_START_BUTTON 0      /* 1 = 上电后按 A 板 KEY 才发车 */
#define APP_START_DELAY_MS       500U   /* 发车前延时（一上电就走，短一点） */
#define APP_CONTROL_PERIOD_MS    10U    /* 控制周期 10ms = 100Hz */

/* 1 = 上电只做电机链路自检（读状态/使能/读转速/点动），确认地址与方向。
 * 确认完必须改回 0。 */
#define APP_WHEEL_TEST_MODE      0

/* 1 = 上电做四模式方向标定：全正 / 左+右- / 前+后- / 对角，
 * 每种各转 2.5 秒并记录 yaw 变化，用来确定哪种组合能真原地转。
 * 标定完必须改回 0。 */
#define APP_MOTION_TEST_MODE     0

/* 1 = 上电只监控 IMU：不停轮、不跑轨迹，把 yaw 极值和峰值角速度记录到 RAM。
 * 用来单独验证陀螺仪是否真的在输出变化。验证完必须改回 0。 */
#define APP_IMU_MONITOR_MODE     0

/* 1 = 架空观察轮子转向：循环做“四轮同向 5s / 停 2s / 左右反向 5s / 停 2s”共 3 轮。
 * 走真实底盘通路（含 INVERT 修正），只看不动车。观察完必须改回 0。 */
#define APP_AIR_TEST_MODE        0


/* 1 = 编译“电机 CAN 诊断固件”：扫描地址 1~8，按 KEY 逐个点动。
 * 正常跑车必须改回 0。 */
#define APP_DIAG_MODE            0

#endif /* APP_CONFIG_H */
