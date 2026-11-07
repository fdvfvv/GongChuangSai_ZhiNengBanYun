# 智能搬运小车 · DM-IMU 陀螺仪闭环版

> 在开源工程 [GongChuangSai_ZhiNengBanYun](https://github.com/fdvfvv/GongChuangSai_ZhiNengBanYun) 的基础上，给 RoboMaster A 板 + 4 台张大头 ZDT 闭环步进 + 80 mm 麦轮底盘加入 **DM-IMU-L1 六轴陀螺仪**，实现：
>
> - 直线行驶航向锁定，不再越走越偏；
> - 原地转向按陀螺仪角度闭环，90°/180° 不靠延时；
> - 圆弧按“线速度 + 角速度前馈 + 航向 PID”走，速度平滑；
> - 每 2 秒自动重发“解除堵转 + 使能”，驱动器掉使能/堵转保护后能自愈；
> - IMU 掉线/轨迹超时自动停车并闪灯报警；
> - 所有参数集中在 `Core/Inc/app_config.h`，按实车改宏即可。

分支：`feat/dm-imu-gyro`（基于原仓库 `main` 的 `9506d13`）

- 分支地址：<https://github.com/fdvfvv/GongChuangSai_ZhiNengBanYun/tree/feat/dm-imu-gyro>
- 新建 PR：<https://github.com/fdvfvv/GongChuangSai_ZhiNengBanYun/pull/new/feat/dm-imu-gyro>

---

## 1. 硬件接线

### 1.1 总线分配

| 总线 | A 板引脚 | 波特率 | 挂载设备 | 帧格式 |
|---|---|---|---|---|
| CAN1 | `PD0` = RX，`PD1` = TX | **500 kbps** | 4 × ZDT 闭环步进（地址 1~4） | 扩展帧，`ExtId = (addr << 8) \| pack` |
| CAN2 | `PB12` = RX，`PB13` = TX | **1 Mbps**（可自动试 500 kbps） | DM-IMU-L1 | 经典标准帧，数据帧 ID = 主机 ID `0x11` |

> ⚠️ A 板丝印上的 CAN2 是 **PB12/PB13**，不是 PB5/PB13。PB5 是加热 PWM。原工程头文件里的注释已修正。

### 1.2 DM-IMU 接线

| DM-IMU | A 板 |
|---|---|
| `CAN_H` | `CAN2_H` |
| `CAN_L` | `CAN2_L` |
| `GND` | `GND` |
| `5V`（或 5~28V 外部电源） | 5V 电源输出（注意共地） |

- CAN 总线两端各需要 **120 Ω 终端电阻**；量 `CAN_H`~`CAN_L` 约 60 Ω 说明两端都接好。
- IMU 用刚性支架固定，**X 轴指向车头、Z 轴向上**；如果实际安装方向不同，在 DM-IMU 上位机里设置“安装方向”，或把 `IMU_YAW_SIGN` 改成 `-1.0f`（仅适用于水平旋转 180°/倒装导致 yaw 反向的情况）。
- IMU 供电和电机电源共地；不要带电插拔 CAN 线。

### 1.3 电机

- 4 台 ZDT 驱动器地址设为 `1/2/3/4`，CAN 波特率 **500 kbps**，Emm 固件。
- 80 mm 麦轮通过 D6 联轴器直连电机轴，`WHEEL_GEAR_RATIO = 1.0`。
- 麦轮有左/右旋之分，四个轮子必须按“X 型辊子”正确安装，否则平移会跑偏。

---

## 2. DM-IMU 上位机设置

用 USB-C 连接 IMU，打开达妙上位机：

1. **通信设置**：接口选 `CAN`，波特率 `1 Mbps`（如果整车用 500 kbps，也可以选 500 kbps，代码会自动尝试两种）。
2. **地址**：从机 ID `1`，主机 ID `0x11`（与 `app_config.h` 的 `IMU_SLAVE_ID` / `IMU_MASTER_ID` 一致）。
3. **数据输出**：打开“欧拉角”和“角速度”，上报间隔 `10 ms`（100 Hz）。代码启动时也会主动写这三个配置。
   - 代码先尝试“主动上报”；如果 500 ms 内没有收到欧拉角，会自动切到 **100 Hz 请求-应答模式**（每 10 ms 发一次欧拉角/角速度读取命令）。所以即使上位机把主动上报关掉，也能正常拿到姿态。
4. **安装方向**：按实际安装选择，推荐 X 前、Z 上。
5. **校准**：把车放平、静止，执行一次“静止零偏校准”；灯快闪约 3 秒，绿灯常亮即成功。
6. **航向归零**：可选。代码每次发车前会做软件归零，不依赖设备保存的零位。

> 如果修改了从机/主机 ID 或波特率，记得同步改 `Core/Inc/app_config.h`。  
> `IMU_SAVE_CONFIG` 默认 `0`，即不把配置写进 IMU Flash；需要固化时再改成 `1`。

---

## 3. 编译与烧录

环境与原工程一致：CMake + Ninja + `arm-none-eabi-gcc`（STM32CubeCLT 或 xPack 均可）。

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

生成：

```
build/Debug/GongChuangSai_Project.elf
```

烧录方式二选一：

- VSCode 里用 STM32 VS Code Extension：`STM32: Build & Download`；
- 命令行 OpenOCD（路径按自己安装位置改）：

```powershell
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg `
  -c "program build/Debug/GongChuangSai_Project.elf verify reset exit"
```

---

## 4. 代码结构

```
Core/
  Inc/
    app_config.h      所有可调参数（IMU、底盘、PID、轨迹、启动方式）
    dm_imu.h          DM-IMU 驱动接口
    chassis.h         底盘闭环接口
    chassis_math.h    纯数学：角度/麦轮/速度规划/PID
    trajectory.h      轨迹段定义与执行器
    zdt_can.h         ZDT 电机 CAN 协议
  Src/
    main.c            上电流程、按键、LED、100Hz 主循环、演示轨迹
    dm_imu.c          CAN2 裸寄存器驱动 + IMU 协议解析 + 偏航角展开
    chassis.c         航向 PID -> 麦轮逆运动学 -> RPM 斜坡 -> CAN 下发
    chassis_math.c    无 HAL 依赖，可在电脑上跑单元测试
    trajectory.c      直线 / 原地转向 / 圆弧 / 等待 状态机
    zdt_can.c         CAN1 电机命令（新增 nowait 版本）
tests/host/           电脑端单元测试（gcc，不需要开发板）
```

### 4.1 控制链路

```
trajectory.c
  │  给出 vx, vy, wz 前馈, 目标 yaw
  ▼
chassis.c
  │  yaw 误差 -> PID -> wz 修正（用 DM-IMU 的 yaw / yaw_rate）
  │  wz 限幅
  ▼
chassis_math.c
  │  麦轮逆运动学：FL/FR/RL/RR 线速度
  │  线速度 -> RPM，按 MOTOR_*_INVERT 取反
  ▼
zdt_can.c
  │  每 10 ms 发 4 帧速度命令（不等 ACK，避免单台掉线拖死循环）
  ▼
4 × ZDT 闭环步进
```

### 4.2 偏航角展开

DM-IMU 输出的 yaw 是 `-180°~+180°` 的归一化角，过 ±180° 会跳变。`dm_imu.c` 里按帧间差展开成连续角度：

```c
delta = yaw - last_yaw;
if (delta >  180) delta -= 360;
if (delta < -180) delta += 360;
yaw_continuous += delta;
```

这样连续左转 360°、720° 都不会在 ±180° 处反向。上电第一帧记为 0°，发车前再软件归零一次。

### 4.3 航向 PID

```c
wz = wz_ff + Kp * err + Ki * ∫err dt - Kd * gyro_z
```

- `err` 是目标连续 yaw 与当前 yaw 的差；
- `gyro_z` 直接来自 IMU 角速度，比“对 yaw 差分”噪声小，用来做阻尼；
- 输出限幅 `HEADING_MAX_WZ_DPS`，每段还可以再限制（例如原地转 90° 时限 50°/s）；
- 小误差软死区 `HEADING_DEADBAND_DEG`，避免静摩擦下左右抖。

### 4.4 麦轮逆运动学

坐标：X 前、Y 左、Z 上，逆时针 yaw 为正。

```
FL = vx - vy - k*wz
FR = vx + vy + k*wz
RL = vx + vy - k*wz
RR = vx - vy + k*wz
k  = (轴距 + 轮距) / 2
```

### 4.5 速度规划

直线和圆弧都用“按剩余距离开方”的梯形/三角形速度曲线：

```
v = min(vmax, sqrt(2*a*s), sqrt(2*a*(D-s)))
```

起步和停止都是 0，中间不超过 `vmax`，没有阶跃，麦轮不容易打滑。

---

## 5. 参数调整（`Core/Inc/app_config.h`）

### 5.1 最先要改的

| 宏 | 默认 | 说明 |
|---|---|---|
| `WHEEL_DIAMETER_M` | `0.080` | 80 mm 麦轮 |
| `WHEEL_BASE_M` | `0.220` | 前轮轴到后轮轴，按实车量 |
| `WHEEL_TRACK_M` | `0.220` | 左轮中心到右轮中心，按实车量 |
| `MOTOR_ADDR_FL/FR/RL/RR` | `1/2/3/4` | 电机地址到车轮位置的映射 |
| `MOTOR_FL_INVERT` 等 | `0` | 某轮正转方向反了就改 `1` |
| `IMU_YAW_SIGN` | `+1.0f` | 左转时 yaw 变小就改 `-1.0f` |
| `APP_REQUIRE_START_BUTTON` | `1` | 上电按 A 板 KEY(PB2) 才发车 |

### 5.2 PID

| 宏 | 默认 | 调整方向 |
|---|---|---|
| `HEADING_KP` | `2.20` | 太小回正慢，太大左右摆 |
| `HEADING_KI` | `0.15` | 消除长期偏差；先小后大 |
| `HEADING_KD` | `0.08` | 用陀螺仪阻尼，过冲大就加 |
| `HEADING_MAX_WZ_DPS` | `120` | 最大修正角速度 |
| `HEADING_DEADBAND_DEG` | `0.35` | 静止抖动大就适当加大 |

### 5.3 轨迹

| 宏 | 默认 | 说明 |
|---|---|---|
| `TRAJ_LINEAR_ACCEL_MPS2` | `0.35` | 直线加/减速度 |
| `TRAJ_ANGULAR_ACCEL_DPS2` | `90` | 圆弧角加速度 |
| `TRAJ_TURN_TOL_DEG` | `1.5` | 转到 1.5° 内算到位 |
| `TRAJ_SETTLE_MS` | `120` | 到位后稳定 120 ms 再切段 |

演示轨迹在 `Core/Src/main.c` 顶部：

```c
static const traj_segment_t g_demo_route[] =
{
    { TRAJ_SEG_STRAIGHT, 0.60f, 0.22f, 0.0f },  // 前进 0.60 m，0.22 m/s
    { TRAJ_SEG_TURN,     90.0f, 50.0f, 0.0f },  // 左转 90°，最大 50°/s
    ...
};
```

跑圆环就换成：

```c
static const traj_segment_t g_demo_route[] =
{
    { TRAJ_SEG_ARC, 360.0f, 0.22f, 0.45f },     // 左转一圈，半径 0.45 m
};
```

---

## 6. 首次上车调试步骤（重要）

> 第一次一定把车架空或留足 2 m 空间，先小速度短距离。

### 6.1 单轮点动，确认地址/方向

1. 把 `APP_WHEEL_TEST_MODE` 改成 `1`。
2. 编译烧录，上电。程序会让地址 1→2→3→4 依次以 30 RPM 转 2 秒。
3. 记录每个地址对应哪个轮子，以及“正转”时轮子是向前还是向后。
4. 填好：

```c
#define MOTOR_ADDR_FL  ...
#define MOTOR_ADDR_FR  ...
#define MOTOR_ADDR_RL  ...
#define MOTOR_ADDR_RR  ...

#define MOTOR_FL_INVERT 0  // 正转向前填 0，向后填 1
...
```

5. 改回 `APP_WHEEL_TEST_MODE 0`。

### 6.2 确认 IMU 方向

- 烧录后上电，LED 先快闪在线电机数，再闪 2 次表示 IMU 就绪，然后慢闪等待按键。
- 按 KEY 发车前，可以用 ST-Link 的 Live Watch 看 `g_imu.yaw_deg`：把车**逆时针**（左转）转 90°，`yaw_deg` 应该 **+90** 左右。
- 如果左转变成 -90，把 `IMU_YAW_SIGN` 改成 `-1.0f`；如果是倒装/旋转 180°，也可以在上位机改安装方向。

### 6.3 先跑直线

把演示轨迹临时改成：

```c
{ TRAJ_SEG_STRAIGHT, 0.40f, 0.15f, 0.0f },
```

观察：

- 车是否大致走直线；
- 走完后车头是否仍指向出发方向；
- 如果左右摆，先降 `HEADING_KP`、加 `HEADING_KD`；
- 如果回正很慢，适当加 `HEADING_KP`。

### 6.4 再跑转向

```c
{ TRAJ_SEG_TURN, 90.0f, 40.0f, 0.0f },
```

- 车应左转 90° 后停住；
- 若转过头，降 `HEADING_KP` 或加 `HEADING_KD`；
- 若转不到 90° 就停，检查 `TRAJ_TURN_TOL_DEG` 是否太大、`HEADING_DEADBAND_DEG` 是否太大。

### 6.5 最后跑整条轨迹

确认单段都没问题后，再恢复 0.6 m 正方形或圆环。  
每次发车前把车放回起点、静止 1 秒，代码会在发车瞬间重新归零 yaw。

---

## 7. LED 状态码

| LED | 含义 |
|---|---|
| 快闪 N 次（1~4） | 探测到 N 台电机在线 |
| 快闪 2 次后慢闪 | 电机 + IMU 都就绪，等待 KEY 发车 |
| 运行中 2 Hz 闪 | 正在跑轨迹 |
| 停止后 1 Hz 闪 | 空闲/轨迹完成，按 KEY 重新发车 |
| 循环闪 1 次 | CAN1 初始化失败（无法退出初始化） |
| 循环闪 2 次 | 电机少于 4 台（地址/接线/终端电阻/供电） |
| 循环闪 3 次 | IMU 未找到（CAN2 接线/波特率/ID/供电） |
| 运行中闪 4 次 | IMU 掉线，已停车；修好后按 KEY 复位重跑 |
| 运行中闪 5 次 | 轨迹超时（转向卡住/打滑/参数不对），按 KEY 复位 |

上电阶段的 1/2 是死循环报码，需要断电复位；3 会每约 2 秒重试一次（补接 IMU 后可自动恢复），运行中的 4/5 可以按 KEY 清除。

---

## 8. 常见问题

**IMU 一直闪 3 次**

1. 确认接的是 A 板 **CAN2**（PB12/PB13），不是 CAN1。
2. 确认 IMU 供电 5~28 V，CAN_H/CAN_L 没接反。
3. 上位机确认 IMU 是 CAN 模式、从机 ID 1、主机 ID 0x11、波特率 1 Mbps 或 500 kbps。
4. 总线两端 120 Ω；A 板 CAN2 到 IMU 的线不要太长。
5. 如果 IMU 之前被设成别的波特率（250K/100K），先用上位机改回 1M 或 500K。

**电机少于 4 台**

1. 驱动器地址是否 1/2/3/4。
2. CAN1 500 kbps 是否和驱动器一致。
3. 终端电阻、CAN_H/CAN_L、驱动器供电。
4. 上电顺序：先给驱动器上电，再复位 A 板。

**直线走斜 / 越走越偏**

1. 先确认 `MOTOR_*_INVERT` 和地址映射正确（单轮点动）。
2. 确认 IMU 左转时 `yaw_deg` 增大。
3. 检查麦轮左右旋是否装反；四个轮子必须按 X 型辊子安装。
4. 适当加 `HEADING_KP`。

**原地转向左右摆**

1. 降 `HEADING_KP`，加 `HEADING_KD`。
2. 加大 `HEADING_DEADBAND_DEG`（例如 0.6）。
3. 检查轮子是否打滑、地面是否太滑。

**平移方向反了**

- 麦轮辊子方向或左右轮装反了；先确认四个轮子的左/右旋型号。
- 如果只是前后反，改 `MOTOR_*_INVERT`；如果前进正常但平移反，通常是轮子装反，不要用代码硬补。

**走圆环半径不对**

- 半径由 `vx / wz` 决定，`wz` 由陀螺仪闭环，所以半径误差主要来自轮速标定和打滑。
- 先量实际轮径（含磨损），改 `WHEEL_DIAMETER_M`；再小速度试。
- `WHEEL_BASE_M` / `WHEEL_TRACK_M` 只影响前馈分配，IMU 会修，影响比轮径小。

---

## 9. 电脑端单元测试

不接开发板，验证角度展开、麦轮公式、速度规划、PID 和轨迹状态机：

```powershell
powershell -ExecutionPolicy Bypass -File tests/host/run_tests.ps1
```

期望输出：

```
All chassis_math tests passed.
All trajectory tests passed.
```

测试覆盖：

- `wrap180f` 在 ±180° 边界保持方向（180° 原地转不会反向）；
- 麦轮前进/左移/逆时针转的符号；
- 梯形速度规划能从 0 起步、到点停止；
- PID 的 P/I/D、积分限幅、输出限幅；
- 直线 → 90° 转向 → 90° 圆弧 → 等待 的完整状态机；
- 180° 原地转向方向；
- 急停。

---

## 10. 已知限制 / 后续可做

1. **直线距离是开环的**：ZDT 速度模式没有把编码器位置回传给主控，直线段按“命令速度 × 时间”积分，长距离会有几厘米误差。航向是 IMU 闭环，所以不会越走越歪。要更高精度可以：
   - 用 ZDT 的位置模式（`0xFD`）做四轮同步位置闭环；
   - 或读 4 台电机实时位置做里程计。
2. **没有视觉/灰度循迹**：当前轨迹是预设的直线/转向/圆弧。要按场地实时纠偏，可以把 OpenMV/灰度传回的横向误差叠加到 `vy` 或 `wz` 上。
3. **IMU 航向会缓慢漂移**：六轴没有磁力计，长时间运行会有零漂；每次发车前软件归零，单次比赛流程内够用。
4. **CAN 电机命令是逐台发的**：4 帧在 1 ms 内发完，已经足够同步；追求极致可以用 ZDT 的“多电机命令”一帧发完四台。
5. **系统时钟仍是 16 MHz HSI**：与原工程一致。若以后改 HSE/PLL，必须同步改 `zdt_can.h` 和 `dm_imu.c` 里的 BTR 分频。

---

## 11. 开源分支

```powershell
git checkout -b feat/dm-imu-gyro
git add .
git commit -m "feat: add DM-IMU heading closed-loop and smooth trajectory"
git push -u origin feat/dm-imu-gyro
```

原仓库：<https://github.com/fdvfvv/GongChuangSai_ZhiNengBanYun>

---

## 12. 参考资料

- DM-IMU-L1 使用说明书 V1.3（CAN 协议见附录 C）
- DM IMU 上位机使用说明书 V1.0
- ZDT_X42S 第二代闭环步进电机用户手册 V1.0.5（速度模式、CAN 帧格式）
- RoboMaster 开发板 A 型原理图（CAN2 = PB12/PB13，KEY = PB2）
