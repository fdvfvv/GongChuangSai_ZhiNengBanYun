# 工创赛智能搬运小车

RoboMaster 开发板 A 型（STM32F427）+ 4 台张大头 ZDT 闭环步进电机 + 80 mm 麦轮底盘。

## 硬件连接

| 总线 | 波特率 | 设备 |
|---|---|---|
| CAN1 | 500 kbps | 4 × ZDT 闭环步进（地址 1~4） |
| CAN2 | 1 Mbps | DM-IMU-L1 陀螺仪 |

- CAN1：`PD0` = RX，`PD1` = TX；
- CAN2：`PB12` = RX，`PB13` = TX；
- RoboMaster C610/C620 电调是 **1 Mbps**，必须接 CAN2，不能和 500K 的 ZDT 共用 CAN1。

## 编译

需要 CMake + Ninja + `arm-none-eabi-gcc`：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

烧录（ST-Link + OpenOCD）：

```powershell
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg `
  -c "program build/Debug/GongChuangSai_Project.elf verify reset exit"
```

## 分支说明

- **`main`**：基础四电机 CAN 控制例程；
- **`feat/dm-imu-gyro`**：加入 DM-IMU 航向闭环、20 cm 直行 → 右转 90° → 20 cm 直行、自动发车和运行黑匣子，**推荐使用这个分支**。

## 目录

```
Core/      应用代码
Drivers/   STM32 HAL / CMSIS
cmake/     构建脚本
archive/   开发过程日志与手册归档
```

详细说明见 `feat/dm-imu-gyro` 分支的 README。
