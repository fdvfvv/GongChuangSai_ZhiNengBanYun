/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* 诊断变量: SysTick 中断独立计数器 + tick 停滞现场记录 (ST-Link 可直读) */
extern volatile uint32_t g_irq_systick;   /* SysTick_Handler 真实进入次数 */
extern volatile uint32_t g_stuck_primask; /* 发现 tick 停滞时的 PRIMASK */
extern volatile uint32_t g_stuck_tick;    /* 停滞时的 uwTick 快照 */
extern volatile uint32_t g_stuck_pc;      /* 停滞时的 PC (调用点附近) */

/* X42S 串口测试诊断变量(USART6=PG14/PG9, 115200 8N1) */
extern volatile uint32_t g_ser_round;          /* 循环轮数 */
extern volatile uint32_t g_ser_status_ok;      /* 读状态是否成功 (0/1) */
extern volatile uint32_t g_ser_status_byte;    /* 读到的状态字节 */
extern volatile uint32_t g_ser_enable_ok;      /* 使能命令是否成功 */
extern volatile uint32_t g_ser_speed_set_ok;   /* 设速命令是否成功 */
extern volatile uint32_t g_ser_speed_read_ok;  /* 读速成功累计次数 */
extern volatile int32_t  g_ser_speed_value;    /* 最近一次读到的转速 (RPM) */
extern volatile uint32_t g_ser_nonzero;        /* 是否读到过非零转速 */

/* 地址扫描诊断: 扫 1~32 找哪个地址能正确回 read_status */
extern volatile uint32_t g_ser_scan_count;     /* 有响应的地址数 */
extern volatile uint32_t g_ser_scan_found[33]; /* found[1..32]=1 表示该地址回了包 */

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
