/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ENA_Pin GPIO_PIN_3
#define ENA_GPIO_Port GPIOB
#define DIR_Pin GPIO_PIN_5
#define DIR_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

typedef struct {
    uint16_t travel_distance;  // 导轨有效行程 mm
    uint16_t distance_per_rev; // 导轨同步轮转一周的长度 mm
    uint8_t reduction_ratio;   // 减速比
    uint16_t max_rpm;          // 电机额定转速 RPM
    uint16_t steps_per_rev;    // 电机转一圈所需的步数 3200步/圈
    float accel_time;        // 期望加速到最大速度所需时间 s
    float decel_time;        // 期望减速到停止所需时间 s

} Profile_t;

typedef struct {
    int32_t pulses; // 总步数
    uint32_t accel; // 加速度 rad/s² X10 后
    uint32_t decel; // 加速度 rad/s² X10 后
    uint32_t max_speed; // 速度 rad/s X10 后
} TCtrlParam_t;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
