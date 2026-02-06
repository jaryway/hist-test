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
#define SENSOE_LD2_Pin GPIO_PIN_13
#define SENSOE_LD2_GPIO_Port GPIOC
#define SENSOR_LD1_Pin GPIO_PIN_14
#define SENSOR_LD1_GPIO_Port GPIOC
#define ENA_Pin GPIO_PIN_15
#define ENA_GPIO_Port GPIOC
#define PUL_Pin GPIO_PIN_0
#define PUL_GPIO_Port GPIOA
#define PUL1_Pin GPIO_PIN_1
#define PUL1_GPIO_Port GPIOA
#define DIR_Pin GPIO_PIN_4
#define DIR_GPIO_Port GPIOA
#define SENSOR_DN_Pin GPIO_PIN_5
#define SENSOR_DN_GPIO_Port GPIOA
#define ENAB_Pin GPIO_PIN_7
#define ENAB_GPIO_Port GPIOA
#define DIRE_Pin GPIO_PIN_0
#define DIRE_GPIO_Port GPIOB
#define STEP_Pin GPIO_PIN_1
#define STEP_GPIO_Port GPIOB
#define BTN_UP_Pin GPIO_PIN_12
#define BTN_UP_GPIO_Port GPIOB
#define BTN_DN_Pin GPIO_PIN_13
#define BTN_DN_GPIO_Port GPIOB
#define BTN_ST_Pin GPIO_PIN_14
#define BTN_ST_GPIO_Port GPIOB
#define BTN_LED_UP_Pin GPIO_PIN_15
#define BTN_LED_UP_GPIO_Port GPIOB
#define BTN_LED_DN_Pin GPIO_PIN_8
#define BTN_LED_DN_GPIO_Port GPIOA
#define RS485_RE_Pin GPIO_PIN_11
#define RS485_RE_GPIO_Port GPIOA
#define BTN_LED_ST_Pin GPIO_PIN_12
#define BTN_LED_ST_GPIO_Port GPIOA
#define BTN_SP_Pin GPIO_PIN_9
#define BTN_SP_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

typedef struct {
    uint16_t travel_distance;  // 导轨有效行程 mm
    uint16_t distance_per_rev; // 导轨同步轮转一周的长度 mm
    uint8_t reduction_ratio;   // 减速比
    uint16_t max_rpm;          // 电机额定转速 RPM
    uint16_t steps_per_rev;    // 电机转一圈所需的步数 3200步/圈
    float accel_time;          // 期望加速到最大速度所需时间 s
    float decel_time;          // 期望减速到停止所需时间 s

} Profile_t;

typedef struct {
    int32_t pulses;       // 总步数
    uint32_t accel;       // 加速度 rad/s² X10 后
    uint32_t decel;       // 加速度 rad/s² X10 后
    uint32_t max_speed;   // 速度 rad/s X10 后
    uint32_t est_time_ms; // 预计运动时间 ms
} TCtrlParam_t;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
