/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "dma_db.h"
#include "helper.h"
#include "bsp_modbus_master.h"
#include "motor_mb.h"
#include "motor_pwm.h"

#define MODBUS_HUART huart1 // modbus 通信 com3
#define LCD_HUART    huart2 // lcd 屏幕通信
#define DEBUG_HUART  huart3 // 串口调试 com4 串口调试

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PI         3.1415926 /* 圆周率*/
#define FSPR       200       /* 步进电机单圈步数 */
#define MICRO_STEP 16        /* 步进电机驱动器细分数 */

// #define OC_IT      0
// #define OC_DMA     1
// #define PWM_IT     2
// #define PWM_DMA    3
// #define RUN_MODE   OC_DMA

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

MotorPWM_t motor_pwm; // 添加电机实例

// Profile_t motor42_profile = {
//     .max_rpm          = 1000,     // 最高转速
//     .steps_per_rev    = 200 * 16, // 16细分
//     .reduction_ratio  = 1,        // 减速比
//     .accel_time       = 0.5,      // 加速时间 ms
//     .decel_time       = 0.3,      // 减速时间 ms
//     .travel_distance  = 450 * 3,  // 导轨有效行程
//     .distance_per_rev = 40,       // T2-20 齿,转一周周长:20*2=40mm
// };

// Profile_t servo_profile = {
//     .max_rpm          = 3000,     // 最高转速
//     .steps_per_rev    = 200 * 16, // 16细分
//     .reduction_ratio  = 12,       // 减速比
//     .accel_time       = 1,        // 加速时间 ms
//     .decel_time       = 0.8,      // 减速时间 ms
//     .travel_distance  = 1650,     // 导轨有效行程
//     .distance_per_rev = 125,      // 同步轮 T5-25 转一周周长
// };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        // printf("HAL_TIM_PWM_PulseFinishedCallback\r\n");
        // motor_pwm_tim_callback(&motor_pwm);
    }
}

void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        // printf("HAL_TIM_PWM_PulseFinishedHalfCpltCallback\r\n");
    }
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        // printf("HAL_TIM_OC_DelayElapsedCallback\r\n");
        // motor_pwm_tim_callback(&motor_pwm);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        motor_pwm_tim_callback(&motor_pwm);
        // printf("HAL_TIM_PeriodElapsedCallback\r\n");
    }
}

void print_motor_status(MotorPWM_t *motor)
{
    const char *state_str[] = {"STOP", "ACCEL", "CONST", "DECEL", "FINISHED"};

    // 计算实际 RPM
    float current_rpm = (motor->current_speed * 60.0f) / SPR;
    float target_rpm  = (motor->speed * 60.0f) / SPR;

    printf("Pos:%7ld | State:%-6s | Speed:%5lu/%5lu (%4.0f/%4.0f RPM) | Step:%6lu\r\n",
           motor_pwm_get_current_pos(motor),
           state_str[motor_pwm_get_state(motor)],
           motor->current_speed,
           motor->speed,
           current_rpm,
           target_rpm,
           motor->step_count);
}

int _write(int file, char *ptr, int len)
{
    (void)file;
    // 在中断上下文中使用非阻塞方式
    if (__get_IPSR() != 0) {
        // 在中断中，使用较短的超时时间
        HAL_UART_Transmit(&huart3, (uint8_t *)ptr, len, 200);
    } else {
        // 在主程序中，使用阻塞方式
        HAL_UART_Transmit(&huart3, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    }
    return len;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_DMA_Init();
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

    printf("System start\r\n");

    /*
    // #define SPR                        800                   // 旋转一圈需要的脉冲数
    // #define RPM                        3000                  // 电机额定转速
    // #define TOTAL_STEPS                1650 / 125 * 12 * SPR // 总步数= 导轨行程/同步轮周长*减速比*每圈脉冲数
    // #define SPEED                      RPM / 60.0f * SPR     // 电机速度
    // #define ACCEL                      (SPEED / 1.0f)        // 加速度 公式 a=v/t
    // #define DECEL                      (SPEED / 0.8f)        // 减速度 公式 a=v/t
    */

    // // 1、初始化电机
    // motor_pwm_init(&motor_pwm,
    //                &htim2,                  // 使用 TIM2
    //                TIM_CHANNEL_1,           // 使用通道1
    //                DIR_GPIO_Port, DIR_Pin,  // 方向引脚
    //                ENA_GPIO_Port, ENA_Pin); // 使能引脚

    // uint32_t speed = 200 / 60.0f * SPR;
    // uint32_t accel = 10000 / 3.0f;
    // uint32_t decel = 10000 / 3.0f;
    // motor_pwm_set_speed(&motor_pwm, speed);
    // motor_pwm_set_accel(&motor_pwm, accel);
    // motor_pwm_set_decel(&motor_pwm, decel);

    // motor_pwm_move(&motor_pwm, 1000000);
    // HAL_Delay(8000);

    // speed = 2000 / 60.0f * SPR;
    // motor_pwm_set_speed(&motor_pwm, speed);
    // motor_pwm_move(&motor_pwm, 2000000);

    // HAL_Delay(8000);

    // speed = 500 / 60.0f * SPR;
    // motor_pwm_set_speed(&motor_pwm, speed);
    // motor_pwm_move(&motor_pwm, 2000000);

    // HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    // uint16_t arr = 1000000 / (500.0f / 60.0f * 800);
    // __HAL_TIM_SET_AUTORELOAD(&htim2, arr);
    // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, arr / 2);

    // while (1) {
    //     /* code */
    // }

    // 1、初始化电机
    motor_pwm_init(&motor_pwm,
                   &htim2,                  // 使用 TIM2
                   TIM_CHANNEL_1,           // 使用通道1
                   DIR_GPIO_Port, DIR_Pin,  // 方向引脚
                   ENA_GPIO_Port, ENA_Pin); // 使能引脚

    // 2、定义速度范围
    uint32_t speed_min = 200.0f / 60.0f * SPR; // 200 RPM = 10667 steps/s
    uint32_t speed_mid = 500.0f / 60.0f * SPR;
    uint32_t speed_max = 2000.0f / 60.0f * SPR; // 2000 RPM = 106667 steps/s

    // 3、根据最大速度差和期望时间计算加速度
    float accel_time    = 1.0f;                           // 期望3秒完成速度变化
    uint32_t speed_diff = speed_max - speed_min;          // 96000 steps/s
    uint32_t accel      = (3000 / 60 * SPR) / accel_time; // a = v/t
    uint32_t decel      = accel;                          // 减速度与加速度相同

    printf("===========================================\r\n");
    printf("Speed range: %lu - %lu steps/s\r\n", speed_min, speed_max);
    printf("Speed difference: %lu steps/s\r\n", speed_diff);
    printf("Accel/Decel: %lu steps/s² (for %0.1f sec transition)\r\n", accel, accel_time);
    printf("===========================================\r\n");

    // 4、设置参数
    motor_pwm_set_accel(&motor_pwm, accel);
    motor_pwm_set_decel(&motor_pwm, decel);
    motor_pwm_set_speed(&motor_pwm, speed_min);

    // 5、开始运动
    printf(">>> Starting motion at 200 RPM\r\n");
    motor_pwm_move(&motor_pwm, 10000);

    // HAL_Delay(5000); // 等待5秒

    // // 6、运行中改变速度到 2000 RPM
    // printf(">>> Changing speed to 2000 RPM (should take ~3 sec)\r\n");
    // uint32_t change_start = HAL_GetTick();
    // motor_pwm_set_speed(&motor_pwm, speed_max);

    // HAL_Delay(5000); // 等待8秒

    // printf(">>> Changing speed to 100 RPM (should take ~3 sec)\r\n");
    // change_start = HAL_GetTick();
    // motor_pwm_set_speed(&motor_pwm, speed_mid);

    // HAL_Delay(3000);
    // change_start = HAL_GetTick();
    // motor_pwm_set_speed(&motor_pwm, 1000.0f / 60.0f * SPR);

    // HAL_Delay(3000);
    // change_start = HAL_GetTick();
    // motor_pwm_set_speed(&motor_pwm, 1500.0f / 60.0f * SPR);

    // HAL_Delay(3000);
    // change_start = HAL_GetTick();
    // motor_pwm_set_speed(&motor_pwm, 500.0f / 60.0f * SPR);

    // HAL_Delay(3000);
    // change_start = HAL_GetTick();
    // motor_pwm_set_speed(&motor_pwm, 2200.0f / 60.0f * SPR);

    // HAL_Delay(3000);
    // change_start = HAL_GetTick();
    // motor_pwm_set_speed(&motor_pwm, 22.0f / 60.0f * SPR);

    // uint32_t change_time = HAL_GetTick() - change_start;
    // printf(">>> Speed change completed in %lu ms\r\n", change_time);

    // // 7、改变目标位置
    // motor_pwm_set_speed(&motor_pwm, 1000.0f / 60.0f * SPR);
    // printf(">>> Changing target to 2000000\r\n");
    // motor_pwm_move_to(&motor_pwm, -1000000);

    // HAL_Delay(5000);
    // motor_pwm_move_to(&motor_pwm, 1000000);
    // HAL_Delay(5000);
    // // 8、再次改变速度到 500 RPM
    // printf(">>> Changing speed to 500 RPM\r\n");
    // // uint32_t speed_mid = 500.0f / 60.0f * SPR;
    // motor_pwm_set_speed(&motor_pwm, 2000.0f / 60.0f * SPR);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    uint32_t last_print = 0;
    uint8_t m           = 1;

    while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

        // motor_pwm_move_to(&motor_pwm, m == 1 ? -1000000 : 1000000);
        // m = !m;
        // HAL_Delay(5000);

        // // 每200ms打印一次状态
        // if (HAL_GetTick() - last_print >= 200) {
        //     last_print = HAL_GetTick();
        //     print_motor_status(&motor_pwm);
        // }

        // HAL_Delay(10);
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
