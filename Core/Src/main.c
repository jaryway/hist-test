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
#include "config.h"
#include "bsp_dwin_dgus.h"

// #define MODBUS_HUART huart1 // modbus 通信 com3
// #define LCD_HUART    huart2 // lcd 屏幕通信
// #define DEBUG_HUART  huart3 // 串口调试 com4 串口调试

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
    if (htim->Instance == TIM3) {
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
    MX_GPIO_Init();
    MX_TIM3_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();
    MX_TIM2_Init();
    /* USER CODE BEGIN 2 */

    printf("System start\r\n");

    uint16_t regs[DWIN_DGUS_MAX_DATA_LEN];

    bsp_dwin_dgus_init(LCD_HUART);

    // 读取当前页面ID 5a a5 04 83 00 14 01
    bsp_dwin_dgus_read_var_regs(0x14, 0x01, &regs[0], 0, 1000);

    // 打印regs
    for (int i = 0; i < 0x01; i++) {
        printf("%02x ", regs[i]);
    }
    // printf("Page ID: %02x %02x %02x %02x %02x %02x %02x\r\n", regs[0], regs[1], regs[2], regs[3], regs[4], regs[5], regs[6]);

    // HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    // motor_pwm_init(&motor_pwm, &htim3, MOTOR42_PWM_TIMER_CHANNEL, DIRE_PORT, DIRE_PIN, ENAB_PORT, ENAB_PIN);

    // float accel_time = 1.0f;                              // 期望1秒完成速度变化
    // uint32_t speed   = 600.0f / 60.0f * SPR;              // 根据转速计算速度
    // uint32_t accel   = (1000.0f / 60 * SPR) / accel_time; // a = v/t
    // uint32_t decel   = accel;                             // 减速度与加速度相同

    // motor_pwm_set_accel(&motor_pwm, accel);
    // motor_pwm_set_decel(&motor_pwm, decel);
    // motor_pwm_set_speed(&motor_pwm, speed);

    // motor_pwm_move(&motor_pwm, 100000);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
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
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
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
