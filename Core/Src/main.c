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
#include "config.h"
#include "stepper.h"
#include "dma_duoblebuffer.h"
#include <stdio.h>  // 添加此行以支持 sprintf
#include <string.h> // 若使用 strlen，也应确保已包含
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define BUFFER_SIZE 256
#define SYS_CLK_HZ 72000000.0f // 系统时钟72MHz
#define MIN_CCR_VALUE 100      // 最小CCR值，对应最高频率
#define MAX_CCR_VALUE 5000     // 最大CCR值，对应最低频率（启动频率）

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

DMA_DoubleBuffer_t dma_doublebuffer;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
StepperTypeDef stepper86;

uint32_t count = 0; // 中断计数器

// 定时器中断回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    count++; // 计数
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // 翻转LED
    Stepper_process(&stepper86);
  }
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  //  printf("[TSUIC1] HAL_TIM_OC_DelayElapsedCallback!\r\n");
  if (htim->Instance == MOTOR86_PWM_TIMER)
  {
    count++; // 计数
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    Stepper_process(&stepper86);
  }
}

void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    printf("HAL_TIM_PWM_PulseFinishedHalfCpltCallback\r\n");
  }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    printf("HAL_TIM_PWM_PulseFinishedCallback\r\n");
  }
}

void generate_trapezoid_ccr(uint32_t *buffer, uint32_t start_idx, uint16_t count)
{
  uint32_t total_pulses = 50000;
  uint32_t accel_pulses = 15000;
  uint32_t decel_pulses = 15000;
  uint32_t cruise_pulses = total_pulses - accel_pulses - decel_pulses;
  // 3000/60*5000=250000Hz
  //  频率参数
  float start_freq = 1000.0f;        // 起始频率 (Hz)
  float max_freq = 3000 / 60 * 5000; // 最大频率 (Hz)
  float end_freq = 1000.0f;          // 结束频率 (Hz)

  // 计算CCR值（假设定时器时钟为72MHz）
  const uint32_t timer_clk = 72000000; // 72MHz
  // TODO: 根据加速脉冲数、匀速脉冲数和减速脉冲数，计算每个脉冲对应的频率
  const uint32_t pwm_period = 288 - 1; // ARR值

  uint32_t temp_buffer[BUFFER_SIZE]; // BUFFER_SIZE=256

  for (uint16_t i = 0; i < count; i++)
  {
    uint32_t idx = start_idx + i;
    uint32_t pulse_idx = start_idx;
    // ccr

    float current_freq;

    if (pulse_idx < accel_pulses)
    {
      // 加速阶段 - 线性加速
      float t = (float)pulse_idx / accel_pulses;
      // 使用线性加速：频率 = 起始频率 + (最大频率-起始频率)*t
      current_freq = start_freq + (max_freq - start_freq) * t;
    }
    else if (pulse_idx < (accel_pulses + cruise_pulses))
    {
      // 匀速阶段
      current_freq = max_freq;
    }
    else if (pulse_idx < total_pulses)
    {
      // 减速阶段
      float t = (float)(pulse_idx - (accel_pulses + cruise_pulses)) / decel_pulses;
      current_freq = max_freq + (end_freq - max_freq) * t;
    }
    else
    {
      // 超出总脉冲数，使用结束频率
      current_freq = end_freq;
    }

    // 计算CCR值
    // 方法1：直接计算周期时间对应的计数值
    // CCR决定脉冲宽度，通常设为周期的一半（50%占空比）
    uint32_t ccr_value;

    if (current_freq > 0)
    {
      // 计算ARR值（周期）
      uint32_t arr_value = (uint32_t)(timer_clk / current_freq);
      // CCR通常设为ARR的一半，得到50%占空比
      ccr_value = arr_value / 2;

      // 限制CCR值范围
      if (ccr_value < MIN_CCR_VALUE)
        ccr_value = MIN_CCR_VALUE;
      if (ccr_value > MAX_CCR_VALUE)
        ccr_value = MAX_CCR_VALUE;
    }
    else
    {
      ccr_value = MAX_CCR_VALUE; // 频率为0时给最大值
    }

    temp_buffer[i] = ccr_value;
  }

  memcpy(buffer, temp_buffer, count * 4);
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
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  __enable_irq();
  HAL_Delay(1000); // 等待1秒，确保系统稳定

  uint32_t arr = 9216;

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // 启动PWM输出

  /*
    PSC = 72MHz
    CCR = 72
    电机最高频率为 250KHz
    通过 DMA 修改 ARR 值，实现频率变化，从而实现电机加减速
    1、怎么通过 DMA 修改 ARR ？
    2、怎么生成一个加减速的 ARR 序列？假设函数如下

    void generate_trapezoid_ccr(uint32_t *buffer, uint32_t start_idx, uint16_t count)
    {
      uint32_t total_pulses = 50000;
      uint32_t accel_pulses = 15000;
      uint32_t decel_pulses = 15000;
      uint32_t cruise_pulses = total_pulses - accel_pulses - decel_pulses;
      // 3000/60*5000=250000Hz
      //  频率参数
      float start_freq = 1000.0f;        // 起始频率 (Hz)
      float max_freq = 3000 / 60 * 5000; // 最大频率 (Hz)
      float end_freq = 1000.0f;          // 结束频率 (Hz)

      // 计算CCR值（假设定时器时钟为72MHz）
      const uint32_t timer_clk = 72000000; // 72MHz
      // TODO: 根据加速脉冲数、匀速脉冲数和减速脉冲数，计算每个脉冲对应的频率
    }
    // 3、dma 双缓冲 怎么实现， 如何监听dma 半传输中断 和 完成传输中断？

    根据 HAL_TIM_PWM_Start_DMA 的代码
    htim->hdma[TIM_DMA_ID_CC1]->XferCpltCallback = TIM_DMADelayPulseCplt;
    htim->hdma[TIM_DMA_ID_CC1]->XferHalfCpltCallback = TIM_DMADelayPulseHalfCplt;

    // Set the DMA error callback
    htim->hdma[TIM_DMA_ID_CC1]->XferErrorCallback = TIM_DMAError ;

    // Enable the DMA channel
    if (HAL_DMA_Start_IT(htim->hdma[TIM_DMA_ID_CC1], (uint32_t)pData, (uint32_t)&htim->Instance->CCR1,
                          Length) != HAL_OK)

     我们是不是可以模仿这个流程，自己实现一个 DMA 修改 ARR 的功能？
    */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    while (arr >= 288)
    {

      __HAL_TIM_SET_AUTORELOAD(&htim3, arr - 1); // 设置ARR值，决定PWM频率
      HAL_Delay(1);
      char buffer[50];
      sprintf(buffer, "current arr: %lu\r\n", arr);
      HAL_UART_Transmit(&huart3, (uint8_t *)buffer, strlen(buffer), 100);
      arr--;
    }
    // // 每过1秒，通过串口报告中断次数
    // char buffer[50];
    // sprintf(buffer, "1s count: %lu\r\n", count);
    // HAL_UART_Transmit(&huart3, (uint8_t *)buffer, strlen(buffer), 100);

    count = 0; // 清零计数器
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
  char buffer[50];
  sprintf(buffer, "error: %lu\r\n", count);
  HAL_UART_Transmit(&huart3, (uint8_t *)buffer, strlen(buffer), 100);
  while (1)
  {
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
