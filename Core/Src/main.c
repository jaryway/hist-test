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
#include "dma_channel_probe.h"
#include <stdio.h>  // 添加此行以支持 sprintf
#include <string.h> // 若使用 strlen，也应确保已包含
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define BUFFER_SIZE 512
#define SYS_CLK_HZ 72000000.0f // 系统时钟72MHz

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

DMA_DoubleBuffer_t dma_doublebuffer = {.total_pulses = 50000};
extern DMA_HandleTypeDef hdma_tim3_ch1_trig;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
StepperTypeDef stepper86;

uint32_t count = 0; // 中断计数器
uint32_t half_count = 0;
uint32_t full_count = 0;
uint32_t oc_half_count = 0;
uint32_t oc_full_count = 0;

int _write(int file, char *ptr, int len)
{
  (void)file;
  // 在中断上下文中使用非阻塞方式
  if (__get_IPSR() != 0)
  {
    // 在中断中，使用较短的超时时间
    HAL_UART_Transmit(&huart3, (uint8_t *)ptr, len, 100);
  }
  else
  {
    // 在主程序中，使用阻塞方式
    HAL_UART_Transmit(&huart3, (uint8_t *)ptr, len, HAL_MAX_DELAY);
  }
  return len;
}
/* Diagnostic dump: print TIM3 PSC/ARR and sample DMA buffer deltas (first N entries) */
void diag_print_timer_and_buffer(uint16_t *buf, int len)
{
  uint32_t timclk = HAL_RCC_GetPCLK1Freq();
  if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0)
    timclk *= 2; // F1: APB1 prescaler !=1 => timerclk*2
  uint32_t psc =htim3.Init.Prescaler;
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim3);
  double tick_hz = (double)timclk / (double)(psc + 1U);

  printf("TIM3: timclk=%lu PSC=%lu ARR=%lu tick_hz=%.0f (tick_us=%.3f)\n",
         (unsigned long)timclk, (unsigned long)psc, (unsigned long)arr, tick_hz, 1e6 / tick_hz);

  int N = (len < 16) ? len : 16;
  printf("First %d buffer entries and deltas:\n", N);
  for (int i = 0; i < N; ++i)
  {
    uint16_t v = buf[i];
    uint16_t next = buf[(i + 1) % len];
    uint16_t delta = (uint16_t)(next - v); // modulo 2^16
    double delta_us = (double)delta * (1e6 / tick_hz);
    double freq = 1.0 / (delta_us * 1e-6); // Hz for one delta period
    printf("buf[%02d]=%5u  next=%5u  delta=%5u ticks  ~=%.3f us  freq=%.1f Hz\n",
           i, (unsigned int)v, (unsigned int)next, (unsigned int)delta, delta_us, freq);
  }
}

void HAL_TIM_PeriodElapsedHalfCpltCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    half_count++;
    if (dma_doublebuffer.active_buffer == 0)
    {
      dma_doublebuffer.next_fill_buffer = 1;
    }
    else if (dma_doublebuffer.active_buffer == 1)
    {
      dma_doublebuffer.next_fill_buffer = 0;
    }
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    full_count++;
    uint32_t temp = dma_doublebuffer.pulses_sent + BUFFER_SIZE;

    // 判断脉冲是否发送完成，如果已经发送完成，停止DMA传输
    if (temp >= dma_doublebuffer.total_pulses)
    {
      dma_doublebuffer.pulses_sent = dma_doublebuffer.total_pulses; // 重新修正发送位置
      HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
      HAL_TIM_OC_Stop_DMA(&htim3, TIM_CHANNEL_1);
      return;
    }

    dma_doublebuffer.pulses_sent = temp;
    uint16_t *pData;
    // 切换缓冲区
    if (dma_doublebuffer.active_buffer == 0)
    {
      pData = dma_doublebuffer.dma_buf1;
      dma_doublebuffer.active_buffer = 1;
    }
    else if (dma_doublebuffer.next_fill_buffer == 1)
    {
      pData = dma_doublebuffer.dma_buf0;
      dma_doublebuffer.next_fill_buffer = 0;
    }

    HAL_TIM_OC_Start_DMA(&htim3, TIM_CHANNEL_1, (uint32_t *)pData, BUFFER_SIZE);
  }
}
void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    oc_half_count++;
    if (dma_doublebuffer.active_buffer == 0)
    {
      dma_doublebuffer.next_fill_buffer = 1;
    }
    else if (dma_doublebuffer.active_buffer == 1)
    {
      dma_doublebuffer.next_fill_buffer = 0;
    }
  }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    oc_full_count++;
    uint32_t temp = dma_doublebuffer.pulses_sent + BUFFER_SIZE;

    // 判断脉冲是否发送完成，如果已经发送完成，停止DMA传输
    if (temp >= dma_doublebuffer.total_pulses)
    {
      dma_doublebuffer.pulses_sent = dma_doublebuffer.total_pulses; // 重新修正发送位置
      HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
      HAL_TIM_OC_Stop_DMA(&htim3, TIM_CHANNEL_1);
      return;
    }

    dma_doublebuffer.pulses_sent = temp;
    uint16_t *pData;
    // 切换缓冲区
    if (dma_doublebuffer.active_buffer == 0)
    {
      pData = dma_doublebuffer.dma_buf1;
      dma_doublebuffer.active_buffer = 1;
    }
    else if (dma_doublebuffer.next_fill_buffer == 1)
    {
      pData = dma_doublebuffer.dma_buf0;
      dma_doublebuffer.next_fill_buffer = 0;
    }

    HAL_TIM_OC_Start_DMA(&htim3, TIM_CHANNEL_1, (uint32_t *)pData, BUFFER_SIZE);
  }
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

  printf("System start\r\n");

  // static uint16_t dma_buffer[] = {4608 * 2, 4608, 2304, 1152, 576, 288, 144};
  uint16_t length = sizeof(dma_doublebuffer.dma_buf0) / sizeof(dma_doublebuffer.dma_buf0[0]);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  init_double_buffer(&dma_doublebuffer, &htim3);

  HAL_TIM_OC_Stop_DMA(&htim3, TIM_CHANNEL_1);
  HAL_TIM_OC_Start_DMA(&htim3, TIM_CHANNEL_1, (uint32_t *)dma_doublebuffer.dma_buf0, length);

  printf("DMA started\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // 测试生成的ccr是否这个正确

    // uint32_t pulse_index = 0;
    // while (pulse_index <= dma_doublebuffer.total_pulses)
    // {
    //   uint32_t period_ticks = generate_trapezoid_period_ticks(pulse_index);
    //    uint32_t ccr = generate_trapezoid_ccr(pulse_index);
    //   printf("pulse_index: %lu,ccr:%lu\r\n", pulse_index, ccr);
    //   pulse_index++;
    // }

    // HAL_Delay(10000);

    // uint16_t arr = __HAL_TIM_GET_AUTORELOAD(&htim3);
    // printf("full_count: %lu, half_count: %lu,arr:%d\r\n", full_count, half_count, arr);

    // uint16_t ccr = __HAL_TIM_GET_COUNTER(&htim3);
    // printf("oc_half_count: %lu, oc_full_count: %lu,ccr:%d\r\n", oc_half_count, oc_full_count, ccr);
    diag_print_timer_and_buffer(dma_doublebuffer.dma_buf0, BUFFER_SIZE);

    fill_buffer_in_background(&dma_doublebuffer);
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
