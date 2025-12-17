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
#include "stdio.h"
#include "dma_doublebuffer.h"
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
DMA_DoubleBuffer_t dma_doublebuffer = {
    .mode = PWM_ARR,
    .htim = &htim1,
    .tim_channel = TIM_CHANNEL_1,
    .total_pulses = 150 * 3, //
    .accel_pulses = 150,
    .decel_pulses = 150,
    .rpm = 3000,
    .pulses_per_rev = 5000,
    .active_buffer = 0,
};
DMA_DoubleBuffer_t dma_doublebuffer_oc = {
    .mode = OC_CCR,
    .htim = &htim3,
    .tim_channel = TIM_CHANNEL_1,
    .total_pulses = 150 * 3, //
    .accel_pulses = 150,
    .decel_pulses = 150,
    .rpm = 3000,
    .pulses_per_rev = 5000,
    .active_buffer = 0,
};

static uint32_t half_count = 0;
static uint32_t finish_count = 0;
static uint32_t half_count_oc = 0;
static uint32_t finish_count_oc = 0;
static uint8_t half_count_changed = 0;
static uint32_t last_time = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief 检查脉冲是否发送完成 */

uint8_t check_pulse_finished(TIM_HandleTypeDef *htim, DMA_DoubleBuffer_t *dma_doublebuffer)
{
  uint32_t temp = dma_doublebuffer->pulses_sent + BUFFER_SIZE / 2;

  // 判断脉冲是否发送完成，如果已经发送完成，停止DMA传输
  if (temp >= dma_doublebuffer->total_pulses)
  {
    dma_doublebuffer->pulses_sent = dma_doublebuffer->total_pulses; // 重新修正发送位置
    if (dma_doublebuffer->mode == PWM_ARR)
    {
      HAL_TIM_PWM_Stop(htim, dma_doublebuffer->tim_channel);
      HAL_TIM_Base_Stop_DMA(htim);
    }
    else if (dma_doublebuffer->mode == OC_CCR)
    {
      HAL_TIM_OC_Stop_DMA(htim, dma_doublebuffer->tim_channel);
    }
    return 1;
  }

  dma_doublebuffer->pulses_sent = temp;
  return 0;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    // uint32_t arr = __HAL_TIM_GET_AUTORELOAD(dma_doublebuffer.htim);
    // printf("HAL_TIM_PeriodElapsedCallback-tim1-arr: %lu\r\n", (unsigned long)arr);
    finish_count++;
    if (check_pulse_finished(htim, &dma_doublebuffer))
      return;
  }
  else if (htim->Instance == TIM2)
  {
    printf("HAL_TIM_PeriodElapsedCallback-TIM2\r\n");
  }
}

void HAL_TIM_PeriodElapsedHalfCpltCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    half_count++;
    half_count_changed = 1;

    // HAL_TIM_Base_Stop_DMA(htim);
    // DMA_HandleTypeDef *hdma = htim->hdma[TIM_DMA_ID_UPDATE];
    // if (hdma == NULL)
    //   return;
    // uint32_t par = (uint32_t)hdma->Instance->CPAR;           /* DMA 外设地址 */
    // uint32_t mar = (uint32_t)hdma->Instance->CMAR;           /* DMA 内存地址 */
    // uint32_t cndtr = (uint32_t)hdma->Instance->CNDTR;        /* 剩余要传的数据项数 */
    // uint32_t arr = (uint32_t)__HAL_TIM_GET_AUTORELOAD(htim); /* TIM 当前 ARR */

    // uint32_t transferred = BUFFER_SIZE - cndtr; /* 已经传送的元素数 */
    // uint32_t last_idx = (transferred == 0) ? (BUFFER_SIZE - 1) : (transferred - 1);
    // uint32_t next_idx = transferred % BUFFER_SIZE;

    // uint16_t last_val = dma_doublebuffer.dma_buffer[last_idx]; /* DMA 最近写入的内存值 */
    // uint16_t next_val = dma_doublebuffer.dma_buffer[next_idx]; /* 下一个将被写的内存值 */

    // printf("HT: CPAR=0x%08lX CMAR=0x%08lX CNDTR=%lu transferred=%lu ARR=%lu\r\n",
    //        (unsigned long)par,
    //        (unsigned long)mar,
    //        (unsigned long)cndtr,
    //        (unsigned long)transferred, //
    //        (unsigned long)arr);
    // printf("HT: last_idx=%lu last_val=%u next_idx=%lu next_val=%u\r\n",
    //        (unsigned long)last_idx,
    //        (unsigned)last_val,
    //        (unsigned long)next_idx, //
    //        (unsigned)next_val);

    if (check_pulse_finished(htim, &dma_doublebuffer))
      return;

    // 切换缓冲区
    if (dma_doublebuffer.active_buffer == 0)
    {
      dma_doublebuffer.active_buffer = 1;
    }
    else if (dma_doublebuffer.active_buffer == 1)
    {
      dma_doublebuffer.active_buffer = 0;
    }
  }
  else if (htim->Instance == TIM2)
  {
    //
    // printf("HAL_TIM_PeriodElapsedHalfCpltCallback-tim2\r\n");
  }
  else if (htim->Instance == TIM3)
  {
    //
    // printf("HAL_TIM_PeriodElapsedHalfCpltCallback-tim2\r\n");
  }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  printf("HAL_TIM_PWM_PulseFinishedCallback\r\n");
}

void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim)
{
  printf("HAL_TIM_PWM_PulseFinishedHalfCpltCallback\r\n");
}
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    printf("HAL_TIM_OC_DelayElapsedCallback-tim3\r\n");
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, __HAL_TIM_GET_COUNTER(&htim3) + 100);
  }
}
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */

  __enable_irq();
  HAL_Delay(1000); // 等待1秒，确保系统稳定

  printf("System start\r\n");

  // init_double_buffer(&dma_doublebuffer);

  // uint16_t length = sizeof(dma_doublebuffer.dma_buffer) / sizeof(dma_doublebuffer.dma_buffer[0]);
  // printf("length=%u ,pulses_filled:%lu\r\n", length, dma_doublebuffer.pulses_filled);
  // for (uint32_t i = 0; i <= dma_doublebuffer.total_pulses; i++)
  // {
  //   uint32_t arr = generate_trapezoid_arr(&dma_doublebuffer, i);
  //   printf("pulse_index:%03lu, arr: %lu\n", i, arr);
  // }

  // // PWM+DMA 修改 ARR
  // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  // HAL_TIM_Base_Start_DMA(&htim1, (uint32_t *)dma_doublebuffer.dma_buffer, length);

  HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, GPIO_PIN_RESET);

  init_double_buffer(&dma_doublebuffer_oc);

  uint16_t length = sizeof(dma_doublebuffer_oc.dma_buffer) / sizeof(dma_doublebuffer_oc.dma_buffer[0]);
  printf("length=%u ,pulses_filled:%lu\r\n", length, dma_doublebuffer_oc.pulses_filled);
  for (uint32_t i = 0; i <= dma_doublebuffer_oc.total_pulses; i++)
  {
    uint32_t arr = generate_trapezoid_ccr(&dma_doublebuffer_oc, i);
    printf("pulse_index:%03lu, ccr: %lu\r\n", i, arr);
  }
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
  // HAL_TIM_OC_Start_DMA(&htim3, TIM_CHANNEL_1, (uint32_t *)dma_doublebuffer_oc.dma_buffer, length);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

    // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, __HAL_TIM_GET_COUNTER(&htim3) + 100);

    printf("cnt: %lu,ccr: %lu\r\n", __HAL_TIM_GET_COUNTER(&htim3), __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1));

    if (half_count_changed)
    {
      // HAL_Delay(500);
      // if (HAL_GetTick() - last_time >= 1000)
      // {
      half_count_changed = 0;
      printf("half_count: %lu, finish_count:%lu\r\n", half_count, finish_count);
      last_time = HAL_GetTick();
    }
  }

  // }
  fill_buffer_in_background(&dma_doublebuffer);
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
