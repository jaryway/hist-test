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
#include "dma_doublebuffer.h"

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
// #define BUFFER_SIZE 1024
static uint16_t dma_buffer[BUFFER_SIZE];
static uint32_t g_ccr32 = 0;
volatile uint8_t active_buffer = 0;
volatile uint8_t next_fill_buffer = 0;
static uint32_t half_count = 0;
static uint32_t finished_count = 0;
static uint8_t has_count_changed = 0;

DMA_DoubleBuffer_t dma_doublebuffer_oc = {
    .mode = OC_CCR,
    .htim = &htim3,
    .tim_channel = TIM_CHANNEL_1,
    .hdma_id = TIM_DMA_ID_CC1,
    .total_pulses = 256 * 10000 + 100, // 总脉冲
    .accel_pulses = 64,                // 加速脉冲
    .decel_pulses = 64,                // 减速脉冲
    .max_rpm = 1000,                   // 电机最高转速
    .pulses_per_rev = 3200,            // 16 细分
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void _fill_buffer()
{
  // uint16_t half_size = BUFFER_SIZE / 2;
  // uint16_t ccr = 0;

  // for (uint16_t i = 0; i < half_size; i++)
  // {
  //   g_ccr32 += 4;
  //   ccr = (uint16_t)(g_ccr32 & 0xFFFF);

  //   if (next_fill_buffer == 0)
  //   {
  //     dma_buffer[i] = ccr; // 填充前半区
  //   }
  //   else if (next_fill_buffer == 1)
  //   {
  //     dma_buffer[i + half_size] = ccr; // 填充后半区
  //   }
  // }
  // next_fill_buffer = 0xFF;

  /*  填充双缓冲区 */

  uint16_t half_size = BUFFER_SIZE / 2;
  uint16_t temp_buffer[half_size];

  for (uint16_t i = 0; i < half_size; i++)
  {
    g_ccr32 += 4;
    temp_buffer[i] = (uint16_t)(g_ccr32 & 0xFFFF);
  }

  if (next_fill_buffer == 0)
  {
    memcpy(&dma_buffer[0], temp_buffer, half_size * sizeof(uint16_t)); // 填充前半区
  }
  else if (next_fill_buffer == 1)
  {
    memcpy(&dma_buffer[half_size], temp_buffer, half_size * sizeof(uint16_t)); // 填充后半区
  }

  next_fill_buffer = 0xFF;
}

void _fill_buffer_in_background()
{
  // dma_doublebuffer->fill_buffer_in_background_count++;
  if (next_fill_buffer == 0xFF)
    return;

  _fill_buffer();

  next_fill_buffer = 0xFF;
}

void _switch_buffer()
{
  if (active_buffer == 0)
  {
    active_buffer = 1;
    next_fill_buffer = 0;
  }
  else
  {
    active_buffer = 0;
    next_fill_buffer = 1;
  }
}
void prinf_dma_info(TIM_HandleTypeDef *htim, DMA_DoubleBuffer_t *dma_doublebuffer)
{
  DMA_HandleTypeDef *hdma = htim->hdma[TIM_DMA_ID_CC1];
  if (hdma != NULL)
  {
    uint32_t par = (uint32_t)hdma->Instance->CPAR;                                       /* DMA 外设地址 */
    uint32_t mar = (uint32_t)hdma->Instance->CMAR;                                       /* DMA 内存地址 */
    uint32_t cndtr = (uint32_t)hdma->Instance->CNDTR;                                    /* 剩余要传的数据项数 */
    uint32_t ccr = (uint32_t)__HAL_TIM_GET_COMPARE(htim, dma_doublebuffer->tim_channel); /* TIM 当前 CCR */

    uint32_t transferred = BUFFER_SIZE - cndtr; /* 已经传送的数据量 */
    uint32_t last_idx = (transferred == 0) ? (BUFFER_SIZE - 1) : (transferred - 1);
    uint32_t next_idx = transferred % BUFFER_SIZE;

    uint16_t last_val = dma_doublebuffer->dma_buffer[last_idx]; /* DMA 最近写入的内存值 */
    uint16_t next_val = dma_doublebuffer->dma_buffer[next_idx]; /* 下一个将被写的内存值 */

    printf("CPAR=0x%08lX CMAR=0x%08lX CNDTR=%lu transferred=%lu CCR=%lu\r\n",
           (unsigned long)par,
           (unsigned long)mar,
           (unsigned long)cndtr,
           (unsigned long)transferred, //
           (unsigned long)ccr);
    printf("last_idx=%lu last_val=%u next_idx=%lu next_val=%u\r\n",
           (unsigned long)last_idx,
           (unsigned)last_val,
           (unsigned long)next_idx, //
           (unsigned)next_val);
  }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    // printf("HAL_TIM_PWM_PulseFinishedCallback\r\n");
    // _switch_buffer();
    finished_count++;
    has_count_changed = 1;
    // prinf_dma_info(htim, &dma_doublebuffer_oc);

    if (dma_doublebuffer_check_finished(&dma_doublebuffer_oc))
      return;

    // dma_doublebuffer_check_and_adjust(&dma_doublebuffer_oc);

    dma_doublebuffer_switch(&dma_doublebuffer_oc);
  }
}

void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    // printf("HAL_TIM_PWM_PulseFinishedHalfCpltCallback\r\n");
    half_count++;
    has_count_changed = 1;
    // prinf_dma_info(htim, &dma_doublebuffer_oc);

    if (dma_doublebuffer_check_finished(&dma_doublebuffer_oc))
      return;

    dma_doublebuffer_switch(&dma_doublebuffer_oc);

    // _switch_buffer();
    // _fill_buffer();
  }
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    // printf("HAL_TIM_OC_DelayElapsedCallback-tim3\r\n");
    // last_time_oc += 1200;
    // __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, last_time_oc);
    uint32_t cnt = __HAL_TIM_GET_COUNTER(htim);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, cnt + 4);
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
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  printf("System start\r\n");

  // uint32_t cnt = __HAL_TIM_GET_COUNTER(&htim3);
  // g_ccr32 = __HAL_TIM_GET_COUNTER(&htim3);

  // next_fill_buffer = 0;
  // _fill_buffer();
  // next_fill_buffer = 1;
  // _fill_buffer();
  // next_fill_buffer = 0xFF;

  dma_doublebuffer_init(&dma_doublebuffer_oc);

  // for (int i = 0; i < BUFFER_SIZE; i++)
  // {
  //   printf("dma_buffer[%04u]: %u\r\n", i, dma_doublebuffer_oc.dma_buffer[i]);
  // }

  // for (uint32_t i = 0; i < dma_doublebuffer_oc.total_pulses; i++)
  // {
  //   uint32_t ccr = dma_doublebuffer_generate_t_ccr(&dma_doublebuffer_oc, i);
  //   // printf("pulse_index[%04lu]: %lu\r\n", i, ccr);
  // }

  // DMA1_Channel1->CCR &= ~DMA_CCR_HTIE; // 禁用半传输中断
  // DMA1_Channel1->CCR |= DMA_CCR_HTIE; // 重新启用半传输中断

  // htim3.hdma[TIM_DMA_ID_CC1]->Instance->CCR &= ~DMA_CCR_HTIE; // 禁用半传输中断
  // htim3.hdma[TIM_DMA_ID_CC1]->XferHalfCpltCallback = NULL;
  HAL_TIM_OC_Start_DMA(&htim3, TIM_CHANNEL_1, (uint32_t *)dma_doublebuffer_oc.dma_buffer, BUFFER_SIZE);
  //  __HAL_DMA_DISABLE_IT(htim3.hdma[TIM_DMA_ID_CC1], DMA_IT_HT);
  // htim3.hdma[TIM_DMA_ID_CC1]->Instance->CCR &= ~DMA_CCR_HTIE; // 禁用半传输中断
  // dma_doublebuffer_check_and_adjust(&dma_doublebuffer_oc);

  // HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);

  // HAL_TIM_Base_Start(&htim3);
  // HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // _fill_buffer_in_background();
    dma_doublebuffer_fill_in_background(&dma_doublebuffer_oc);
    // if (has_count_changed)
    // {
    //   has_count_changed = 0;
    //   // prinf_dma_info(&htim3, &dma_doublebuffer_oc);
    //   printf("finished_count:%lu,half_count:%lu\r\n", finished_count, half_count);
    // }
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
