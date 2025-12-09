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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

DMA_DoubleBuffer_t dma_doublebuffer;
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    static uint8_t has_print = 0;
    if (has_print == 0)
    {
      printf("HAL_TIM_PeriodElapsedCallback\r\n");
      has_print = 1;
    }
  }
}
void HAL_TIM_PeriodElapsedHalfCpltCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    // 查看 的实现，它的回调函数是 TIM_DMAPeriodElapsedHalfCplt 里面执行 HAL_TIM_PeriodElapsedHalfCpltCallback
    // 所以 HAL_TIM_PeriodElapsedHalfCpltCallback 才是半传输完成的回调函数
    static uint8_t has_half_print = 0;
    if (has_half_print == 0)
    {
      printf("HAL_TIM_PeriodElapsedHalfCpltCallback\r\n");
      has_half_print = 1;
    }
  }
}

void HAL_DMA_XferHalfCpltCallback(DMA_HandleTypeDef *hdma)
{
  if (hdma == &hdma_tim3_ch1_trig)
  {
    /* 在实际中断中不要做大量 printf（可能阻塞），这里仅用于调试。
       更稳妥的做法是设置一个 volatile flag 或 toggle GPIO。 */
    printf("HAL_DMA_XferHalfCpltCallback\r\n");
  }
}

void HAL_DMA_XferCpltCallback(DMA_HandleTypeDef *hdma)
{
  if (hdma == &hdma_tim3_ch1_trig)
  {

    printf("HAL_DMA_XferCpltCallback\r\n");
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

  /* 替换你现有的 DMA 启动相关片段（在 MX_*Init() 调用之后） */

  static uint16_t dma_buffer[] = {288, 4608, 2304, 1152, 576, 288, 144};
  uint16_t length = sizeof(dma_buffer) / sizeof(dma_buffer[0]);

  /* 1) 将 DMA 外设地址设为 TIM3->ARR（确保 PeriphInc = DISABLE 已在 tim.c 中设置） */
  hdma_tim3_ch1_trig.Instance->CPAR = (uint32_t)(&(TIM3->ARR));

  /* 2) 为了测试半传，使用 CIRCULAR 模式（如果 tim.c 把 Init.Mode 设置为 NORMAL，需要在这里重新 init） */
  hdma_tim3_ch1_trig.Init.Mode = DMA_CIRCULAR;
  if (HAL_DMA_Init(&hdma_tim3_ch1_trig) != HAL_OK)
  {
    Error_Handler();
  }

  /* 3) 确保链接到 UPDATE 槽（如果未链接） */
  __HAL_LINKDMA(&htim3, hdma[TIM_DMA_ID_UPDATE], hdma_tim3_ch1_trig);

  /* 4) 确保 NVIC 对应 IRQ 已启（MX_DMA_Init 通常已做，但重复一次无妨） */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

  /* 5) 启动 TIM（必须，使 Update/ TRGO 实际产生）——使用 PWM Start 更保险 */
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /* 6) 启动 Base+DMA 并检查返回值 */
  HAL_StatusTypeDef rc = HAL_TIM_Base_Start_DMA(&htim3, (uint32_t *)dma_buffer, length);
  printf("HAL_TIM_Base_Start_DMA rc=%d\r\n", (int)rc);

  /* 7) 读出关键寄存器用于现场诊断（在串口或调试器查看） */
  volatile uint32_t dbg_DMA_CCR = hdma_tim3_ch1_trig.Instance->CCR;
  volatile uint32_t dbg_DMA_CNDTR = hdma_tim3_ch1_trig.Instance->CNDTR;
  volatile uint32_t dbg_DMA_CPAR = hdma_tim3_ch1_trig.Instance->CPAR;
  printf("DMA CCR=0x%08lx CNDTR=%lu CPAR=0x%08lx\r\n",
         (unsigned long)dbg_DMA_CCR, (unsigned long)dbg_DMA_CNDTR, (unsigned long)dbg_DMA_CPAR);

  /* 诊断：启用 TIM Update 中断并打印关键寄存器 */
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE); /* 临时启用 TIM 更新中断用于验证 */

  /* 读 TIM3 状态寄存器等并打印 */
  printf("TIM3 CR1=0x%08lx CR2=0x%08lx DIER=0x%08lx SR=0x%08lx CNT=0x%08lx ARR=0x%08lx\n",
         (unsigned long)TIM3->CR1, (unsigned long)TIM3->CR2,
         (unsigned long)TIM3->DIER, (unsigned long)TIM3->SR,
         (unsigned long)TIM3->CNT, (unsigned long)TIM3->ARR);

  /* 打印 DMA1 ISR 寄存器（原始），并解析 channel6 的四个标志位 */
  uint32_t dma_isr = DMA1->ISR;
  printf("DMA1->ISR = 0x%08lx\n", (unsigned long)dma_isr);

  /* 每个通道占 4 位，从低到高：GIFx, TCIFx, HTIFx, TEIFx (或设备手册顺序)
     在 STM32F1 中，channel n 的位偏移 = 4*(n-1)。
     channel6 的偏移 = 20 */
  uint32_t ch6_shift = 4 * (6 - 1);
  uint32_t ch6_flags = (dma_isr >> ch6_shift) & 0xF;
  printf("DMA CH6 flags (bit3..0 = TE/HT/TC/GI) = 0x%1lx\n", (unsigned long)ch6_flags);

  /* 打印并解析 TIM3 DIER（查看 Update DMA request enable 是否置位） */
  uint32_t dier = TIM3->DIER;
  printf("TIM3 DIER = 0x%08lx\n", (unsigned long)dier);
  /* 如果设备手册一致，UDE（Update DMA request enable）通常是 DIER bit 8 (或者一个具体位) —— 直接打印供我判断 */
  printf("TIM3 DIER bits: UIE=%d, UDE=%d\n", (int)((dier >> 0) & 1), (int)((dier >> 8) & 1));

  /* 打印 DMA channel registers（CCR/CNDTR/CPAR）以复核 */
  printf("DMA CH6 CCR=0x%08lx CNDTR=%lu CPAR=0x%08lx CMAR=0x%08lx\n",
         (unsigned long)DMA1_Channel6->CCR,
         (unsigned long)DMA1_Channel6->CNDTR,
         (unsigned long)DMA1_Channel6->CPAR,
         (unsigned long)DMA1_Channel6->CMAR);

  /* 检查 NVIC 是否使能对应中断（简易检查） */
  uint32_t iser = NVIC->ISER[DMA1_Channel6_IRQn >> 5];
  uint32_t enabled = (iser >> (DMA1_Channel6_IRQn & 0x1F)) & 1;
  printf("NVIC ISER for DMA1_Channel6_IRQn = %lu (1=enabled)\n", (unsigned long)enabled);

  /* 检查 htim3.hdma 指针是否指向你正在使用的 hdma 句柄 */
  printf("htim3.hdma[UPDATE] = %p, hdma_tim3_ch1_trig = %p\n",
         (void *)htim3.hdma[TIM_DMA_ID_UPDATE], (void *)&hdma_tim3_ch1_trig);

  printf("DMA started\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    count = 0; // 清零计数器
    // printf()
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
