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
#include "dma_db.h"
#include "motor.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PI 3.1415926  /* 圆周率*/
#define FSPR 200      /* 步进电机单圈步数 */
#define MICRO_STEP 16 /* 步进电机驱动器细分数 */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static uint32_t half_count = 0;
static uint32_t finished_count = 0;
static uint8_t has_count_changed = 0;
static uint32_t oc_it_count = 0;

DMA_DB_t dma_db_oc;

Motor_t motor = {STOP, CW, 0, 0, 0, 0, 0, 0, 0, 0};



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void print_motor_profile()
{
  uint16_t L = 1650;                                          // 导轨有效行程
  uint16_t C = 125;                                           // 同步轮 T5-25 转一周周长
  uint8_t reduction_ratio = 12;                               // 减速比
  uint16_t max_rpm = 3000;                                    // 电机额定转速 3000 RPM
  uint16_t steps_per_rev = FSPR * MICRO_STEP;                 // 电机转一圈所需的步数 3200步/圈
  float steps_per_sec = max_rpm * steps_per_rev / 60.0f;      // 最大速度 单位 steps/sec;  160000步/s
  float deg_per_sec = max_rpm * 360.0f / 60.0f;               // 最大角速度 deg/s
  float rad_per_sec = max_rpm * 2.0f * PI / 60.0f;            // 最大角速度 rad/s
  float steps_per_mm = (reduction_ratio * steps_per_rev) / C; // 1mm 电机需要转的步数  307.2 步/mm
  float total_steps = steps_per_mm * L;                       // 总步数
  float mm_per_step = 1.0f / steps_per_mm;                    // 每步对应的 mm 数 0.003255 mm/步
  float accel_distance = 300.0f;                              // 加速距离 单位 mm
  float accel_time = 1.0f;                                    // 加速时间 单位 s
  float accel_steps = accel_distance * steps_per_mm;          // 加速步数 92160

  float accel_steps_per_sec2 = 2.0f * accel_steps / (accel_time * accel_time);   // 加速度 单位 steps/sec^2 18432 steps/s²
  float accel_deg_per_sec2 = accel_steps_per_sec2 * (360.0f / steps_per_rev);    // 加速度 单位 deg/sec^2 2073.6 deg/s²
  float accel_rad_per_sec2 = accel_steps_per_sec2 * (2.0f * PI / steps_per_rev); // 加速度 单位 rad/sec^2 36.1 rad/s²

  float step_interval_sec = 1.0f / steps_per_sec;          // 步间隔时间（秒）
  float step_interval_us = step_interval_sec * 1000000.0f; // 6.25 微秒

  // 打印计算结果
  printf("=== Motor Motion Parameters ===\r\n");
  printf("Guide rail travel: %d mm\r\n", L);                    // 导轨有效行程
  printf("Synchronous wheel circumference: %d mm\r\n", C);      // 同步轮周长
  printf("Reduction ratio: %d\r\n", reduction_ratio);           // 减速比
  printf("Motor rated speed: %d RPM\r\n", max_rpm);             // 电机额定转速
  printf("Steps per revolution: %d steps\r\n", steps_per_rev);  // 电机步数/圈
  printf("Max step speed: %.2f steps/sec\r\n", steps_per_sec);  // 最大步进速度
  printf("Steps per mm: %.2f steps/mm\r\n", steps_per_mm);      // 每mm对应的步数
  printf("mm per step: %.6f mm/step\r\n", mm_per_step);         // 每步对应的mm数
  printf("Total steps: %.2f \r\n", total_steps);                // 总步数
  printf("Acceleration distance: %.2f mm\r\n", accel_distance); // 加速距离
  printf("Acceleration time: %.0f sec\r\n", accel_time);        // 加速时间
  printf("Acceleration steps: %.2f steps\r\n", accel_steps);    // 加速步数
  printf("=== Acceleration Parameters ===\r\n");
  printf("Step acceleration: %.2f steps/s²\r\n", accel_steps_per_sec2); // 加速度 单位 steps/s²
  printf("Angular acceleration: %.2f deg/s²\r\n", accel_deg_per_sec2);  // 加速度 单位 deg/s²
  printf("Radial acceleration: %.2f rad/s²\r\n", accel_rad_per_sec2);   // 加速度 单位 rad/s²
  printf("=== Max Speed Conversion ===\r\n");
  printf("Max degree speed: %.2f deg/s\r\n", deg_per_sec);        // 最大角速度 deg/s
  printf("Max radian speed: %.2f rad/s\r\n", rad_per_sec);        // 最大角速度 rad/s
  printf("Max step speed: %.2f steps/s\r\n", steps_per_sec);      // 最大步进速度 steps/s
  printf("Step-to-step interval: %.2f μs\r\n", step_interval_us); // 最大速度时的步间隔时间
  printf("========================\r\n");
}

void prinf_dma_info(TIM_HandleTypeDef *htim, DMA_DB_t *dma_doublebuffer)
{
  DMA_HandleTypeDef *hdma = htim->hdma[TIM_DMA_ID_CC1];
  if (hdma != NULL)
  {
    uint32_t par = (uint32_t)hdma->Instance->CPAR;                                       /* DMA 外设地址 */
    uint32_t mar = (uint32_t)hdma->Instance->CMAR;                                       /* DMA 内存地址 */
    uint32_t cndtr = (uint32_t)hdma->Instance->CNDTR;                                    /* 剩余要传的数据项数 */
    uint32_t ccr = (uint32_t)__HAL_TIM_GET_COMPARE(htim, dma_doublebuffer->tim_channel); /* TIM 当前 CCR */

    uint32_t transferred = dma_doublebuffer->buffer_size - cndtr; /* 已经传送的数据量 */
    uint32_t last_idx = (transferred == 0) ? ((uint32_t)dma_doublebuffer->buffer_size - 1) : (transferred - 1);
    uint32_t next_idx = transferred % dma_doublebuffer->buffer_size;

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
    finished_count++;
    has_count_changed = 1;

    // // prinf_dma_info(htim, &dma_db_oc);
    // if (dma_db_check_finished(&dma_db_oc))
    //   return;

    // dma_db_switch_buffer(&dma_db_oc);
    dma_db_transfer_complete_cb_handle(&dma_db_oc);
  }
}

void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    // printf("HAL_TIM_PWM_PulseFinishedHalfCpltCallback\r\n");
    half_count++;
    has_count_changed = 1;

    // prinf_dma_info(htim, &dma_db_oc);

    // if (dma_db_check_finished(&dma_db_oc))
    //   return;

    // dma_db_switch_buffer(&dma_db_oc);
    dma_db_half_transfer_cb_handle(&dma_db_oc);
  }
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    // printf("HAL_TIM_OC_DelayElapsedCallback-tim3\r\n");
    // last_time_oc += 1200;
    // __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, last_time_oc);
    // uint32_t cnt = __HAL_TIM_GET_COUNTER(htim);
    // __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, cnt + 40);
    oc_it_count++;
    has_count_changed = 1;
  }
}

int _write(int file, char *ptr, int len)
{
  (void)file;
  // 在中断上下文中使用非阻塞方式
  if (__get_IPSR() != 0)
  {
    // 在中断中，使用较短的超时时间
    HAL_UART_Transmit(&huart3, (uint8_t *)ptr, len, 2000);
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
  // HAL_Delay(1000);
  print_motor_profile();
  int32_t pulses = 506550;    // 总步数
  uint32_t accel = 3616.8;    // 加速度 rad/s² X10 后
  uint32_t decel = 3616.8;    // 加速度 rad/s² X10 后
  // uint32_t speed = 3141.5926; // 速度 rad/s X10 后
  uint32_t speed = 1200; // 速度 rad/s X10 后
  // 输出的   max_s_lim = 69478
  // 预期的 accel_steps = 92160

  // dma_db_init(&dma_db_oc);

  // 1、初始化电机
  motor_init(&motor);
  motor_attach(&motor, GPIOB, GPIO_PIN_4, DIR_GPIO_Port, DIR_Pin, ENA_GPIO_Port, ENA_Pin);
  motor_attach_timer(&motor, &htim3, TIM_CHANNEL_1);
  motor_create_t_ctrl_param(&motor, pulses, accel, decel, speed);
  motor_oc_start_dma(&motor, &dma_db_oc);
  // while (motor.run_state != STOP)
  // {
  //   HAL_Delay(50);
  //   calc_step_delay(&motor);
  //   printf("motor run_state:%u\r\n", motor.run_state);
  //   printf("add_pulse_count:%lu\r\n", motor.add_pulse_count);
  //   printf("step_delay:%lu\r\n", motor.step_delay);
  // }

  // uint32_t cnt = __HAL_TIM_GET_COUNTER(&htim3);
  // g_ccr32 = __HAL_TIM_GET_COUNTER(&htim3);

  // next_fill_buffer = 0;
  // _fill_buffer();
  // next_fill_buffer = 1;
  // _fill_buffer();
  // next_fill_buffer = 0xFF;

  // dma_db_init(&dma_db_oc);
  // dma_db_init(&dma_db_oc);
  // uint16_t n = 0;
  // for (uint32_t i = 0; i < dma_db_oc.total_pulses; i++)
  // {

  //   uint16_t *dma_buffer = dma_db_oc.active_buffer == 0 ? dma_db_oc.dma_buf0 : dma_db_oc.dma_buf1;

  //   printf("pulse_index[%04lu]: %u,active_buffer:%u\r\n", i, dma_buffer[n], //
  //          dma_db_oc.active_buffer);

  //   n++;

  //   if ((i + 1) % (DMA_DB_BUF_SIZE) == 0)
  //   {
  //     printf("Switch buffer, active_buffer:%u, next_fill_buffer:%u \r\n", //
  //            dma_db_oc.active_buffer, dma_db_oc.next_fill_buffer);
  //     dma_db_switch_buffer(&dma_db_oc);
  //     printf("After switch buffer, active_buffer:%u, next_fill_buffer:%u \r\n", //
  //            dma_db_oc.active_buffer, dma_db_oc.next_fill_buffer);
  //     // uint8_t next_fill_buffer = dma_db_oc.next_fill_buffer;
  //     // printf("dma_db_switch_buffer:%u\r\n", next_fill_buffer);
  //     dma_db_fill_in_background(&dma_db_oc);
  //     n = 0;
  //   }
  // }

  // for (int i = 0; i < BUFFER_SIZE; i++)
  // {
  //   printf("dma_buffer[%04u]: %u\r\n", i, dma_db_oc.dma_buffer[i]);
  // }

  // for (uint32_t i = 0; i < dma_db_oc.total_pulses; i++)
  // {
  //   uint32_t ccr = dma_db_generate_t_ccr(&dma_db_oc, i);
  //   // printf("pulse_index[%04lu]: %lu\r\n", i, ccr);
  // }

  // DMA1_Channel1->CCR &= ~DMA_CCR_HTIE; // 禁用半传输中断
  // DMA1_Channel1->CCR |= DMA_CCR_HTIE; // 重新启用半传输中断

  // HAL_TIM_OC_Start_DMA(&htim3, TIM_CHANNEL_1, (uint32_t *)dma_db_oc.dma_buffer, dma_db_oc.buffer_size);

  //  __HAL_DMA_DISABLE_IT(htim3.hdma[TIM_DMA_ID_CC1], DMA_IT_HT);
  // htim3.hdma[TIM_DMA_ID_CC1]->Instance->CCR &= ~DMA_CCR_HTIE; // 禁用半传输中断
  // dma_db_check_and_adjust(&dma_db_oc);

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
    dma_db_fill_in_background(&dma_db_oc);
    static uint32_t last_time = 0;
    if (HAL_GetTick() - last_time > 1000 && has_count_changed)
    {
      has_count_changed = 0;
      last_time = HAL_GetTick();
      // prinf_dma_info(&htim3, &dma_db_oc);
      // printf("finished_count:%lu,half_count:%lu,oc_it_count:%lu\r\n", finished_count, half_count, oc_it_count);
    }
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
