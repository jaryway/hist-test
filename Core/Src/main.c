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
#include "debug.h"
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
// #include "motor.h"
#include "helper.h"
#include "bsp_modbus_master.h"
#include "motor_mb.h"

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

#define OC_IT      0
#define OC_DMA     1
#define PWM_IT     2
#define PWM_DMA    3
#define RUN_MODE   OC_DMA

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static uint32_t half_count       = 0;
static uint32_t finished_count   = 0;
static uint8_t has_count_changed = 0;
static uint32_t oc_it_count      = 0;
// static uint32_t last_time        = 0;
static int16_t rpm             = 0;
static int16_t load_rate       = 0;
static int32_t pos             = 0;
static uint8_t pos_reached     = 0;
static uint8_t homing_complete = 0;
// static uint8_t has_homed       = 0;

DMA_DB_t dma_db_oc;

// Motor_t motor = {
//     .run_state = MOTOR_STOP,
//     .pulses    = 0,
// };
MotorMB_t motor_mb = {
    .slave_addr = 0x01,
};

Profile_t motor42_profile = {
    .max_rpm          = 1000,     // 最高转速
    .steps_per_rev    = 200 * 16, // 16细分
    .reduction_ratio  = 1,        // 减速比
    .accel_time       = 0.5,      // 加速时间 ms
    .decel_time       = 0.3,      // 减速时间 ms
    .travel_distance  = 450 * 3,  // 导轨有效行程
    .distance_per_rev = 40,       // T2-20 齿,转一周周长:20*2=40mm
};

Profile_t servo_profile = {
    .max_rpm          = 3000,     // 最高转速
    .steps_per_rev    = 200 * 16, // 16细分
    .reduction_ratio  = 12,       // 减速比
    .accel_time       = 1,        // 加速时间 ms
    .decel_time       = 0.8,      // 减速时间 ms
    .travel_distance  = 1650,     // 导轨有效行程
    .distance_per_rev = 125,      // 同步轮 T5-25 转一周周长
};

uint32_t get_modbus_timeout(uint32_t baudrate, uint16_t quantity, uint8_t func_code)
{
    // 计算字符传输时间
    float char_time_ms = (10.0f * 1000.0f) / baudrate; // 10位/字符

    switch (func_code) {
        case 0x03: // 读保持寄存器
            // 请求: 8字节, 响应: 5+2*quantity 字节
            return (uint32_t)(8 + 5 + 2 * quantity) * char_time_ms + 300; // +300ms 处理时间

        case 0x06: // 写单个寄存器
            // 只需发送，响应类似
            return (uint32_t)(8 * char_time_ms) + 200; // +200ms 处理时间

        case 0x10: // 写多个寄存器
            // 发送: 9+2*quantity 字节
            return (uint32_t)((9 + 2 * quantity) * char_time_ms) + 200;

        default:
            return 500; // 默认值
    }
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

TCtrlParam_t motor_profile_2_t_ctrl_param(Profile_t pro)
{
    uint16_t travel_distance  = pro.travel_distance;                                // 导轨有效行程
    uint16_t distance_per_rev = pro.distance_per_rev;                               // 导轨同步轮转一周的长度
    uint8_t reduction_ratio   = pro.reduction_ratio <= 0 ? 1 : pro.reduction_ratio; // 减速比
    uint16_t max_rpm          = pro.max_rpm;                                        // 电机额定转速
    uint16_t steps_per_rev    = pro.steps_per_rev;                                  // 电机转一圈所需的脉冲数
    float accel_time          = pro.accel_time < 0 ? 0.0f : pro.accel_time;         // 加速时间
    float decel_time          = pro.decel_time < 0 ? 0.0f : pro.decel_time;         // 减速时间
    float steps_per_mm        = (reduction_ratio * steps_per_rev) / distance_per_rev;
    float rads_per_mm         = (float)reduction_ratio * 2.0f * PI / (float)distance_per_rev; // 每mm对应的角度 rad/mm

    float max_speed    = (float)max_rpm * 2.0f * PI / 60.0f; // 最大角速度 rad/s
    float accel        = max_speed / accel_time;             // 加速度 rad/s²  v = at
    float decel        = max_speed / decel_time;             // 减速度 rad/s²
    float total_pulses = steps_per_mm * travel_distance;     // 总步数
    float total_rads   = rads_per_mm * travel_distance;      // 总角度 rad
    float time_sec     = total_rads / max_speed;             // 运动总时间 s

    if (decel_time + accel_time > time_sec) {
        printf("Warning: Accel time + Decel time > Total time, adjusting...\r\n");
        float scale = time_sec / (accel_time + decel_time);
        accel_time *= scale;
        decel_time *= scale;
        accel = max_speed / accel_time;
        decel = max_speed / decel_time;
    }

    // --- 新增：预计时间（梯形/三角）---
    float theta_acc = (max_speed * max_speed) / (2.0f * accel);
    float theta_dec = (max_speed * max_speed) / (2.0f * decel);

    float t_total_sec = 0.0f;

    if ((theta_acc + theta_dec) <= total_rads) {
        // 梯形：能跑到 max_speed
        float t_acc       = max_speed / accel;
        float t_dec       = max_speed / decel;
        float theta_const = total_rads - theta_acc - theta_dec;
        float t_const     = theta_const / max_speed;
        t_total_sec       = t_acc + t_const + t_dec;
    } else {
        // 三角：跑不到 max_speed，计算 v_peak
        float inv    = (1.0f / accel) + (1.0f / decel);
        float v_peak = sqrtf((2.0f * total_rads) / inv);
        t_total_sec  = (v_peak / accel) + (v_peak / decel);
    }

    TCtrlParam_t t_ctrl_param;
    t_ctrl_param.accel       = (uint32_t)(accel * 10.0f);     // 加速度 rad/s² X10 后
    t_ctrl_param.decel       = (uint32_t)(decel * 10.0f);     // 加速度 rad/s² X10 后
    t_ctrl_param.pulses      = (int32_t)(total_pulses);       // 总步数
    t_ctrl_param.max_speed   = (uint32_t)(max_speed * 10.0f); // 速度 rad/s X10 后
    t_ctrl_param.est_time_ms = (uint32_t)(t_total_sec * 1000.0f + 0.5f);

    return t_ctrl_param;
}
void delay_ms_with_dma_service(uint32_t ms, DMA_DB_t *dma_db)
{
    uint32_t t0 = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - t0) < ms) {
        dma_db_fill_in_background(dma_db);

        // 可选：如果你希望更省电/更少空转，可以开 WFI
        // 但前提是 DMA 半传输/全传输中断、SysTick 等能唤醒
        __WFI();
    }
}

void print_motor_profile()
{
    uint16_t travel_distance  = 1650;                                                 // 导轨有效行程
    uint16_t distance_per_rev = 125;                                                  // 同步轮 T5-25 转一周周长
    uint8_t reduction_ratio   = 12;                                                   // 减速比
    uint16_t max_rpm          = 3000;                                                 // 电机额定转速 3000 RPM
    uint16_t steps_per_rev    = FSPR * MICRO_STEP;                                    // 电机转一圈所需的步数 3200步/圈
    float steps_per_sec       = max_rpm * steps_per_rev / 60.0f;                      // 最大速度 单位 steps/sec;  160000步/s
    float deg_per_sec         = max_rpm * 360.0f / 60.0f;                             // 最大角速度 deg/s
    float rad_per_sec         = max_rpm * 2.0f * PI / 60.0f;                          // 最大角速度 rad/s
    float steps_per_mm        = (reduction_ratio * steps_per_rev) / distance_per_rev; // 1mm 电机需要转的步数  307.2 步/mm
    float total_steps         = steps_per_mm * travel_distance;                       // 总步数
    float mm_per_step         = 1.0f / steps_per_mm;                                  // 每步对应的 mm 数 0.003255 mm/步
    float accel_distance      = 300.0f;                                               // 加速距离 单位 mm
    float accel_time          = 1.0f;                                                 // 加速时间 单位 s
    float accel_steps         = accel_distance * steps_per_mm;                        // 加速步数 92160

    float accel_steps_per_sec2 = 2.0f * accel_steps / (accel_time * accel_time);     // 加速度 单位 steps/sec^2 18432 steps/s²
    float accel_deg_per_sec2   = accel_steps_per_sec2 * (360.0f / steps_per_rev);    // 加速度 单位 deg/sec^2 2073.6 deg/s²
    float accel_rad_per_sec2   = accel_steps_per_sec2 * (2.0f * PI / steps_per_rev); // 加速度 单位 rad/sec^2 36.1 rad/s²

    float step_interval_sec = 1.0f / steps_per_sec;           // 步间隔时间（秒）
    float step_interval_us  = step_interval_sec * 1000000.0f; // 6.25 微秒

    // 打印计算结果
    printf("=== Motor Motion Parameters ===\r\n");
    printf("Guide rail travel: %d mm\r\n", travel_distance);                // 导轨有效行程
    printf("Synchronous wheel circumference: %d mm\r\n", distance_per_rev); // 同步轮周长
    printf("Reduction ratio: %d\r\n", reduction_ratio);                     // 减速比
    printf("Motor rated speed: %d RPM\r\n", max_rpm);                       // 电机额定转速
    printf("Steps per revolution: %d steps\r\n", steps_per_rev);            // 电机步数/圈
    printf("Max step speed: %.2f steps/sec\r\n", steps_per_sec);            // 最大步进速度
    printf("Steps per mm: %.2f steps/mm\r\n", steps_per_mm);                // 每mm对应的步数
    printf("mm per step: %.6f mm/step\r\n", mm_per_step);                   // 每步对应的mm数
    printf("Total steps: %.2f \r\n", total_steps);                          // 总步数
    printf("Acceleration distance: %.2f mm\r\n", accel_distance);           // 加速距离
    printf("Acceleration time: %.0f sec\r\n", accel_time);                  // 加速时间
    printf("Acceleration steps: %.2f steps\r\n", accel_steps);              // 加速步数
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
    if (hdma != NULL) {
        uint32_t par   = (uint32_t)hdma->Instance->CPAR;                                       /* DMA 外设地址 */
        uint32_t mar   = (uint32_t)hdma->Instance->CMAR;                                       /* DMA 内存地址 */
        uint32_t cndtr = (uint32_t)hdma->Instance->CNDTR;                                      /* 剩余要传的数据项数 */
        uint32_t ccr   = (uint32_t)__HAL_TIM_GET_COMPARE(htim, dma_doublebuffer->tim_channel); /* TIM 当前 CCR */

        uint32_t transferred = dma_doublebuffer->buffer_size - cndtr; /* 已经传送的数据量 */
        uint32_t last_idx    = (transferred == 0) ? ((uint32_t)dma_doublebuffer->buffer_size - 1) : (transferred - 1);
        uint32_t next_idx    = transferred % dma_doublebuffer->buffer_size;

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
    if (htim->Instance == TIM3) {
        // printf("HAL_TIM_PWM_PulseFinishedCallback\r\n");
#if RUN_MODE == OC_DMA
        dma_db_transfer_complete_it_cb_handle(&dma_db_oc);
#endif

        finished_count++;
        has_count_changed = 1;
    }
}

void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        // printf("HAL_TIM_PWM_PulseFinishedHalfCpltCallback\r\n");

#if RUN_MODE == OC_DMA
        dma_db_half_transfer_it_cb_handle(&dma_db_oc);
#endif

        half_count++;
        has_count_changed = 1;
    }
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        // printf("HAL_TIM_OC_DelayElapsedCallback-tim3\r\n");
        // last_time_oc += 1200;
        // __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, last_time_oc);

        // __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, cnt + 40);
#if RUN_MODE == OC_IT
        motor_oc_it_cb_handle(&motor);
#endif

        oc_it_count++;
        has_count_changed = 1;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        // printf("HAL_TIM_PeriodElapsedCallback-tim2\r\n");
        // dma_db_fill_in_background(&dma_db_oc);
    }
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

void monitor_task(void)
{
    MB_Status_t res;
    // uint16_t value;
    // uint16_t write_values[2];
    uint16_t read_values[12];

    // uint32_t micro_step = 10000;
    /* 08.P0B-00 监控转速单位rpm ------------------------------------*/
    res = modbus_read_holding_registers(1, REG_ADDR_RPM, 1, read_values, 100);
    if (res != MB_OK) {
        PRINT_DEBUG("read P0B-00 error,res:%d\n", res);
    } else {
        rpm = (int16_t)read_values[0];
    }

    /* 09.P0B-02 监控负载率百分比单位 --------------------------------*/
    res = modbus_read_holding_registers(1, REG_ADDR_LOAD_RATE, 1, read_values, 100);
    if (res != MB_OK) {
        PRINT_DEBUG("read P0B-02 error,res:%d\n", res);
    } else {
        load_rate = (int16_t)read_values[0];
    }

    /* 10.P0B-07 监控反馈位置 -----------------------------------------*/
    res = modbus_read_holding_registers(1, REG_ADDR_POS, 2, read_values, 100);
    if (res != MB_OK) {
        PRINT_DEBUG("read P0B-07 error,res:%d\n", res);
    } else {
        pos = regs_to_int32_le(read_values);
    }

    /* 11.P0B-04 监控状态字 -------------------------------------------*/
    // Bit0：位置到达
    // Bit1：速度到达
    // Bit2：转矩到达
    // Bit3：回零完成
    res = modbus_read_holding_registers(1, REG_ADDR_BUS_STATE, 2, read_values, 100);
    if (res != MB_OK) {
        PRINT_DEBUG("read P0B-04 error,res:%d\n", res);
        return;
    }

    uint8_t pos_sta  = (read_values[0] & (1 << 0)) != 0; // 位置到达
    uint8_t home_sta = (read_values[0] & (1 << 3)) != 0; // 回零完成
    homing_complete  = home_sta;
    pos_reached      = pos_sta;

    // PRINT_DEBUG("read P0B-04 success, value: %d (0x%04X)\r\n", read_values[0], read_values[0]);
    // PRINT_DEBUG("  Position reached: %s\r\n", (read_values[0] & 0x01) ? "YES" : "NO");
    // PRINT_DEBUG("  Speed reached: %s\r\n", (read_values[0] & 0x02) ? "YES" : "NO");
    // PRINT_DEBUG("  Torque reached: %s\r\n", (read_values[0] & 0x04) ? "YES" : "NO");
    // PRINT_DEBUG("  Homing complete: %s\r\n", (read_values[0] & 0x08) ? "YES" : "NO");
}

void homing(void)
{
    // 01.P02-00=9 控制模式为Modbus总线模式
    // 02.P10-03=6 运行模式为Home模式
    // 03.P10-35=19 回零模式为正向回原点
    // 04.P10-36=20000 回零速度为20000Pr/s,即120r/min
    // 05.P03-07=1 伺服上电使能
    // 06.P0D-08=128 Home模式启动
    // 07.P0D-08=256 停止
    // 08.P0B-00 监控转速单位rpm
    // 09.P0B-02 监控负载率百分比单位
    // 10.P0B-07 监控反馈位置
    // 11.P0B-04 监控状态字

    PRINT_DEBUG("homing\n");
    MB_Status_t res;
    uint16_t value;
    uint16_t write_values[2];
    // uint16_t read_values[12];
    uint32_t micro_step = 10000;

    /* 02.P10-03=6 运行模式为Home模式 --------------------------------*/
    value = 6;
    res   = modbus_write_single_register(1, REG_ADDR_RUN_MODE, value, 100);
    if (res != MB_OK) {
        PRINT_DEBUG("write P10-03=%d error,res:%d\n", value, res);
        return;
    }
    PRINT_DEBUG("write P10-03=%d success\n", value);

    /* 03.P10-35=19 回零模式为正向回原点 ------------------------------*/
    value = 6;
    res   = modbus_write_single_register(1, REG_ADDR_HOME_MODE, value, 100);
    if (res != MB_OK) {
        PRINT_DEBUG("write P10-35=%d error,res:%d\n", value, res);
        return;
    }
    PRINT_DEBUG("write P10-35=%d success\n", value);

    /* 04.P10-36=20000 回零速度为20000Pr/s,即120r/min ---------------*/
    uint32_t speed = 5 * micro_step;
    uint32_to_regs_le(speed, write_values);
    res = modbus_write_multiple_registers(1, REG_ADDR_HOME_SPEED, 2, write_values, 200);
    if (res != MB_OK) {
        PRINT_DEBUG("write P10-36=%lu error,res:%d\n", speed, res);
        return;
    }
    PRINT_DEBUG("write P10-36=%lu success\n", speed);

    /* 05.P03-07=1 伺服上电使能 --------------------------------------*/
    value = 1;
    res   = modbus_write_single_register(1, REG_ADDR_EN, value, 100);
    if (res != MB_OK) {
        PRINT_DEBUG("write P03-07=%d error,res:%d\n", value, res);
        return;
    }
    PRINT_DEBUG("write P03-07=%d success\n", value);

    /* 06.P0D-08=128 Home模式启动 -----------------------------------*/
    value = 128;
    res   = modbus_write_single_register(1, REG_ADDR_BUS_CTRL_MODE, value, 100);
    if (res != MB_OK) {
        PRINT_DEBUG("write P0D-08=%d error,res:%d\n", value, res);
        return;
    }
    PRINT_DEBUG("write P0D-08=%d success\n", value);

    // /* 07.P0D-08=256 停止 -------------------------------------------*/
    // HAL_Delay(5000);
    // value = 256;
    // res   = modbus_write_single_register(1, REG_ADDR_BUS_CTRL_MODE, value, 100);
    // if (res != MB_OK) {
    //     PRINT_DEBUG("write P0D-08=%d error,res:%d\n", value, res);
    //     return;
    // }
    // PRINT_DEBUG("write P0D-08=%d success\n", value);

    // /* 08.P0B-00 监控转速单位rpm ------------------------------------*/
    // res = modbus_read_holding_registers(1, REG_ADDR_RPM, 1, read_values, 100);
    // if (res != MB_OK) {
    //     PRINT_DEBUG("read P0B-00 error,res:%d\n", res);
    //     return;
    // }
    // PRINT_DEBUG("read P0B-00 success,value=%d\n", (int16_t)read_values[0]);

    // /* 09.P0B-02 监控负载率百分比单位 --------------------------------*/
    // res = modbus_read_holding_registers(1, REG_ADDR_LOAD_RATE, 1, read_values, 100);
    // if (res != MB_OK) {
    //     PRINT_DEBUG("read P0B-02 error,res:%d\n", res);
    //     return;
    // }
    // PRINT_DEBUG("read POB-02 success,value=%d\n", (int16_t)read_values[0]);

    // /* 10.P0B-07 监控反馈位置 -----------------------------------------*/
    // res = modbus_read_holding_registers(1, REG_ADDR_POS, 2, read_values, 100);
    // if (res != MB_OK) {
    //     PRINT_DEBUG("read P0B-07 error,res:%d\n", res);
    //     return;
    // }
    // PRINT_DEBUG("read P0B-07 success,value=%lu\n", regs_to_uint32_le(read_values));

    // /* 11.P0B-04 监控状态字 -------------------------------------------*/
    // // Bit0：位置到达
    // // Bit1：速度到达
    // // Bit2：转矩到达
    // // Bit3：回零完成
    // res = modbus_read_holding_registers(1, REG_ADDR_BUS_STATE, 2, read_values, 100);
    // if (res != MB_OK) {
    //     PRINT_DEBUG("read P0B-04 error,res:%d\n", res);
    //     return;
    // }

    // PRINT_DEBUG("read P0B-04 success, value: %d (0x%04X)\r\n", read_values[0], read_values[0]);
    // PRINT_DEBUG("  Position reached: %s\r\n", (read_values[0] & 0x01) ? "YES" : "NO");
    // PRINT_DEBUG("  Speed reached: %s\r\n", (read_values[0] & 0x02) ? "YES" : "NO");
    // PRINT_DEBUG("  Torque reached: %s\r\n", (read_values[0] & 0x04) ? "YES" : "NO");
    // PRINT_DEBUG("  Homing complete: %s\r\n", (read_values[0] & 0x08) ? "YES" : "NO");
}

// 正转5圈
void run_5_circle(void)
{
    // 01.P02-00=9  控制模式为Modbus总线模式 x
    // 02.P0D-17=1 强制DI使能  x
    // 03.P10-03=1 运行模式为PP模式
    // 04.P10-27=50000 设置加速度
    // 05.P10-27=50000 设置减速度
    // 06.P10-14=50000 设置目标位置
    // 07.P10-25=10000 设置速度
    // 08.P0D-18=507 使能   x
    // 09.P0D-08=1 相对定位启动
    // 10.P0D-08=0 复位
    // 11.POB-O0 监控转速单位rpm
    // 12.POB-02 监控负载率百分比单位
    // 13.POB-07 监控反馈位置
    // 14.P0B-04 监控状态字

    PRINT_DEBUG("run_5_circle\n");
    pos_reached = 0;

    MB_Status_t res;
    uint16_t value;
    uint32_t val32;
    uint16_t write_values[2];
    // uint16_t read_values[12];
    uint32_t micro_step = 800;

    /* 01.P02-00=9  控制模式为Modbus总线模式 x */
    // res = modbus_write_single_register(1, REG_ADDR_CTRL_MODE, 9, 100);
    // if (res != MB_OK) {
    //     PRINT_DEBUG("REG_ADDR_CTRL_MODE error,res:%d\n", res);
    //     return;
    // }

    /* 02.P0D-17=1 强制DI使能 x */
    // res = modbus_write_single_register(1, REG_ADDR_FORCE_IO, 1, 100);
    // if (res != MB_OK) {
    //     PRINT_DEBUG("write P0D-17=1 error,res:%d\n", res);
    //     return;
    // }
    // PRINT_DEBUG("write P0D-17=1 success\n");

    /* 03.P10-03=1 运行模式为PP模式 --------------------------------*/
    value = 1;
    res   = modbus_write_single_register(1, REG_ADDR_RUN_MODE, value, 100);
    if (res != MB_OK) {
        PRINT_DEBUG("write P10-03=%d error,res:%d\n", value, res);
        return;
    }
    PRINT_DEBUG("write P10-03=%d success\n", value);

    /* 04.P10-27=50000 设置加速度 -----------------------------*/
    val32 = 5 * micro_step;
    uint32_to_regs_le(val32, write_values);
    res = modbus_write_multiple_registers(1, REG_ADDR_ACCEL, 2, write_values, 200);
    if (res != MB_OK) {
        PRINT_DEBUG("write P10-27=%lu error,res:%d\n", val32, res);
        return;
    }
    PRINT_DEBUG("write P10-27=%lu success\n", val32);

    /* 05.P10-29=50000 设置减速度 -----------------------------*/
    val32 = 5 * micro_step;
    uint32_to_regs_le(val32, write_values);
    res = modbus_write_multiple_registers(1, REG_ADDR_DECEL, 2, write_values, 200);
    if (res != MB_OK) {
        PRINT_DEBUG("write P10-29=%lu error,res:%d\n", val32, res);
        return;
    }
    PRINT_DEBUG("write P10-29=%lu success\n", val32);

    /* 06.P10-14=x 设置目标位置 ------------------------------------- */
    val32 = 3000 * micro_step;
    uint32_to_regs_le(val32, write_values);
    res = modbus_write_multiple_registers(1, REG_ADDR_TARGET_POS, 2, write_values, 200);
    if (res != MB_OK) {
        PRINT_DEBUG("write P10-14=%lu error,res:%d\n", val32, res);
        return;
    }
    PRINT_DEBUG("write P10-14=%lu success\n", val32);

    /* 07.P10-25=1 设置速度 --------------------------------------- */
    uint32_t rps = 3000 / 60.0f * micro_step;
    uint32_to_regs_le(rps, write_values); // 3000RPM
    res = modbus_write_multiple_registers(1, REG_ADDR_SPEED, 2, write_values, 200);
    if (res != MB_OK) {
        PRINT_DEBUG("write P10-25=%lu error,res:%d\n", rps, res);
        return;
    }
    PRINT_DEBUG("write P10-25=%lu success\n", rps);

    // /* 08.P0D-18=507 使能 */
    // value = 507;
    // res   = modbus_write_single_register(1, REG_ADDR_FORCE_INPUT, value, 100);
    // if (res != MB_OK) {
    //     PRINT_DEBUG("write P0D-18=%d error,res:%d\n", value, res);
    //     return;
    // }
    // PRINT_DEBUG("write P0D-18=%d success\n", value);

    /* 09.P0D-08=1 相对定位启动 --------------------------------------------- */
    value = 1;
    res   = modbus_write_single_register(1, REG_ADDR_BUS_CTRL_MODE, value, 100);
    if (res != MB_OK) {
        PRINT_DEBUG("write P0D-08=%d error,res:%d\n", value, res);
        return;
    }
    PRINT_DEBUG("write P0D-08=%d success\n", value);
    /* 10.P0D-08=0 复位 --------------------------------------------- */
    value = 0;
    res   = modbus_write_single_register(1, REG_ADDR_BUS_CTRL_MODE, value, 100);
    if (res != MB_OK) {
        PRINT_DEBUG("write P0D-08=%d error,res:%d\n", value, res);
        return;
    }
    PRINT_DEBUG("write P0D-08=%d success\n", value);

    // /* 11.P0B-00 监控转速单位rpm ------------------------------------*/
    // res = modbus_read_holding_registers(1, REG_ADDR_RPM, 1, read_values, 100);
    // if (res != MB_OK) {
    //     PRINT_DEBUG("read P0B-00 error,res:%d\n", res);
    //     return;
    // }
    // PRINT_DEBUG("read POB-02 success, value=%d\n", (int16_t)read_values[0]);

    // /* 12.P0B-02 监控负载率百分比单位 --------------------------------*/
    // res = modbus_read_holding_registers(1, REG_ADDR_LOAD_RATE, 1, read_values, 100);
    // if (res != MB_OK) {
    //     PRINT_DEBUG("read P0B-02 error,res:%d\n", res);
    //     return;
    // }
    // PRINT_DEBUG("read POB-02 success,value=%d\n", (int16_t)read_values[0]);

    // /* 13.P0B-07 监控反馈位置 -----------------------------------------*/
    // res = modbus_read_holding_registers(1, REG_ADDR_POS, 2, read_values, 100);
    // if (res != MB_OK) {
    //     PRINT_DEBUG("read P0B-07 error,res:%d\n", res);
    //     return;
    // }
    // PRINT_DEBUG("read POB-07 success,value=%lu\n", regs_to_uint32_le(read_values));

    // /* 14.P0B-04 监控状态字 -------------------------------------------*/
    // // Bit0：位置到达
    // // Bit1：速度到达
    // // Bit2：转矩到达
    // // Bit3：回零完成
    // res = modbus_read_holding_registers(1, REG_ADDR_BUS_STATE, 2, read_values, 100);
    // if (res != MB_OK) {
    //     PRINT_DEBUG("read P0B-04 error,res:%d\n", res);
    //     return;
    // }
    // PRINT_DEBUG("read P0B-04 success\n");

    // printf("read P0B-04 success,value: %d (0x%04X)\r\n", read_values[0], read_values[0]);
    // printf("  Position reached: %s\r\n", (read_values[0] & 0x01) ? "YES" : "NO");
    // printf("  Speed reached: %s\r\n", (read_values[0] & 0x02) ? "YES" : "NO");
    // printf("  Torque reached: %s\r\n", (read_values[0] & 0x04) ? "YES" : "NO");
    // printf("  Homing complete: %s\r\n", (read_values[0] & 0x08) ? "YES" : "NO");
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
    HAL_Delay(1000);

    //     Profile_t profiles[] = {motor42_profile, servo_profile};

    //     for (int i = 0; i < 2; i++) {
    //         Profile_t pro             = profiles[i];
    //         TCtrlParam_t t_ctrl_param = motor_profile_2_t_ctrl_param(pro);
    //         printf("Profile %d:\r\n", i + 1);
    //         printf("  pulses: %ld\r\n", t_ctrl_param.pulses);
    //         printf("  accel:  %lu (rad/sec² * 10)\r\n", t_ctrl_param.accel);
    //         printf("  decel:  %lu (rad/sec² * 10)\r\n", t_ctrl_param.decel);
    //         printf("  max_speed:  %lu (rad/sec * 10)\r\n", t_ctrl_param.max_speed);
    //         printf("  est_time: %lu ms\r\n", t_ctrl_param.est_time_ms);
    //     }

    //     // 1、初始化电机
    //     TCtrlParam_t t_ctrl_param = motor_profile_2_t_ctrl_param(motor42_profile);
    //     motor_init(&motor);
    //     motor_attach(&motor, GPIOB, GPIO_PIN_4, DIR_GPIO_Port, DIR_Pin, ENA_GPIO_Port, ENA_Pin);
    //     motor_attach_timer(&motor, &htim3, TIM_CHANNEL_1);
    //     // motor_create_t_ctrl_param(&motor, t_ctrl_param.pulses, t_ctrl_param.accel, t_ctrl_param.decel, t_ctrl_param.speed);
    //     motor_set_pulses(&motor, t_ctrl_param.pulses);
    //     motor_set_accel(&motor, t_ctrl_param.accel);
    //     motor_set_decel(&motor, t_ctrl_param.decel);
    //     motor_set_max_speed(&motor, t_ctrl_param.max_speed);

    // #if RUN_MODE == OC_DMA
    //     motor_oc_start_dma(&motor, &dma_db_oc);
    // #endif

    // #if RUN_MODE == OC_IT
    //     motor_oc_start_it(&motor);
    // #endif

    modbus_init(&MODBUS_HUART, RS485_RE_GPIO_Port, RS485_RE_Pin);

    // homing();
    // HAL_Delay(1000 * 10);
    // run_5_circle();

    motor_mb_init(&motor_mb, 0x01);
    motor_mb_set_speed(&motor_mb, SPEED);
    motor_mb_set_accel(&motor_mb, ACCEL);
    motor_mb_set_decel(&motor_mb, DECEL);
    // motor_mb_homing(&motor_mb);
    motor_mb_move(&motor_mb, 800000);
    // HAL_Delay(1000);
    // motor_mb_stop(&motor_mb);
    // motor_mb_move_to(&motor_mb, 0);
    // HAL_Delay(1000);
    // motor_mb_stop(&motor_mb);

    // uint16_t quantity = 2;
    // uint16_t read_buffer[2];
    // uint16_t write_values[2];
    // MB_Status_t status;
    // status = modbus_read_holding_registers(0x01, REG_ADDR_POS, quantity, read_buffer, 1000);

    // status = modbus_write_single_register(0x01, 0x1, 666, 1000);
    // printf("status=%d\r\n", status);
    // 对于32位的数值，是采用小端字节序存储的，即低位在前，高位在后。
    // uint32_to_regs_le(speed_value, write_values);

    // printf("write multiple registers\r\n");
    // status = modbus_write_multiple_registers(0x01, REG_ADDR_SPEED, quantity, write_values, 1000);
    // printf("write multiple registers status=%d\r\n", status);

    // if (status == MB_OK) {
    //     printf("Modbus read successful!\r\n");
    //     printf("Read %d registers from slave 0x%02X:\r\n", quantity, 0x01);

    //     // 打印读取到的数据
    //     for (int i = 0; i < quantity; i++) {
    //         printf("Register[0x%04X] = 0x%04X (%d)\r\n",
    //                0x0000 + i, read_buffer[i], read_buffer[i]);
    //     }
    //     // int32_t read_value = regs_to_int32_le(read_buffer);
    //     // printf("Read value: %ld\r\n", read_value);
    // } else {
    //     printf("Modbus read failed with status: %d\r\n", status);
    // }

    // modbus_write_06(0x01, 0x0000, 0x0001);

    // DMA1_Channel1->CCR &= ~DMA_CCR_HTIE; // 禁用半传输中断
    // DMA1_Channel1->CCR |= DMA_CCR_HTIE; // 重新启用半传输中断

    // HAL_TIM_OC_Start_DMA(&htim3, TIM_CHANNEL_1, (uint32_t *)dma_db_oc.dma_buffer, dma_db_oc.buffer_size);

    // HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);

    // HAL_TIM_Base_Start(&htim3);
    // HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    uint8_t d = 0;
    while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        // monitor_task();
        motor_mb_process(&motor_mb);
        // if (motor_mb_is_stopped(&motor_mb)) {
        //     motor_mb_move(&motor_mb, d == 0 ? -TOTAL_STEPS : TOTAL_STEPS);
        //     d = !d;
        // }
        // if (homing_complete && !has_homed) {
        //     homing_complete = 0;
        //     has_homed       = 1;
        //     run_5_circle();
        // }
        // if (pos_reached && has_homed) {
        //     PRINT_DEBUG("Position reached\r\n");
        // }
        // motor_mb_process(&motor_mb);
        // if (motor_mb_is_stopped(&motor_mb)) {
        //     // printf("Motor stopped\r\n");
        //     // HAL_Delay(1000);
        //     // motor_mb_move(&motor_mb, d == 0 ? -TOTAL_STEPS : TOTAL_STEPS);
        //     // HAL_Delay(1000);
        //     d = !d;
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
