// motor_driver.h
#ifndef __DMA_DOUBLEBUFFER_H
#define __DMA_DOUBLEBUFFER_H

#include "stm32f1xx_hal.h"

// ========== 配置参数 ==========
#define BUFFER_SIZE 96         // 每个缓冲区256个脉冲
#define MIN_CCR_VALUE 10       // 最小CCR值（对应最大频率）
#define MAX_CCR_VALUE 65536    // 最大CCR值（对应最小频率）
#define MAX_ARR_VALUE 65536    // 最大ARR值（对应最大频率）
#define MIN_ARR_VALUE 288      // 最小ARR值（对应最小频率）
#define SYS_CLK_HZ 72000000.0f // 系统时钟72MHz PSC=0 的情况下
#define MIN_START_FREQ 100     // 最小启动频率100Hz

// ========== 数据结构 ==========

typedef enum DMA_DoubleBuffer_Mode_t
{
    PWM_ARR = 0,
    OC_CCR = 1,
} DMA_DoubleBuffer_Mode_t;

typedef struct
{
    DMA_DoubleBuffer_Mode_t mode; // 模式 0 = pwm+arr, 1 = oc+ccr
    TIM_HandleTypeDef *htim;
    uint32_t tim_channel;  // 定时器通道
    uint32_t total_pulses; // 总脉冲数
    uint32_t accel_pulses; // 加速脉冲数
    uint32_t decel_pulses; // 减速脉冲数
    uint32_t rpm;          // 电机转速
    float pulses_per_rev;  // 脉冲数/圈

    // DMA缓冲区
    // uint32_t *dma_buf0;       // DMA缓冲区0
    // uint32_t *dma_buf1;       // DMA缓冲区1
    // uint16_t dma_buf0[BUFFER_SIZE];       // DMA缓冲区0
    // uint16_t dma_buf1[BUFFER_SIZE];       // DMA缓冲区1
    uint16_t dma_buffer[BUFFER_SIZE];  // DMA缓冲区
    volatile uint8_t active_buffer;    // 当前活动缓冲区（0或1）
    volatile uint8_t next_fill_buffer; // 下一次要填充的缓冲区(0填充前半区，1填充后半区，255表示已填充)
    volatile uint32_t pulses_sent;     // 已发送的脉冲数
    volatile uint32_t pulses_filled;   // 已填充的脉冲数
    volatile uint64_t g_last_accum;    // 上一个绝对CCR时间点

} DMA_DoubleBuffer_t;

void init_double_buffer(DMA_DoubleBuffer_t *dma_doublebuffer);
void fill_single_buffer(DMA_DoubleBuffer_t *dma_doublebuffer, uint32_t start_idx);
void fill_buffer(DMA_DoubleBuffer_t *dma_doublebuffer);

uint32_t generate_trapezoid_arr(DMA_DoubleBuffer_t *dma_doublebuffer, uint32_t pulse_index);
uint32_t generate_trapezoid_ccr(DMA_DoubleBuffer_t *dma_doublebuffer, uint32_t pulse_index);
uint32_t generate_trapezoid_period_ticks(DMA_DoubleBuffer_t *dma_doublebuffer, uint32_t pulse_index);
void fill_buffer_in_background(DMA_DoubleBuffer_t *dma_doublebuffer);

#endif /* __DMA_DOUBLEBUFFER_H */