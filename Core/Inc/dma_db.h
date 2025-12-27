
#ifndef __DMA_DB_H
#define __DMA_DB_H

#include "stm32f1xx_hal.h"

// ========== 配置参数 ==========
#define MAX_DMA_BUFFER_SIZE 512            // 每个缓冲区256个脉冲
#define CLK_PSC 1.0f                       // 时钟预分频数
#define HAED_CLK_HZ 72000000.0f            // 硬件时钟频率
#define SYS_CLK_HZ (HAED_CLK_HZ / CLK_PSC) // 时钟频率
#define MIN_CCR_VALUE 10                   // 最小CCR值（对应最大频率）
#define MAX_CCR_VALUE 65536                // 最大CCR值（对应最小频率）
#define MAX_ARR_VALUE 65536                // 最大ARR值（对应最大频率）
#define MIN_ARR_VALUE 288                  // 最小ARR值（对应最小频率）

// ========== 数据结构 ==========

typedef enum DMA_DB_Mode_t
{
    PWM_ARR = 0,
    OC_CCR = 1,
} DMA_DB_Mode_t;

typedef struct
{
    DMA_DB_Mode_t mode; // 模式 0 = pwm+arr, 1 = oc+ccr

    TIM_HandleTypeDef *htim;
    uint32_t tim_channel; // 定时器通道

    // uint32_t max_rpm;     // 电机最高转速
    // float pulses_per_rev; // 脉冲数/圈

    // uint32_t total_pulses; // 总脉冲数
    // uint32_t accel_pulses; // 加速脉冲数
    // uint32_t decel_pulses; // 减速脉冲数

    // DMA缓冲区
    uint8_t hdma_id;
    uint16_t buffer_size;                     // DMA缓冲区大小
    uint16_t dma_buffer[MAX_DMA_BUFFER_SIZE]; // DMA缓冲区
    uint32_t total_data;                      // 总数据长度（单位：字节）
    uint32_t transfered_data;                 // 已传输数据长度（单位：字节）
    uint64_t g_last_accum;                    // 上一次累加值

    volatile uint8_t active_buffer;    // 当前活动缓冲区（0或1）
    volatile uint8_t next_fill_buffer; // 下一次要填充的缓冲区(0填充前半区，1填充后半区，255表示已填充)

    volatile uint32_t fill_buffer_in_background_count;
    void (*transfer_complete_callback)(void *context);
    int32_t (*on_fill_buffer)(void *context);
    void *callback_context; // 上下文参数（用户自定义数据）

} DMA_DB_t;

void dma_db_start(DMA_DB_t *dma_db);
// void _dma_db_fill_buffer(DMA_DB_t *dma_db);
// void _dma_db_switch_buffer(DMA_DB_t *dma_db);
void dma_db_fill_in_background(DMA_DB_t *dma_db);
// uint8_t dma_db_check_finished(DMA_DB_t *dma_db);
// void dma_db_transfer_start(DMA_DB_t *dma_db);
void dma_db_half_transfer_cb_handle(DMA_DB_t *dma_db);
void dma_db_transfer_complete_cb_handle(DMA_DB_t *dma_db);

// uint32_t dma_db_generate_t_arr(DMA_DB_t *dma_db, uint32_t pulse_index);
// uint32_t dma_db_generate_t_ccr(DMA_DB_t *dma_db, uint32_t pulse_index);
// uint32_t dma_db_generate_t_step_delay(DMA_DB_t *dma_db, uint32_t pulse_index);

#endif /* __DMA_DB_H */