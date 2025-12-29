
#ifndef __DMA_DB_H
#define __DMA_DB_H

#include "stm32f1xx_hal.h"

// ========== 配置参数 ==========
#define MAX_DMA_BUFFER_SIZE 256 // 每个缓冲区256个脉冲

// ========== 数据结构 ==========

typedef enum DMA_DB_Mode_t {
    PWM_ARR = 0,
    OC_CCR  = 1,
} DMA_DB_Mode_t;

typedef struct
{
    DMA_DB_Mode_t mode; // 模式 0 = pwm+arr, 1 = oc+ccr

    TIM_HandleTypeDef *htim;
    uint32_t tim_channel; // 定时器通道
    // DMA缓冲区
    uint8_t hdma_id;
    uint16_t buffer_size;                                                  // DMA缓冲区大小
    uint16_t dma_buffer[MAX_DMA_BUFFER_SIZE] __attribute__((aligned(32))); // DMA缓冲区
    uint32_t total_data;                                                   // 总数据长度（单位：字节）
    uint32_t transfered_data;                                              // 已传输数据长度（单位：字节）
    uint16_t g_last_accum;                                                 // 上一次累加值

    volatile uint8_t active_buffer;    // 当前活动缓冲区（0或1）
    volatile uint8_t next_fill_buffer; // 下一次要填充的缓冲区(0填充前半区，1填充后半区，255表示已填充)
    // volatile uint8_t fill_flag;        // 正在填充标志
    volatile uint8_t dma_buffer_0_filled;
    volatile uint8_t dma_buffer_1_filled;

    volatile uint32_t fill_buffer_in_background_count;
    void (*transfer_complete_callback)(void *context);
    int32_t (*on_fill_buffer)(void *context);
    void *callback_context; // 上下文参数（用户自定义数据）

} DMA_DB_t;

extern uint32_t start_time;
void dma_db_start(DMA_DB_t *dma_db);
void dma_db_stop(DMA_DB_t *dma_db);

void dma_db_fill_in_background(DMA_DB_t *dma_db);

void dma_db_half_transfer_it_cb_handle(DMA_DB_t *dma_db);
void dma_db_transfer_complete_it_cb_handle(DMA_DB_t *dma_db);

#endif /* __DMA_DB_H */