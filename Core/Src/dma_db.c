
#include "dma_db.h"
#include <string.h>
#include <stm32f1xx_hal_tim.h>
#include <stdio.h>

uint32_t start_time;

// static void __dma_db_fill_buffer(DMA_DB_t *dma_db)
// {
//     // uint32_t start_index = dma_db->pulses_filled;
//     uint16_t half_size = dma_db->buffer_size / 2;
//     // uint16_t temp_buffer[half_size] __attribute__((aligned(32)));

//     uint16_t *dma_buffer = dma_db->dma_buffer;

//     uint16_t *dst = NULL;
//     if (dma_db->next_fill_buffer == 0) {
//         dst = &dma_buffer[0];
//     } else if (dma_db->next_fill_buffer == 1) {
//         dst = &dma_buffer[half_size];
//     } else {
//         return; // next_fill_buffer == 0xFF 或非法
//     }

//     for (uint16_t i = 0; i < half_size; i++) {

//         if (dma_db->mode == PWM_ARR) {
//             // uint32_t pulse_index = start_index + i;

//             // if (pulse_index >= dma_db->total_pulses)
//             // {
//             //     pulse_index = dma_db->total_pulses - 1;
//             // }

//             // uint32_t arr32 = dma_db_generate_t_arr(dma_db, pulse_index);
//             // temp_buffer[i] = (uint16_t)(arr32 & 0xFFFF);
//             // dma_db->pulses_filled++;
//         } else {
//             int32_t step_delay = 1000;
//             if (dma_db->on_fill_buffer)
//                 step_delay = dma_db->on_fill_buffer(dma_db->callback_context);

//             if (step_delay < 0)
//                 step_delay = 0;

//             uint16_t next_val_16 = (uint16_t)((dma_db->g_last_accum + step_delay) & 0xFFFF);
//             dma_db->g_last_accum = next_val_16;

//             // printf("cnt=%u\r\n", dma_db->g_last_accum);
//             // temp_buffer[i] = dma_db->g_last_accum;
//             dst[i] = dma_db->g_last_accum;
//         }
//     }

//     if (dma_db->next_fill_buffer == 0) {
//         // memcpy(&dma_buffer[0], temp_buffer, half_size * sizeof(uint16_t)); // 填充前半区
//         dma_db->dma_buffer_0_filled = 1;
//     } else if (dma_db->next_fill_buffer == 1) {
//         // memcpy(&dma_buffer[half_size], temp_buffer, half_size * sizeof(uint16_t)); // 填充后半区
//         dma_db->dma_buffer_1_filled = 1;
//     }
// }

static void _dma_db_fill_half(DMA_DB_t *dma_db, uint8_t which_half)
{
    uint16_t half_size   = dma_db->buffer_size / 2;
    uint16_t *dma_buffer = dma_db->dma_buffer;

    uint16_t *dst = (which_half == 0) ? &dma_buffer[0] : &dma_buffer[half_size];

    for (uint16_t i = 0; i < half_size; i++) {
        if (dma_db->mode == PWM_ARR) {
            // TODO: dst[i] = ...
        } else { // OC_CCR
            int32_t step_delay = 1000;
            if (dma_db->on_fill_buffer) {
                step_delay = dma_db->on_fill_buffer(dma_db->callback_context);
            }
            if (step_delay < 0) step_delay = 0;
            if (step_delay > 0xFFFF) step_delay = 0xFFFF;

            // dma_db->g_last_accum = (uint16_t)(dma_db->g_last_accum + (uint16_t)step_delay);
            dma_db->g_last_accum += (uint16_t)step_delay;
            dst[i] = dma_db->g_last_accum;
        }
    }

    if (which_half == 0) {
        dma_db->dma_buffer_0_filled = 1;
    } else {
        dma_db->dma_buffer_1_filled = 1;
    }
}

static void _dma_db_switch_buffer(DMA_DB_t *dma_db)
{
    if (dma_db->active_buffer == 0) {
        dma_db->active_buffer       = 1;
        dma_db->dma_buffer_0_filled = 0;
        if (dma_db->dma_buffer_1_filled != 1) {
            // printf("Warning: DMA buffer 1 not filled in time!\r\n");
        }
        dma_db->next_fill_buffer = 0;
    } else {
        dma_db->active_buffer       = 0;
        dma_db->dma_buffer_1_filled = 0;
        if (dma_db->dma_buffer_0_filled != 1) {
            // printf("Warning: DMA buffer 0 not filled in time!\r\n");
        }
        dma_db->next_fill_buffer = 1;
    }
}

void dma_db_start(DMA_DB_t *dma_db)
{
    dma_db->dma_buffer_0_filled = 0;
    dma_db->dma_buffer_1_filled = 0;
    if (dma_db->total_data > MAX_DMA_BUFFER_SIZE) {
        uint16_t buffer_size = MAX_DMA_BUFFER_SIZE;
        while (dma_db->total_data % buffer_size != 0) {
            buffer_size--;
        }
        dma_db->buffer_size = buffer_size; // 确保总脉冲数能被缓冲区大小整除
    } else {
        dma_db->buffer_size = dma_db->total_data;
    }

    /* 初始化时，两个缓冲区都填充数据 */
    dma_db->next_fill_buffer = 0;
    _dma_db_fill_half(dma_db, 0);

    if (dma_db->total_data > dma_db->buffer_size / 2) {
        dma_db->next_fill_buffer = 1;
        // _dma_db_fill_buffer(dma_db);
        _dma_db_fill_half(dma_db, 1);
    }

    dma_db->active_buffer                   = 0;
    dma_db->next_fill_buffer                = 0xFF;
    dma_db->transfered_data                 = 0;
    dma_db->fill_buffer_in_background_count = 0;

    printf("buffer_size: %u\r\n", dma_db->buffer_size);
    printf("total_data: %lu\r\n", dma_db->total_data);

    if (dma_db->mode == PWM_ARR) {
        HAL_TIM_PWM_Start_DMA(dma_db->htim, dma_db->tim_channel, (uint32_t *)dma_db->dma_buffer, dma_db->buffer_size);
    } else if (dma_db->mode == OC_CCR) {
        start_time = HAL_GetTick();
        HAL_TIM_OC_Start_DMA(dma_db->htim, dma_db->tim_channel, (uint32_t *)dma_db->dma_buffer, dma_db->buffer_size);
    }
}

void dma_db_stop(DMA_DB_t *dma_db)
{
    if (dma_db->mode == PWM_ARR) {
        HAL_TIM_PWM_Stop_DMA(dma_db->htim, dma_db->tim_channel);
    } else if (dma_db->mode == OC_CCR) {
        HAL_TIM_OC_Stop_DMA(dma_db->htim, dma_db->tim_channel);
    }
}

void dma_db_fill_in_background(DMA_DB_t *dma_db)
{

    if (dma_db->transfered_data >= dma_db->total_data || dma_db->filling)
        return;

    uint8_t which;

    // 极短临界区：拿到 which 并清除 next_fill_buffer，避免重复填同一半
    __disable_irq();
    if (dma_db->filling) {
        __enable_irq();
        return;
    }
    which = dma_db->next_fill_buffer;
    if (which != 0 && which != 1) {
        __enable_irq();
        return;
    }
    dma_db->filling          = 1;
    dma_db->next_fill_buffer = 0xFF;
    __enable_irq();

    dma_db->fill_buffer_in_background_count++;
    _dma_db_fill_half(dma_db, which);

    dma_db->filling = 0;
}

void dma_db_half_transfer_it_cb_handle(DMA_DB_t *dma_db)
{
    _dma_db_switch_buffer(dma_db);
}

void dma_db_transfer_complete_it_cb_handle(DMA_DB_t *dma_db)
{
    uint32_t temp = dma_db->transfered_data + dma_db->buffer_size;
    // 判断脉冲是否发送完成，如果已经发送完成，停止DMA传输
    if (temp >= dma_db->total_data) {
        dma_db->transfered_data = dma_db->total_data; // 重新修正发送位置
        if (dma_db->mode == PWM_ARR) {
            HAL_TIM_PWM_Stop(dma_db->htim, dma_db->tim_channel);
            HAL_TIM_Base_Stop_DMA(dma_db->htim);
        } else if (dma_db->mode == OC_CCR) {
            HAL_TIM_OC_Stop_DMA(dma_db->htim, dma_db->tim_channel);
        }

        if (dma_db->transfer_complete_callback != NULL) {
            dma_db->transfer_complete_callback(dma_db->callback_context);
        }

        return;
    }

    dma_db->transfered_data = temp;
    _dma_db_switch_buffer(dma_db);
}

uint32_t dma_db_get_sent_elements(const DMA_DB_t *db)
{
    DMA_HandleTypeDef *hdma = db->htim->hdma[db->hdma_id];
    if (hdma == NULL) return 0;

    uint32_t cndtr         = hdma->Instance->CNDTR; // 当前循环剩余元素数
    uint32_t sent_in_round = (db->buffer_size >= cndtr) ? (db->buffer_size - cndtr) : 0;

    return db->transfered_data + sent_in_round; // 全局已发送元素数
}

uint32_t dma_db_get_remaining_elements(const DMA_DB_t *db)
{
    uint32_t sent = dma_db_get_sent_elements(db);
    if (sent >= db->total_data) return 0;
    return db->total_data - sent;
}