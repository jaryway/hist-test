
#include "dma_doublebuffer.h"
#include <string.h>
#include <stm32f1xx_hal_tim.h>
#include <stdio.h>

static uint8_t flag = 0;
static float generate_trapezoid_freq(DMA_DoubleBuffer_t *dma_doublebuffer, uint32_t pulse_index)
{
    // 运动参数
    uint32_t total_pulses = dma_doublebuffer->total_pulses;
    uint32_t accel_pulses = dma_doublebuffer->accel_pulses;
    uint32_t decel_pulses = dma_doublebuffer->decel_pulses;
    uint32_t cruise_pulses = total_pulses - accel_pulses - decel_pulses;
    uint32_t max_rpm = dma_doublebuffer->max_rpm;
    uint32_t pulses_per_rev = dma_doublebuffer->pulses_per_rev;

    // 频率参数（单位：Hz）
    // uint16_t PSC = dma_doublebuffer->htim->Instance->PSC;
    // float start_freq = 72000000.0f / (float)(PSC + 1);               // 起始频率
    float start_freq = 1000;                                         // 起始频率
    float max_freq = (float)max_rpm / 60.0f * (float)pulses_per_rev; // 最大频率
    // float end_freq = 72000000.0f / (float)(PSC + 1);                 // 结束频率
    float end_freq = 1000; // 结束频率

    // printf("start_freq: %f Hz\n", start_freq);

    // 验证参数
    if (max_freq > 250000.0f)
        max_freq = 250000.0f; // 限制最大频率
    if (start_freq > max_freq)
        start_freq = max_freq;
    if (end_freq > max_freq)
        end_freq = max_freq;

    float freq_hz;

    if (pulse_index < accel_pulses)
    {
        // 加速段：线性加速
        float ratio = (float)pulse_index / accel_pulses;
        // 使用线性插值
        freq_hz = start_freq + (max_freq - start_freq) * ratio;
    }
    else if (pulse_index < accel_pulses + cruise_pulses)
    {
        // 匀速段
        freq_hz = max_freq;
    }
    else if (pulse_index < total_pulses)
    {
        // 减速段：线性减速
        uint32_t decel_start = accel_pulses + cruise_pulses;
        float ratio = (float)(pulse_index - decel_start) / decel_pulses;
        // 从最大频率减速到结束频率
        freq_hz = max_freq - (max_freq - end_freq) * ratio;
    }
    else
    {
        // 不应该执行到这里
        freq_hz = end_freq;
    }

    return freq_hz;
}

void dma_doublebuffer_init(DMA_DoubleBuffer_t *dma_doublebuffer)
{
    // dma_doublebuffer_check_and_adjust(dma_doublebuffer);
    // 使用 OC + CCR 模式时，初始化 last_accum
    if (dma_doublebuffer->mode == OC_CCR)
    {
        /* align accumulator to current counter to ensure CCR timings are relative to TIM CNT */
        dma_doublebuffer->g_last_accum = (uint64_t)__HAL_TIM_GET_COUNTER(dma_doublebuffer->htim);
        dma_doublebuffer->g_last_accum += 100ULL; /* small offset to avoid immediate match */
        // uint16_t psc1 = dma_doublebuffer->htim->Instance->PSC;
        // printf("psc=%d\n", psc1);

        // dma_doublebuffer->tick_hz=
    }

    /* fill initial buffer - caller should update pulses_filled accordingly */
    /* 初始化时，两个缓冲区都填充数据 */
    dma_doublebuffer->next_fill_buffer = 0;
    dma_doublebuffer_fill(dma_doublebuffer);
    dma_doublebuffer->next_fill_buffer = 1;
    dma_doublebuffer_fill(dma_doublebuffer);

    dma_doublebuffer->active_buffer = 0;
    dma_doublebuffer->next_fill_buffer = 0xFF;
    dma_doublebuffer->fill_buffer_in_background_count = 0;
}
void dma_doublebuffer_fill(DMA_DoubleBuffer_t *dma_doublebuffer)
{
    uint32_t start_index = dma_doublebuffer->pulses_filled;
    uint16_t half_size = dma_doublebuffer->buffer_size / 2;
    uint16_t temp_buffer[half_size];
    uint16_t *dma_buffer = dma_doublebuffer->dma_buffer;

    for (uint16_t i = 0; i < half_size; i++)
    {
        dma_doublebuffer->pulses_filled++;
        uint32_t pulse_index = start_index + i;

        if (pulse_index >= dma_doublebuffer->total_pulses)
        {
            pulse_index = dma_doublebuffer->total_pulses - 1;
        }

        if (dma_doublebuffer->mode == PWM_ARR)
        {
            if (dma_doublebuffer->pulses_filled >= dma_doublebuffer->total_pulses)
            {
                uint32_t arr32 = dma_doublebuffer_generate_t_arr(dma_doublebuffer, pulse_index);
                temp_buffer[i] = (uint16_t)(arr32 & 0xFFFF);
            }
            else
            {
                temp_buffer[i] = 0;
            }
        }
        else
        {
            // uint32_t ccr32 = dma_doublebuffer->g_last_accum + 4;
            // dma_doublebuffer->g_last_accum = ccr32;

            uint32_t ccr32 = dma_doublebuffer_generate_t_ccr(dma_doublebuffer, pulse_index);
            temp_buffer[i] = (uint16_t)(ccr32 & 0xFFFF);
        }
    }

    if (dma_doublebuffer->next_fill_buffer == 0)
    {
        memcpy(&dma_buffer[0], temp_buffer, half_size * sizeof(uint16_t)); // 填充前半区
    }
    else if (dma_doublebuffer->next_fill_buffer == 1)
    {
        memcpy(&dma_buffer[half_size], temp_buffer, half_size * sizeof(uint16_t)); // 填充后半区
    }
}

void dma_doublebuffer_switch(DMA_DoubleBuffer_t *dma_doublebuffer)
{
    if (dma_doublebuffer->active_buffer == 0)
    {
        dma_doublebuffer->active_buffer = 1;
        dma_doublebuffer->next_fill_buffer = 0;
    }
    else
    {
        dma_doublebuffer->active_buffer = 0;
        dma_doublebuffer->next_fill_buffer = 1;
    }
}

void dma_doublebuffer_fill_in_background(DMA_DoubleBuffer_t *dma_doublebuffer)
{
    dma_doublebuffer->fill_buffer_in_background_count++;
    if (dma_doublebuffer->next_fill_buffer == 0xFF)
        return;

    dma_doublebuffer_fill(dma_doublebuffer);

    dma_doublebuffer->next_fill_buffer = 0xFF;
    // dma_doublebuffer_check_and_adjust(dma_doublebuffer);
}

// void dma_doublebuffer_check_and_adjust(DMA_DoubleBuffer_t *dma_doublebuffer)
// {
//     uint32_t remaining_pulses = dma_doublebuffer->total_pulses - dma_doublebuffer->pulses_filled;

//     if (remaining_pulses < MAX_BUFFER_SIZE && flag == 0)
//     {
//         flag = 1;
//         DMA_HandleTypeDef *hdma = dma_doublebuffer->htim->hdma[dma_doublebuffer->hdma_id];

//         printf("dma_doublebuffer_check_and_adjust: remaining_pulses=%lu,CNDTR:%lu\n", remaining_pulses, hdma->Instance->CNDTR);
//         // __HAL_DMA_DISABLE(hdma);
//         hdma->Instance->CNDTR = remaining_pulses; // 设置DMA传输数量
//         // hdma->Instance->CCR &= ~DMA_CCR_HTIE;     // 禁用半传输中断
//         // __HAL_DMA_ENABLE(hdma);
//     }
// }

uint8_t dma_doublebuffer_check_finished(DMA_DoubleBuffer_t *dma_doublebuffer)
{
    uint32_t temp = dma_doublebuffer->pulses_sent + dma_doublebuffer->buffer_size / 2;

    // 判断脉冲是否发送完成，如果已经发送完成，停止DMA传输
    if (temp >= dma_doublebuffer->total_pulses)
    {
        dma_doublebuffer->pulses_sent = dma_doublebuffer->total_pulses; // 重新修正发送位置
        if (dma_doublebuffer->mode == PWM_ARR)
        {
            HAL_TIM_PWM_Stop(dma_doublebuffer->htim, dma_doublebuffer->tim_channel);
            HAL_TIM_Base_Stop_DMA(dma_doublebuffer->htim);
        }
        else if (dma_doublebuffer->mode == OC_CCR)
        {
            HAL_TIM_OC_Stop_DMA(dma_doublebuffer->htim, dma_doublebuffer->tim_channel);
        }
        return 1;
    }

    dma_doublebuffer->pulses_sent = temp;
    return 0;
}

uint32_t dma_doublebuffer_generate_t_arr(DMA_DoubleBuffer_t *dma_doublebuffer, uint32_t pulse_index)
{
    float freq_hz = generate_trapezoid_freq(dma_doublebuffer, pulse_index);

    // const uint32_t SYS_CLK_HZ = 72000000; // 72MHz
    const float MAX_FREQ = 250000.0f; // 250KHz最大频率

    if (freq_hz < 1.0f)
        return 0xFFFF; // 停止信号

    // 限制最大频率
    if (freq_hz > MAX_FREQ)
        freq_hz = MAX_FREQ;

    // 计算ARR值
    uint32_t arr = (uint32_t)(SYS_CLK_HZ / freq_hz) - 1;

    // 限制最小ARR值（对应最大频率250KHz）
    uint32_t min_arr = (uint32_t)(SYS_CLK_HZ / MAX_FREQ) - 1; // ARR = 287
    if (arr < min_arr)
        arr = min_arr;

    // 限制最大ARR值（对应最小频率1Hz）
    // uint32_t max_arr = (uint32_t)(SYS_CLK_HZ / 1.0f) - 1; // ARR = 71,999,999
    // 但实际上我们不会用到这么低的频率，限制在1KHz对应的ARR即可
    uint32_t practical_max_arr = (uint32_t)MAX_ARR_VALUE - 1; // ARR = 65536
    if (arr > practical_max_arr)
        arr = practical_max_arr;

    return arr;
}

uint32_t dma_doublebuffer_generate_t_ccr(DMA_DoubleBuffer_t *dma_doublebuffer, uint32_t transfer_index)
{

    dma_doublebuffer->g_last_accum += 30;
    return dma_doublebuffer->g_last_accum;

    // // 翻转模式下，传输两次才是个完整的脉冲
    // dma_doublebuffer->g_last_accum;
    // dma_doublebuffer->step_delay = 40;
    // uint32_t tmp = dma_doublebuffer->g_last_accum + dma_doublebuffer->step_delay / 2;
    // dma_doublebuffer->g_last_accum = tmp;

    // if ((transfer_index + 1) % 2 == 0)
    // {
    //     //
    // }

    // return (uint32_t)(dma_doublebuffer->g_last_accum & 0xFFFFFFFFUL);

    // dma_doublebuffer->i

    // // uint32_t index = step_index / 2;
    // if ((step_index + 1) % 2 == 0)
    // {
    //     dma_doublebuffer->g_last_accum += (uint64_t)dma_doublebuffer->step_delay;
    //     return (uint32_t)(dma_doublebuffer->g_last_accum & 0xFFFFFFFFUL);
    // }

    // float freq_hz = generate_trapezoid_freq(dma_doublebuffer, step_index);

    // // return freq_hz;
    // // 防止除以零或极小值
    // if (freq_hz <= 1e-6f)
    //     freq_hz = 1.0f;

    // // period in seconds = 1 / freq
    // double period_sec = 1.0 / (double)freq_hz; // 得到当前频率下电机走一步所需要的时间
    // uint16_t PSC = dma_doublebuffer->htim->Instance->PSC;
    // const double TICK_HZ = 1.0 / ((double)72000000.0 / (double)(PSC + 1)); // 定时器走一步所需的时间 0.000001
    // // double max_freg = dma_doublebuffer->max_rpm * dma_doublebuffer->pulses_per_rev / 60; // 电机最高转速时，走一步所需的时间
    // // double motor_min_tick = 1.0 / max_freg;                                              // 0.000004
    // // double min_ccr = motor_min_tick / TICK_HZ;                                           // 40

    // dma_doublebuffer->step_delay = period_sec / TICK_HZ;

    // if ((step_index + 1) % 2 == 0)
    // {
    //     dma_doublebuffer->g_last_accum += (uint64_t)dma_doublebuffer->step_delay;
    // }
    // /* 返回低 32 位绝对时间点（写入 DMA buffer 时只取低16位） */
    // uint32_t ccr = (uint32_t)(dma_doublebuffer->g_last_accum & 0xFFFFFFFFUL);
    // //
    // printf("step_index:%lu ,freq_hz: %.2f ccr: %lu,step_delay:%u\n", step_index, freq_hz, ccr, dma_doublebuffer->step_delay);
    // return ccr;
}