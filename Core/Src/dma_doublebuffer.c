
#include "dma_doublebuffer.h"
#include <string.h>
#include <stm32f1xx_hal_tim.h>
#include <stdio.h>

extern TIM_HandleTypeDef htim3;

static const double TICK_HZ = 1000000.0; // 1MHz计数频率
/* runtime tick frequeng_tick_hzcy (ticks per second) - will be updated from TIM3 PSC at init */
// static double g_tick_hz = 1000000.0; /* default 1 MHz */

// static uint64_t g_last_accum = 0;        // 记录上一个绝对CCR时间点
// 频率到ARR的转换函数
static uint32_t freq_to_arr(float freq_hz)
{
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

static float generate_trapezoid_freq(DMA_DoubleBuffer_t *dma_doublebuffer, uint32_t pulse_index)
{
    // 运动参数
    uint32_t total_pulses = dma_doublebuffer->total_pulses;
    uint32_t accel_pulses = dma_doublebuffer->accel_pulses;
    uint32_t decel_pulses = dma_doublebuffer->decel_pulses;
    uint32_t cruise_pulses = total_pulses - accel_pulses - decel_pulses;
    uint32_t rpm = dma_doublebuffer->rpm;
    uint32_t pulses_per_rev = dma_doublebuffer->pulses_per_rev;

    // 频率参数（单位：Hz）
    float start_freq = SYS_CLK_HZ / 65536;                       // 起始频率 1099Hz
    float max_freq = (float)rpm / 60.0f * (float)pulses_per_rev; // 最大频率 = 250KHz
    float end_freq = SYS_CLK_HZ / 65536;                         // 结束频率 1099Hz

    // 验证参数
    if (max_freq > 250000.0f)
        max_freq = 250000.0f; // 限制最大频率
    if (start_freq > max_freq)
        start_freq = max_freq;
    if (end_freq > max_freq)
        end_freq = max_freq;

    float current_freq;

    if (pulse_index < accel_pulses)
    {
        // 加速段：线性加速
        float ratio = (float)pulse_index / accel_pulses;
        // 使用线性插值
        current_freq = start_freq + (max_freq - start_freq) * ratio;
    }
    else if (pulse_index < accel_pulses + cruise_pulses)
    {
        // 匀速段
        current_freq = max_freq;
    }
    else if (pulse_index < total_pulses)
    {
        // 减速段：线性减速
        uint32_t decel_start = accel_pulses + cruise_pulses;
        float ratio = (float)(pulse_index - decel_start) / decel_pulses;
        // 从最大频率减速到结束频率
        current_freq = max_freq - (max_freq - end_freq) * ratio;
    }
    else
    {
        // 不应该执行到这里
        current_freq = end_freq;
    }

    return current_freq;
}
/* 根据 pulse_index 返回该脉冲对应的周期（以 timer ticks 为单位）
   使用梯形速度剖面（与原 generate_trapezoid_arr 相同的分段逻辑），
   但这里返回 period_ticks = round(tick_hz / freq)
*/

uint32_t generate_trapezoid_arr(DMA_DoubleBuffer_t *dma_doublebuffer, uint32_t pulse_index)
{
    // (void)dma_doublebuffer; // 未使用该参数，避免编译器警告
    // (void)pulse_index;
    float current_freq = generate_trapezoid_freq(dma_doublebuffer, pulse_index);
    return freq_to_arr(current_freq);
}

uint32_t generate_trapezoid_ccr(DMA_DoubleBuffer_t *dma_doublebuffer, uint32_t pulse_index)
{
    // 假设 PSC =0, ARR = 65535
    float current_freq = generate_trapezoid_freq(dma_doublebuffer, pulse_index);
    // 防止除以零或极小值
    if (current_freq <= 1e-6f)
        current_freq = 1.0f;

    // period in seconds = 1 / freq
    double period_sec = 1.0 / (double)current_freq; // 得到当前频率下的时间间隔

    // convert to ticks
    uint32_t period_ticks = (uint32_t)(period_sec * TICK_HZ + 0.5);

    if (period_ticks == 0)
        period_ticks = 1;

    dma_doublebuffer->g_last_accum += (uint64_t)period_ticks;
    /* 返回低 32 位绝对时间点（写入 DMA buffer 时只取低16位） */
    return (uint32_t)(dma_doublebuffer->g_last_accum & 0xFFFFFFFFUL);
}

void fill_single_buffer(DMA_DoubleBuffer_t *dma_doublebuffer, uint32_t start_idx)
{
    uint16_t count = BUFFER_SIZE / 2;
    uint16_t temp_buffer[count];
    uint16_t *buffer = dma_doublebuffer->dma_buffer;

    for (uint16_t i = 0; i < count; i++)
    {
        uint32_t pulse_idx = start_idx + i;
        pulse_idx = pulse_idx >= dma_doublebuffer->total_pulses ? dma_doublebuffer->total_pulses - 1 : pulse_idx;
        if (dma_doublebuffer->mode == PWM_ARR)
        {
            uint32_t arr = generate_trapezoid_arr(dma_doublebuffer, pulse_idx);
            // printf("pulse_idx:%lu,temp_buffer[%u]:%lu\r\n", pulse_idx, i, arr);
            temp_buffer[i] = arr;
        }
        else
        {

            // uint32_t ccr = dma_doublebuffer->g_last_accum + 1200;
            // dma_doublebuffer->g_last_accum = ccr;
            // printf("oc_ccr_pulse_idx:%lu,temp_buffer[%u]:%lu\r\n", pulse_idx, i, ccr);
            temp_buffer[i] = generate_trapezoid_ccr(dma_doublebuffer, pulse_idx);
        }
    }

    if (dma_doublebuffer->active_buffer == 0)
    {
        // 填充前半区
        memcpy(buffer, temp_buffer, count * sizeof(uint16_t));
    }
    else
    {
        // 填充后半区
        memcpy(&buffer[BUFFER_SIZE / 2], temp_buffer, count * sizeof(uint16_t));
    }
}
void fill_buffer(DMA_DoubleBuffer_t *dma_doublebuffer)
{
    uint32_t start_idx = dma_doublebuffer->pulses_filled;
    uint16_t fill_count = BUFFER_SIZE / 2;

    if (start_idx + fill_count > dma_doublebuffer->total_pulses)
    {
        fill_count = dma_doublebuffer->total_pulses - start_idx;
    }

    if (fill_count > 0)
    {
        fill_single_buffer(dma_doublebuffer, start_idx);
        dma_doublebuffer->pulses_filled += fill_count; // 更新填充位置
    }
}

void init_double_buffer(DMA_DoubleBuffer_t *dma_doublebuffer)
{
    // update_tick_hz_from_tim3();
    // 使用 OC + CCR 模式时，初始化 last_accum
    if (dma_doublebuffer->mode == 1)
    {
        /* align accumulator to current counter to ensure CCR timings are relative to TIM CNT */
        dma_doublebuffer->g_last_accum = (uint64_t)__HAL_TIM_GET_COUNTER(dma_doublebuffer->htim);
        dma_doublebuffer->g_last_accum += 10ULL; /* small offset to avoid immediate match */
    }
    else
    {
    }

    /* fill initial buffer - caller should update pulses_filled accordingly */
    /* 初始化时，两个缓冲区都填充数据 */
    dma_doublebuffer->active_buffer = 0;
    fill_buffer(dma_doublebuffer);
    dma_doublebuffer->active_buffer = 1;
    fill_buffer(dma_doublebuffer);

    dma_doublebuffer->active_buffer = 0;
    dma_doublebuffer->next_fill_buffer = 0xFF;
}
void fill_buffer_in_background(DMA_DoubleBuffer_t *dma_doublebuffer)
{
    if (dma_doublebuffer->next_fill_buffer == 0xFF)
        return;

    fill_buffer(dma_doublebuffer);

    dma_doublebuffer->next_fill_buffer = 0xFF;
}