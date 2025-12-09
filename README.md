PSC = 72MHz
CCR = 72
电机最高频率为 250KHz
通过 DMA 修改 ARR 值，实现频率变化，从而实现电机加减速
1、怎么通过 DMA 修改 ARR ？
2、怎么生成一个加减速的 ARR 序列？假设函数如下

```c

    void generate_trapezoid_ccr(uint32_t *buffer, uint32_t start_idx, uint16_t count)
    {
      uint32_t total_pulses = 50000;
      uint32_t accel_pulses = 15000;
      uint32_t decel_pulses = 15000;
      uint32_t cruise_pulses = total_pulses - accel_pulses - decel_pulses;
      // 3000/60*5000=250000Hz
      //  频率参数
      float start_freq = 1000.0f;        // 起始频率 (Hz)
      float max_freq = 3000 / 60 * 5000; // 最大频率 (Hz)
      float end_freq = 1000.0f;          // 结束频率 (Hz)

      // 计算CCR值（假设定时器时钟为72MHz）
      const uint32_t timer_clk = 72000000; // 72MHz
      // TODO: 根据加速脉冲数、匀速脉冲数和减速脉冲数，计算每个脉冲对应的频率
    }
    // 3、dma 双缓冲 怎么实现， 如何监听dma 半传输中断 和 完成传输中断？

    根据 HAL_TIM_PWM_Start_DMA 的代码
    htim->hdma[TIM_DMA_ID_CC1]->XferCpltCallback = TIM_DMADelayPulseCplt;
    htim->hdma[TIM_DMA_ID_CC1]->XferHalfCpltCallback = TIM_DMADelayPulseHalfCplt;

    // Set the DMA error callback
    htim->hdma[TIM_DMA_ID_CC1]->XferErrorCallback = TIM_DMAError ;

    // Enable the DMA channel
    if (HAL_DMA_Start_IT(htim->hdma[TIM_DMA_ID_CC1], (uint32_t)pData, (uint32_t)&htim->Instance->CCR1,
                          Length) != HAL_OK)

```

我们是不是可以模仿这个流程，自己实现一个 DMA 修改 ARR 的功能？

先验证使用 HAL_DMA_Start_IT 能不能修改 ARR 的值，能不能通过中断回调函数监听到 DMA 传输完成事件？
// 先写死一个 buffer ，然后通过 DMA 拷贝到 TIM1_CH1 引脚，然后通过中断回调函数监听 DMA 完成事件，然后修改 ARR 值


// dma 转运的条件
// 1、传输计数大于零
// 2、触发源有触发信号
// 3、DMA 使能