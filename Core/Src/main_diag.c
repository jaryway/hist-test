/* 临时诊断代码：放在 HAL_TIM_Base_Start_DMA(...) 之后执行 */
#include "stm32f1xx_hal.h"
#include <stdio.h>

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

/* 如果你在 stm32f1xx_it.c 中加入了 GPIO toggle（在 IRQ handler），也请告诉我你是否观察到 LED 翻转或示波器信号 */