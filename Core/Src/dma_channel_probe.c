#include "main.h"
#include "tim.h"
#include "dma.h"
#include <stdio.h>

/* 外部 hdma 句柄（在 tim.c 中定义）*/
extern DMA_HandleTypeDef hdma_tim3_ch1_trig;
extern TIM_HandleTypeDef htim3;

/* Helper: return DMA1_ChannelX pointer for index 1..7 */
static DMA_Channel_TypeDef *dma_channel_ptr(int ch)
{
    switch (ch)
    {
    case 1:
        return DMA1_Channel1;
    case 2:
        return DMA1_Channel2;
    case 3:
        return DMA1_Channel3;
    case 4:
        return DMA1_Channel4;
    case 5:
        return DMA1_Channel5;
    case 6:
        return DMA1_Channel6;
    case 7:
        return DMA1_Channel7;
    default:
        return NULL;
    }
}

/* Try to find which DMA1 channel responds to TIM3 Update requests.
   Returns channel number (1..7) if found, 0 if none found.
   WARNING: this temporarily re-initializes hdma_tim3_ch1_trig.Instance and re-inits HAL DMA for testing.
*/
int dma_probe_find_tim3_update_channel(void)
{
    uint16_t length = 7;
    static uint16_t dma_buffer[] = {288, 4608, 2304, 1152, 576, 288, 144};

    printf("Starting DMA channel probe for TIM3 Update...\r\n");

    /* Ensure TIM running */
    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
    {
        printf("HAL_TIM_PWM_Start failed\r\n");
        return 0;
    }

    /* Make sure TIM3->ARR is target (we'll reassign CPAR after setting Instance) */
    for (int ch = 1; ch <= 7; ++ch)
    {
        DMA_Channel_TypeDef *chptr = dma_channel_ptr(ch);
        if (chptr == NULL)
            continue;

        /* Stop any previous DMA if running */
        HAL_DMA_Abort_IT(&hdma_tim3_ch1_trig);
        HAL_DMA_DeInit(&hdma_tim3_ch1_trig);

        /* Assign instance to tested channel */
        hdma_tim3_ch1_trig.Instance = chptr;

        /* Keep previous Init settings but re-init (Mode/CIRC/priority/etc) */
        hdma_tim3_ch1_trig.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_tim3_ch1_trig.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_tim3_ch1_trig.Init.MemInc = DMA_MINC_ENABLE;
        hdma_tim3_ch1_trig.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        hdma_tim3_ch1_trig.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
        hdma_tim3_ch1_trig.Init.Mode = DMA_CIRCULAR;
        hdma_tim3_ch1_trig.Init.Priority = DMA_PRIORITY_MEDIUM;

        if (HAL_DMA_Init(&hdma_tim3_ch1_trig) != HAL_OK)
        {
            printf("HAL_DMA_Init failed for channel %d\r\n", ch);
            continue;
        }

        /* Force CPAR to TIM3->ARR (important) */
        hdma_tim3_ch1_trig.Instance->CPAR = (uint32_t)(&(TIM3->ARR));

        /* Start DMA on this channel to TIM3->ARR */
        if (HAL_DMA_Start_IT(&hdma_tim3_ch1_trig, (uint32_t)dma_buffer, (uint32_t)&(TIM3->ARR), length) != HAL_OK)
        {
            printf("HAL_DMA_Start_IT failed for channel %d\r\n", ch);
            HAL_DMA_DeInit(&hdma_tim3_ch1_trig);
            continue;
        }

        /* Enable NVIC for this channel temporarily */
        HAL_NVIC_EnableIRQ((IRQn_Type)((int)DMA1_Channel1_IRQn + (ch - 1)));

        /* Give TIM some time to produce updates and potential DMA requests */
        HAL_Delay(200);

        /* Read CNDTR of this channel to see if it decreased (indicating transfers) */
        uint32_t cndtr = hdma_tim3_ch1_trig.Instance->CNDTR;
        printf("Probe ch%d: CNDTR=%lu CCR=0x%08lx ISR=0x%08lx\r\n",
               ch, (unsigned long)cndtr, (unsigned long)hdma_tim3_ch1_trig.Instance->CCR, (unsigned long)DMA1->ISR);

        /* Stop and clean up this trial */
        HAL_DMA_Abort_IT(&hdma_tim3_ch1_trig);
        HAL_DMA_DeInit(&hdma_tim3_ch1_trig);
        HAL_NVIC_DisableIRQ((IRQn_Type)((int)DMA1_Channel1_IRQn + (ch - 1)));

        /* If CNDTR != length then channel responded (decreased) */
        if (cndtr != length)
        {
            printf("Found TIM3 Update -> DMA channel: %d (CNDTR changed from %u to %lu)\r\n", ch, length, (unsigned long)cndtr);
            return ch;
        }
    }

    printf("Probe finished: no responding DMA channel found for TIM3 Update\r\n");
    return 0;
}