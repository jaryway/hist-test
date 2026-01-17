/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    gpio.c
 * @brief   This file provides code for the configuration
 *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
void MX_GPIO_Init(void)
{

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, BTN_LED_ST_Pin | BTN_LED_DN_Pin | BTN_LED_UP_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, BTN_SP_Pin | SENSOR_LD2_Pin | SENSOR_LD1_Pin | SENSOR_DN_Pin | SENSOR_UP_Pin | BTN_ST_Pin | RS485_RE_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, BTN_DN_Pin | BTN_UP_Pin | ENAB_Pin | ENA_Pin | DIR_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : BTN_LED_ST_Pin BTN_LED_DN_Pin BTN_LED_UP_Pin */
    GPIO_InitStruct.Pin   = BTN_LED_ST_Pin | BTN_LED_DN_Pin | BTN_LED_UP_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : BTN_SP_Pin SENSOR_LD2_Pin SENSOR_LD1_Pin SENSOR_DN_Pin
                             SENSOR_UP_Pin BTN_ST_Pin RS485_RE_Pin */
    GPIO_InitStruct.Pin   = BTN_SP_Pin | SENSOR_LD2_Pin | SENSOR_LD1_Pin | SENSOR_DN_Pin | SENSOR_UP_Pin | BTN_ST_Pin | RS485_RE_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : BTN_DN_Pin BTN_UP_Pin ENAB_Pin ENA_Pin
                             DIR_Pin */
    GPIO_InitStruct.Pin   = BTN_DN_Pin | BTN_UP_Pin | ENAB_Pin | ENA_Pin | DIR_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
