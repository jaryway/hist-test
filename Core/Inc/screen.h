/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    screen.h
 * @brief   This file contains all the function prototypes for
 *          the screen.c file
 */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SCREEN_H__
#define __SCREEN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#define DWIN_DGUS_MAX_DATA_LEN 256

typedef enum {
    SN_OK = 0,
    SN_ERR_TIMEOUT,
    SN_ERR_CRC,
    SN_ERR_EXCEPTION,
    SN_ERR_BAD_RESPONSE,
    SN_ERR_HW,
    SN_ERR_PARAM
} SN_Status_t;

typedef struct {
    UART_HandleTypeDef *huart;
} Screen_t;

/* 调试开关：定义为1 将打印发送/接收帧（使用 printf）*/
#ifndef SCREEN_DEBUG
#define SCREEN_DEBUG 1
#endif

void screen_init(Screen_t *dwin, UART_HandleTypeDef *huart);

uint8_t *screen_read_data(Screen_t *dwin, uint16_t var_addr, uint8_t len);
/**
 * @brief  写入DWIN标签
 * @param  dwin: DWIN句柄
 * @param  var_addr: 变量地址
 * @param  text: 文本
 * @retval 状态
 */
SN_Status_t screen_write_text(Screen_t *dwin, uint16_t var_addr, const char *text);
/**
 * @brief  写入DWIN标签
 * @param  dwin: DWIN句柄
 * @param  var_addr: 变量地址
 * @param  bytes: 字节数组
 * @param  len: 字节长度
 * @retval 状态
 */
SN_Status_t screen_write_str_bytes(Screen_t *dwin, uint16_t var_addr, const uint8_t *bytes, uint8_t len);
/**
 * @brief  切换DWIN页面
 * @param  dwin: DWIN句柄
 * @param  page: 页面编号
 * @retval 状态
 */
SN_Status_t screen_switch_page(Screen_t *dwin, uint8_t page);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__SCREEN_H__ */
