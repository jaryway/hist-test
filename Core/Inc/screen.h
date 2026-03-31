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

#define DWIN_DGUS_MAX_DATA_LEN    256
#define SN_REG_ADDR_COMPANY_NAME  0x1000 // 公司名称
#define SN_REG_ADDR_SLOGAN        0x1100 // 公司口号
#define SN_REG_ADDR_TOT_CNT_TXT   0x1200 // 总计数标签
#define SN_REG_ADDR_TOT_CNT_VAL   0x1300 // 总计数值
#define SN_REG_ADDR_RPM_TXT       0x1400 // 转速标签
#define SN_REG_ADDR_RPM_VAL       0x1500 // 转速值
#define SN_REG_ADDR_CUR_CNT_TXT   0x1600 // 当前计数标签
#define SN_REG_ADDR_CUR_CNT_VAL   0x1700 // 当前计数值
#define SN_REG_ADDR_POS_TXT       0x1800 // 位置标签
#define SN_REG_ADDR_POS_VAL       0x1900 // 位置值
#define SN_REG_ADDR_MODE_TXT      0x1A00 // 模式标签
#define SN_REG_ADDR_MODE_VAL      0x1B00 // 模式值
#define SN_REG_ADDR_STA_TXT       0x1C00 // 状态标签
#define SN_REG_ADDR_STA_VAL       0x1D00 // 状态值
#define SN_REG_ADDR_LOAD_RATE_TXT 0x1E00 // 负载率标签
#define SN_REG_ADDR_LOAD_RATE_VAL 0x1F00 // 负载率值
#define SN_REG_ADDR_DN_TXT        0x2000 // 下限位标签
#define SN_REG_ADDR_DN_VAL        0x2001 // 下限位值
#define SN_REG_ADDR_UP_TXT        0x2002 // 上限位标签
#define SN_REG_ADDR_UP_VAL        0x2003 // 上限位值
#define SN_REG_ADDR_LD_TXT        0x2004 // 装载标签
#define SN_REG_ADDR_LD_VAL        0x2005 // 装载值

// 上烟 (4字节)
static const uint8_t shangyan[] = {
    0xC9, 0xCF, // 上
    0xD1, 0xCC  // 烟
};

// 下烟 (4字节)
static const uint8_t xiayan[] = {
    0xCF, 0xC2, // 下
    0xD1, 0xCC  // 烟
};

// 运行中
static const uint8_t running[] = {
    0xD4, 0xCB, // 运
    0xD0, 0xD0, // 行
    0xD6, 0xD0  // 中
};
// 急停
static const uint8_t stoped[] = {
    0xBC, 0xB1, // 急
    0xCD, 0xA3  // 停
};

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

#ifndef SCREEN_TYPE
#define SCREEN_TYPE 1 /* 1: DGUS 2: TSUIC1*/
#endif

void screen_init(Screen_t *screen, UART_HandleTypeDef *huart);
/**
 * @brief  读取数据
 */
uint8_t *screen_read_data(Screen_t *screen, uint16_t var_addr, uint8_t len);
/**
 * @brief  写入数据(不支持中文)
 * @param  screen: screen句柄
 * @param  var_addr: 变量地址
 * @param  text: 文本
 * @retval 状态
 */
SN_Status_t screen_write_text(Screen_t *screen, uint16_t var_addr, const char *text);
/**
 * @brief  写入数据
 * @param  screen: screen句柄
 * @param  var_addr: 变量地址
 * @param  bytes: 字节数组
 * @param  len: 字节长度
 * @retval 状态
 */
SN_Status_t screen_write_str_bytes(Screen_t *screen, uint16_t var_addr, const uint8_t *bytes, uint8_t len);
/**
 * @brief  切换页面
 * @param  screen: screen 句柄
 * @param  page: 页面编号
 * @retval 状态
 */
SN_Status_t screen_switch_page(Screen_t *screen, uint8_t page);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__SCREEN_H__ */
