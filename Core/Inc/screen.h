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
// #define SN_REG_ADDR_STA_TXT       0x1C00 // 状态标签
// #define SN_REG_ADDR_STA_VAL       0x1D00 // 状态值
#define SN_REG_ADDR_LOAD_RATE_TXT 0x1C00 // 负载率标签
#define SN_REG_ADDR_LOAD_RATE_VAL 0x1D00 // 负载率值
#define SN_REG_ADDR_DN_TXT        0x1E00 // 下限位标签
#define SN_REG_ADDR_DN_VAL        0x1F00 // 下限位值
#define SN_REG_ADDR_UP_TXT        0x2000 // 上限位标签
#define SN_REG_ADDR_UP_VAL        0x2100 // 上限位值
#define SN_REG_ADDR_LD_TXT        0x2200 // 装载标签
#define SN_REG_ADDR_LD_VAL        0x2300 // 装载值
#define SN_REG_ADDR_HOMING        0x2700 // 回零中

static const uint8_t company_name[] = {
    // 0xD4, 0xC6, // 云
    // 0xC4, 0xCF, // 南
    // 0xC3, 0xF1, // 民
    // 0xB4, 0xB4, // 创
    // 0xC5, 0xA9, // 农
    // 0xD2, 0xB5, // 业
    // 0xBF, 0xC6, // 科
    // 0xBC, 0xBC, // 技
    // 0xD3, 0xD0, // 有
    // 0xCF, 0xDE, // 限
    // 0xB9, 0xAB, // 公
    // 0xCB, 0xBE  // 司
    0xD4, 0xC6, // 云
    0xC4, 0xCF, // 南
    0xCD, 0xF5, // 王
    0xBC, 0xD7, // 甲
    0xB9, 0xA4, // 工
    0xC3, 0xB3, // 贸
    0xD3, 0xD0, // 有
    0xCF, 0xDE, // 限
    0xB9, 0xAB, // 公
    0xCB, 0xBE  // 司
};
// 专注烟草机械化10年
static const uint8_t slogan[] = {
    0xD7, 0xA8, // 专
    0xD7, 0xA2, // 注
    0xD1, 0xCC, // 烟
    0xB2, 0xDD, // 草
    0xBB, 0xFA, // 机
    0xD0, 0xB5, // 械
    0xBB, 0xAF, // 化
    0x31, 0x30, // 10 (ASCII)
    0xC4, 0xEA  // 年
};

// 正在初始化MODBUS通信
static const uint8_t modbus_init_msg[] = {
    0xD5, 0xFD, // 正
    0xD4, 0xDA, // 在
    0xB3, 0xF5, // 初
    0xCA, 0xBC, // 始
    0x4D, 0x4F, // MO
    0x44, 0x42, // DB
    0x55, 0x53, // US
    0xCD, 0xA8, // 通
    0xD0, 0xC5  // 信
};
// 正在初始化MODBUS通信
static const uint8_t motor_init_msg[] = {
    // 0xD5, 0xFD,      // 正
    // 0xD4, 0xDA,      // 在
    // 0xB3, 0xF5,      // 初
    // 0xCA, 0xBC,      // 始
    // 0xB5, 0xE7,      // 电
    // 0xBB, 0xFA,      // 机
    // 0xA3, 0xAC,      // ， (GBK全角逗号)
    0xB5, 0xE7, // 电
    0xBB, 0xFA, // 机
    0xBB, 0xD8, // 回
    0xC1, 0xE3, // 零
    0xD6, 0xD0  // 中
    // 0x2E, 0x2E, 0x2E // ...
};
// 总数
static const uint8_t tot_cnt_txt[] = {
    0xD7, 0xDC, // 总
    0xCA, 0xFD  // 数
    // 0x3A        // ：
};

// 当前
static const uint8_t cur_cnt_txt[] = {
    0xB5, 0xB1, // 当
    0xC7, 0xB0  // 前
    // 0x3A        // ：
};

// 状态
static const uint8_t sta_txt[] = {
    0xD7, 0xB4, // 状
    0xCC, 0xAC  // 态
    // 0x3A        // ：
};
// 模式
static const uint8_t mode_txt[] = {
    0xC4, 0xA3, // 模
    0xCA, 0xBD  // 式
    // 0x3A        // ：
};

// 空闲
static const uint8_t kongxian[] = {
    0xBF, 0xD5, // 空
    0xCF, 0xD0  // 闲
};

// 上烟
static const uint8_t shangyan[] = {
    0xC9, 0xCF, // 上
    0xD1, 0xCC  // 烟
};

// 下烟
static const uint8_t xiayan[] = {
    0xCF, 0xC2, // 下
    0xD1, 0xCC  // 烟
};

// 暂停
static const uint8_t pause[] = {
    0xD4, 0xDD, // 暂
    0xCD, 0xA3  // 停
};
// 工作中
static const uint8_t running[] = {
    0xB9, 0xA4, // 工
    0xD7, 0xF7, // 作
    0xD6, 0xD0  // 中
};

// 感应器状态
static const uint8_t const_sensor_state[] = {
    0xB8, 0xD0, // 感
    0xD3, 0xA6, // 应
    0xC6, 0xF7, // 器
    0xD7, 0xB4, // 状
    0xCC, 0xAC  // 态
};

// 上限位
static const uint8_t const_sensor_up[] = {
    0xC9, 0xCF, // 上
    0xCF, 0xDE, // 限
    0xCE, 0xBB  // 位
    // 0x3A        // ：
};

// 下限位
static const uint8_t const_sensor_dn[] = {
    0xCF, 0xC2, // 下
    0xCF, 0xDE, // 限
    0xCE, 0xBB  // 位
    // 0x3A        // ：
};

// 负载
static const uint8_t const_sensor_ld[] = {
    0xB8, 0xBA, // 负
    0xD4, 0xD8  // 载
    // 0x3A        // ：
};

// 转速
static const uint8_t const_rpm[] = {
    0xd7, 0xaa, // 转
    0xcb, 0xd9  // 速
    // 0x3A        // ：
};
// 位置
static const uint8_t const_pos[] = {
    0xce, 0xbb, // 位
    0xd6, 0xc3  // 置
    // 0x3A        // ：
};
// 电流
static const uint8_t const_cur[] = {
    0xb5, 0xe7, // 电
    0xc1, 0xf7, // 流
    0x3A        // ：
};
// 负载率
static const uint8_t const_load_rate[] = {
    // 0xb8, 0xba, // 负
    0xd4, 0xd8, // 载
    0xc2, 0xca  // 率
    // 0x3A        // ：
};

// 转矩
static const uint8_t const_load[] = {
    0xb8, 0xba, // 负
    0xd4, 0xd8, // 载
    0x3A        // ：
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

typedef enum {
    SN_STATE_DISCONNECTED = 0,
    SN_STATE_CONNECTING,
    SN_STATE_CONNECTED,
    SN_STATE_ERROR
} SN_State_t;

typedef struct {
    UART_HandleTypeDef *huart;
    SN_State_t state;
} Screen_t;

/* 调试开关：定义为1 将打印发送/接收帧（使用 printf）*/
#ifndef SCREEN_DEBUG
#define SCREEN_DEBUG 1
#endif

// #ifndef SCREEN_TYPE
// #define SCREEN_TYPE 1 /* 1: DGUS 2: TSUIC1*/
// #endif

uint8_t screen_init(Screen_t *screen, UART_HandleTypeDef *huart);
void screen_receive_callback(Screen_t *screen);
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
SN_Status_t screen_write_text(Screen_t *screen, uint16_t var_addr, const char *text, uint8_t clear_bg);
/**
 * @brief  写入数据
 * @param  screen: screen句柄
 * @param  var_addr: 变量地址
 * @param  bytes: 字节数组
 * @param  len: 字节长度
 * @retval 状态
 */
SN_Status_t screen_write_str_bytes(Screen_t *screen, uint16_t var_addr, const uint8_t *bytes, uint8_t len, uint8_t clear_bg);
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
