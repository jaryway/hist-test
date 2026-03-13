#ifndef _BSP_DWIN_DGUS_H
#define _BSP_DWIN_DGUS_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

#define DWIN_DGUS_MAX_DATA_LEN 256

typedef enum {
    DD_OK = 0,
    DD_ERR_TIMEOUT,
    DD_ERR_CRC,
    DD_ERR_EXCEPTION,
    DD_ERR_BAD_RESPONSE,
    DD_ERR_HW,
    DD_ERR_PARAM
} DD_Status_t;

/* 调试开关：定义为1 将打印发送/接收帧（使用 printf）*/
#ifndef DWIN_DGUS_DEBUG
#define DWIN_DGUS_DEBUG 1
#endif

void bsp_dwin_dgus_init(UART_HandleTypeDef *huart);
/* 0x80功能码：从指定地址开始写数据串到寄存器。 */
DD_Status_t bsp_dwin_dgus_write_regs(uint8_t page_addr, uint8_t reg_addr, uint16_t *data, uint16_t data_len, uint8_t need_crc, uint32_t timeout_ms);

/* 0x81功能码：从指定寄存器开始读数据。 */
DD_Status_t bsp_dwin_dgus_read_regs(uint8_t page_addr, uint8_t reg_addr, uint8_t read_len, uint16_t *dest, uint8_t need_crc, uint32_t timeout_ms);

/* 0x82功能码：从指定地址开始写数据串(字数据)到变量空间。 */
DD_Status_t bsp_dwin_dgus_write_var_regs(uint16_t var_addr, uint16_t *data, uint16_t data_len, uint8_t need_crc, uint32_t timeout_ms);

/* 0x83功能码：从变量空间指定地址开始读指定长度字数据。  */
DD_Status_t bsp_dwin_dgus_read_var_regs(uint16_t var_addr, uint8_t read_len, uint16_t *dest, uint8_t need_crc, uint32_t timeout_ms);

#endif