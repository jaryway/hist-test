#ifndef _BSP_MODBUS_MASTER_H
#define _BSP_MODBUS_MASTER_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

#define MODBUS_MAX_ADU_LEN 256

typedef enum {
    MB_OK = 0,
    MB_ERR_TIMEOUT,
    MB_ERR_CRC,
    MB_ERR_EXCEPTION, // 从站返回异常码
    MB_ERR_BAD_RESPONSE,
    MB_ERR_HW,
    MB_ERR_PARAM,
    MB_ERR_BUSY
} MB_Status_t;

/* 调试开关：定义为1 将打印发送/接收帧（使用 printf）*/
#ifndef MODBUS_DEBUG
#define MODBUS_DEBUG 1
#endif


void modbus_init(UART_HandleTypeDef *huart_ptr, GPIO_TypeDef *de_gpio_port, uint16_t de_pin);

/* 03功能码：读保持寄存器
 * timeout_ms 为整体请求超时（包含发送/接收）
 */
MB_Status_t modbus_read_holding_registers(uint8_t slave, uint16_t addr, uint16_t quantity, uint16_t *dest, uint32_t timeout_ms);

/* 06功能码：写单路保持寄存器 */
MB_Status_t modbus_write_single_register(uint8_t slave, uint16_t addr, uint16_t value, uint32_t timeout_ms);

/* 16功能码：写多个保持寄存器
 * values 指向 quantity 个 uint16_t 数据
 */
MB_Status_t modbus_write_multiple_registers(uint8_t slave, uint16_t addr, uint16_t quantity, const uint16_t *values, uint32_t timeout_ms);

#endif