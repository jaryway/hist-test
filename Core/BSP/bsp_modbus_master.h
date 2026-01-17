#ifndef _BSP_MODBUS_MASTER_H
#define _BSP_MODBUS_MASTER_H

#include "stm32f1xx_hal.h"

#define MODBUS_MAX_ADU_LEN 256

typedef struct
{
    uint8_t id;
    uint8_t function;
    uint16_t address;
    uint16_t length;
} modbus_rtu_request_t;

typedef enum {
    MB_OK = 0,
    MB_ERR_TIMEOUT,
    MB_ERR_CRC,
    MB_ERR_EXCEPTION, // 从站返回异常码
    MB_ERR_BAD_RESPONSE,
    MB_ERR_HW
} MB_Status_t;

void modbus_init(UART_HandleTypeDef *huart_ptr, GPIO_TypeDef *de_gpio_port, uint16_t de_pin);
// void Modbus_Handle_Rx(uint8_t *data, uint8_t len);
// 03功能码：读单路保持寄存器
MB_Status_t modbus_read_holding_registers(uint8_t slave, uint16_t addr, uint16_t quantity, uint16_t *dest, uint32_t timeout_ms);
// 06功能码：写单路保持寄存器
MB_Status_t modbus_write_single_register(uint8_t slave, uint16_t addr, uint16_t value, uint32_t timeout_ms);
// 16功能码：写多路保持寄存器
MB_Status_t modbus_write_multiple_registers(uint8_t slave, uint16_t addr, uint16_t quantity, uint16_t *values,uint32_t timeout_ms);

#endif