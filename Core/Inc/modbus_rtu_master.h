#ifndef _MODBUS_RTU_MASTER_H
#define _MODBUS_RTU_MASTER_H

#include "stm32f1xx_hal.h"

typedef struct
{
    uint8_t id;
    uint8_t function;
    uint16_t address;
    uint16_t length;
} modbus_rtu_request_t;

void ModbusInit(void);
void Modbus_Handle_Rx(uint8_t *data, uint8_t len);
// 03功能码：读单路保持寄存器
void Modbus_Read_03(uint8_t slave_addr, uint16_t start_addr, uint16_t num_regs, uint8_t *user_buf);
// 06功能码：写单路保持寄存器
void Modbus_Write_06(uint8_t slave_addr, uint16_t reg_addr, uint16_t value);
// 16功能码：写多路保持寄存器
void Modbus_Write_16(uint8_t slave_addr, uint16_t start_addr, uint16_t num_regs, uint16_t *values);

#endif