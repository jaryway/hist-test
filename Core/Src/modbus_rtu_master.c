#include "modbus_rtu_master.h"
#include <stdio.h>

static UART_HandleTypeDef *mb_huart = NULL;
static GPIO_TypeDef *mb_de_port     = NULL;
static uint16_t mb_de_pin           = 0;

/* CRC16 (Modbus) */
uint16_t modbus_crc16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)buf[pos];
        for (int i = 0; i < 8; i++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/* 控制 DE（驱动使能） */
static inline void RS485_DE_Enable(void)
{
    if (mb_de_port) HAL_GPIO_WritePin(mb_de_port, mb_de_pin, GPIO_PIN_SET);
}
static inline void RS485_DE_Disable(void)
{
    if (mb_de_port) HAL_GPIO_WritePin(mb_de_port, mb_de_pin, GPIO_PIN_RESET);
}

/* Helper: wait for UART TC (transmission complete) with timeout */
static HAL_StatusTypeDef wait_for_tc(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while (__HAL_UART_GET_FLAG(mb_huart, UART_FLAG_TC) == RESET) {
        if ((HAL_GetTick() - start) > timeout_ms) return HAL_TIMEOUT;
    }
    return HAL_OK;
}

void Modbus_Send(uint8_t *data, uint16_t len)
{
    if (!mb_huart) {
        return;
    }

    // 发送：启用 DE -> 发送 -> 等待 TC -> 禁用 DE
    RS485_DE_Enable();
    if (HAL_UART_Transmit(mb_huart, data, len, HAL_MAX_DELAY) != HAL_OK) {
        RS485_DE_Disable();
        // return MB_ERR_HW;
        return;
    }

    if (wait_for_tc(100) != HAL_OK) { // 等待最多 100ms，按需调整
        RS485_DE_Disable();
        // return MB_ERR_HW;
        return;
    }
    RS485_DE_Disable();
}

void modbus_master_init(UART_HandleTypeDef *huart_ptr, GPIO_TypeDef *de_gpio_port, uint16_t de_pin)
{
    mb_huart   = huart_ptr;
    mb_de_port = de_gpio_port;
    mb_de_pin  = de_pin;
    // 确保 DE 初始为接收（低）
    RS485_DE_Disable();
}

/**
 * @brief  03功能码：Modbus读取保持寄存器功能码
 * @param  slave_addr 从站地址
 * @param  start_addr 起始地址
 * @param  num_regs   寄存器数量
 * @param  user_buf   用户缓冲区
 * @retval None
 */
void Modbus_Read_03(uint8_t slave_addr, uint16_t start_addr, uint16_t num_regs, uint8_t *user_buf)
{
    uint8_t tx_data[8];
    uint16_t crc;
    // 将user_buf指针地址给到current_buffer，修改current_buffer实质就是修改user_buf
    // uint8_t *current_buffer = user_buf;

    // 1.构造请求帧
    tx_data[0] = slave_addr;               // 从站地址
    tx_data[1] = 0x03;                     // 功能码
    tx_data[2] = (start_addr >> 8) & 0xFF; // 起始地址高字节
    tx_data[3] = start_addr & 0xFF;        // 起始地址低字节
    tx_data[4] = (num_regs >> 8) & 0xFF;   // 寄存器数量高字节
    tx_data[5] = num_regs & 0xFF;          // 寄存器数量低字节

    // 2.计算CRC
    crc        = modbus_crc16(tx_data, 6);
    tx_data[6] = crc & 0xFF;        // CRC低字节
    tx_data[7] = (crc >> 8) & 0xFF; // CRC高字节

    Modbus_Send(tx_data, 8);
    // 3.发送请求
    printf("Sending 03");
    // ，打印发送的数据帧
    for (int i = 0; 1 < 8; i++) {
        printf("%02X", tx_data[i]);
    }

    printf("\n");
}

/**
 * @brief  06功能码：写单路保持寄存器
 * @param  slave_addr 从站地址
 * @param  reg_addr   寄存器地址
 * @param  value      寄存器值
 * @retval None
 */
void Modbus_Write_06(uint8_t slave_addr, uint16_t reg_addr, uint16_t value)
{
    uint8_t tx_data[8];
    uint16_t crc;

    // 1.构造请求帧
    tx_data[0] = slave_addr;             // 从站地址
    tx_data[1] = 0x06;                   // 功能码
    tx_data[2] = (reg_addr >> 8) & 0xFF; // 寄存器地址高字节
    tx_data[3] = reg_addr & 0xFF;
    tx_data[4] = (value >> 8) & 0xFF;
    tx_data[5] = value & 0xFF;

    crc        = modbus_crc16(tx_data, 6);
    tx_data[6] = crc & 0xFF;
    tx_data[7] = (crc >> 8) & 0xFF;

    Modbus_Send(tx_data, 8);
}

/**
 * @brief  16功能码：写多个保持寄存器
 * @param  slave_addr 从站地址
 * @param  start_addr 起始地址
 * @param  num_regs   寄存器数量
 * @param  values     寄存器值
 * @retval None
 */
void Modbus_Write_16(uint8_t slave_addr, uint16_t start_addr, uint16_t num_regs, uint16_t *values)
{
    uint8_t tx_data[9 + 2 * num_regs];
    uint16_t crc;

    // 1.构造请求帧
    tx_data[0] = slave_addr;               // 从站地址
    tx_data[1] = 0x10;                     // 功能码
    tx_data[2] = (start_addr >> 8) & 0xFF; // 起始地址高字节
    tx_data[3] = start_addr & 0xFF;        // 起始地址低字节
    tx_data[4] = (num_regs >> 8) & 0xFF;   // 寄存器数量高字节
    tx_data[5] = num_regs & 0xFF;          // 寄存器数量低字节
    tx_data[6] = num_regs * 2;             // 字节数每个寄存器占2字节

    for (size_t i = 0; i < num_regs; i++) {
        tx_data[7 + 2 * i] = (values[i] >> 8) & 0xFF; // 寄存器值高字节
        tx_data[8 + 2 * i] = values[i] & 0xFF;        // 寄存器值低字节
    }

    crc = modbus_crc16(tx_data, 7 + 2 * num_regs);

    tx_data[7 + 2 * num_regs] = crc & 0xFF;
    tx_data[8 + 2 * num_regs] = (crc >> 8) & 0xFF;

    Modbus_Send(tx_data, 9 + 2 * num_regs);
}