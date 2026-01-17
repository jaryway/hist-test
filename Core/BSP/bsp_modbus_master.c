#include "bsp_modbus_master.h"
#include <stdio.h>
#include <string.h>

static UART_HandleTypeDef *mb_huart = NULL;
static GPIO_TypeDef *mb_de_port     = NULL;
static uint16_t mb_de_pin           = 0;

/* CRC16 (Modbus) */
static uint16_t _modbus_crc16(const uint8_t *buf, uint16_t len)
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
static inline void _rs485_de_enable(void)
{
    if (mb_de_port) HAL_GPIO_WritePin(mb_de_port, mb_de_pin, GPIO_PIN_SET);
}
static inline void _rs485_de_disable(void)
{
    if (mb_de_port) HAL_GPIO_WritePin(mb_de_port, mb_de_pin, GPIO_PIN_RESET);
}

/* Helper: wait for UART TC (transmission complete) with timeout */
static HAL_StatusTypeDef _wait_for_tc(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while (__HAL_UART_GET_FLAG(mb_huart, UART_FLAG_TC) == RESET) {
        if ((HAL_GetTick() - start) > timeout_ms) return HAL_TIMEOUT;
    }
    return HAL_OK;
}

static void _modbus_send(uint8_t *data, uint16_t len)
{
    // 发送：启用 DE -> 发送 -> 等待 TC -> 禁用 DE
    _rs485_de_enable();
    if (HAL_UART_Transmit(mb_huart, data, len, HAL_MAX_DELAY) != HAL_OK) {
        _rs485_de_disable();
        // return MB_ERR_HW;
        return;
    }

    if (_wait_for_tc(100) != HAL_OK) { // 等待最多 100ms，按需调整
        _rs485_de_disable();
        // return MB_ERR_HW;
        return;
    }
    _rs485_de_disable();
}

void modbus_init(UART_HandleTypeDef *huart_ptr, GPIO_TypeDef *de_gpio_port, uint16_t de_pin)
{
    mb_huart   = huart_ptr;
    mb_de_port = de_gpio_port;
    mb_de_pin  = de_pin;
    // 确保 DE 初始为接收（低）
    _rs485_de_disable();
}

/**
 * @brief  03功能码：Modbus读取保持寄存器功能码
 * @param  slave 从站地址
 * @param  addr 起始地址
 * @param  quantity   寄存器数量
 * @param  dest   用户缓冲区
 * @param  timeout_ms 超时时间
 * @retval None
 */
MB_Status_t modbus_read_holding_registers(uint8_t slave, uint16_t addr, uint16_t quantity, uint16_t *dest, uint32_t timeout_ms)
{
    uint8_t tx_data[8];
    uint16_t crc;
    // 将user_buf指针地址给到current_buffer，修改current_buffer实质就是修改user_buf
    // uint8_t *current_buffer = user_buf;

    // 1.构造请求帧
    tx_data[0] = slave;                  // 从站地址
    tx_data[1] = 0x03;                   // 功能码
    tx_data[2] = (addr >> 8) & 0xFF;     // 起始地址高字节
    tx_data[3] = addr & 0xFF;            // 起始地址低字节
    tx_data[4] = (quantity >> 8) & 0xFF; // 寄存器数量高字节
    tx_data[5] = quantity & 0xFF;        // 寄存器数量低字节

    // 2.计算CRC
    crc        = _modbus_crc16(tx_data, 6);
    tx_data[6] = crc & 0xFF;        // CRC低字节
    tx_data[7] = (crc >> 8) & 0xFF; // CRC高字节

    _modbus_send(tx_data, 8);
    // 3.发送请求
    printf("Sending 03");
    // ，打印发送的数据帧
    for (int i = 0; i < 8; i++) {
        printf("%02X", tx_data[i]);
    }

    printf("\n");

    // 预期应答长度计算：Slave(1) + Func(1) + ByteCount(1) + Data(2*quantity) + CRC(2)
    uint16_t expected_len = 5 + 2 * quantity;
    if (expected_len > MODBUS_MAX_ADU_LEN) return MB_ERR_BAD_RESPONSE;

    // 接收阻塞等待 expected_len 字节（timeout_ms 为整体超时）
    uint8_t resp[MODBUS_MAX_ADU_LEN];
    memset(resp, 0, expected_len);

    // 使用 HAL_UART_Receive 阻塞接收 expected_len，超时为 timeout_ms（可改进为分段接收）
    if (HAL_UART_Receive(mb_huart, resp, expected_len, timeout_ms) != HAL_OK) {
        return MB_ERR_TIMEOUT;
    }

    // CRC 校验
    uint16_t resp_crc = (uint16_t)resp[expected_len - 2] | ((uint16_t)resp[expected_len - 1] << 8);
    uint16_t calc_crc = _modbus_crc16(resp, expected_len - 2);
    if (resp_crc != calc_crc) {
        return MB_ERR_CRC;
    }

    // 校验地址和功能码
    if (resp[0] != slave) return MB_ERR_BAD_RESPONSE;
    if (resp[1] == (0x83)) { // 0x03 | 0x80 = 0x83 表示异常应答
        // 异常码在 resp[2]
        return MB_ERR_EXCEPTION;
    }
    if (resp[1] != 0x03) return MB_ERR_BAD_RESPONSE;

    uint8_t byte_count = resp[2];
    if (byte_count != 2 * quantity) return MB_ERR_BAD_RESPONSE;

    // 解析数据（大端寄存器）
    for (uint16_t i = 0; i < quantity; i++) {
        uint16_t hi = resp[3 + 2 * i];
        uint16_t lo = resp[3 + 2 * i + 1];
        dest[i]     = (uint16_t)((hi << 8) | lo);
    }

    return MB_OK;
}

/**
 * @brief  06功能码：写单路保持寄存器
 * @param  slave      从站地址
 * @param  addr       寄存器地址
 * @param  value      寄存器值
 * @retval None
 */
MB_Status_t modbus_write_single_register(uint8_t slave, uint16_t addr, uint16_t value, uint32_t timeout_ms)
{
    uint8_t tx_data[8];
    uint16_t crc;

    // 1.构造请求帧
    tx_data[0] = slave;              // 从站地址
    tx_data[1] = 0x06;               // 功能码
    tx_data[2] = (addr >> 8) & 0xFF; // 寄存器地址高字节
    tx_data[3] = addr & 0xFF;
    tx_data[4] = (value >> 8) & 0xFF;
    tx_data[5] = value & 0xFF;

    crc        = _modbus_crc16(tx_data, 6);
    tx_data[6] = crc & 0xFF;
    tx_data[7] = (crc >> 8) & 0xFF;

    _modbus_send(tx_data, 8);

    // 预期应答长度计算：Slave(1) + Func(1) + Addr(2) + Data(2) + CRC(2)
    uint16_t expected_len = 8;
    uint8_t resp[expected_len];

    memset(resp, 0, expected_len); // 将缓冲区所有字节设为0

    if (HAL_UART_Receive(mb_huart, resp, expected_len, timeout_ms) != HAL_OK) {
        return MB_ERR_TIMEOUT;
    }

    // CRC 校验
    uint16_t resp_crc = (uint16_t)resp[expected_len - 2] | ((uint16_t)resp[expected_len - 1] << 8);
    uint16_t calc_crc = _modbus_crc16(resp, expected_len - 2);
    if (resp_crc != calc_crc) return MB_ERR_CRC;

    if (resp[1] & 0x80) return MB_ERR_EXCEPTION;

    // 验证回显地址和值是否和请求一致
    if (resp[0] != slave) return MB_ERR_BAD_RESPONSE;
    if (resp[1] != 0x06) return MB_ERR_BAD_RESPONSE;

    uint16_t raddr = (uint16_t)resp[2] << 8 | resp[3];
    uint16_t rval  = (uint16_t)resp[4] << 8 | resp[5];
    if (raddr != addr || rval != value) return MB_ERR_BAD_RESPONSE;
    return MB_OK;
}

/**
 * @brief  16功能码：写多个保持寄存器
 * @param  slave 从站地址
 * @param  addr 起始地址
 * @param  quantity   寄存器数量
 * @param  values     寄存器值
 * @retval None
 */
MB_Status_t modbus_write_multiple_registers(uint8_t slave, uint16_t addr, uint16_t quantity, uint16_t *values, uint32_t timeout_ms)
{
    uint8_t tx_data[9 + 2 * quantity];
    uint16_t crc;

    // 1.构造请求帧
    tx_data[0] = slave;                  // 从站地址
    tx_data[1] = 0x10;                   // 功能码
    tx_data[2] = (addr >> 8) & 0xFF;     // 起始地址高字节
    tx_data[3] = addr & 0xFF;            // 起始地址低字节
    tx_data[4] = (quantity >> 8) & 0xFF; // 寄存器数量高字节
    tx_data[5] = quantity & 0xFF;        // 寄存器数量低字节
    tx_data[6] = quantity * 2;           // 字节数每个寄存器占2字节

    for (size_t i = 0; i < quantity; i++) {
        tx_data[7 + 2 * i] = (values[i] >> 8) & 0xFF; // 寄存器值高字节
        tx_data[8 + 2 * i] = values[i] & 0xFF;        // 寄存器值低字节
    }

    crc = _modbus_crc16(tx_data, 7 + 2 * quantity);

    tx_data[7 + 2 * quantity] = crc & 0xFF;
    tx_data[8 + 2 * quantity] = (crc >> 8) & 0xFF;

    _modbus_send(tx_data, 9 + 2 * quantity);

    // 预期应答长度计算：Slave(1) + Func(1) + Addr(2) + ByteCount(2) + CRC(2)
    // 接收响应（8字节）
    uint8_t resp[8];
    if (HAL_UART_Receive(mb_huart, resp, 8, timeout_ms) != HAL_OK) {
        // printf("Timeout waiting for write multiple registers response\n");
        return MB_ERR_TIMEOUT;
    }

    // 打印接收到的响应数据
    // printf("Write multiple registers response: ");
    // for (int i = 0; i < 8; i++) {
    //     printf("%02X ", resp[i]);
    // }
    // printf("\n");

    // CRC校验
    uint16_t resp_crc = (uint16_t)resp[6] | ((uint16_t)resp[7] << 8);
    uint16_t calc_crc = _modbus_crc16(resp, 6);
    if (resp_crc != calc_crc) {
        // printf("CRC check failed for write multiple registers\n");
        return MB_ERR_CRC;
    }

    // 检查异常响应
    if (resp[1] & 0x80) {
        // printf("Exception response received: 0x%02X\n", resp[2]);
        return MB_ERR_EXCEPTION;
    }

    // 验证响应内容
    if (resp[0] != slave) {
        // printf("Slave address mismatch in response\n");
        return MB_ERR_BAD_RESPONSE;
    }

    if (resp[1] != 0x10) {
        // printf("Function code mismatch in response\n");
        return MB_ERR_BAD_RESPONSE;
    }

    // 验证起始地址和数量是否与请求一致
    uint16_t resp_addr = (uint16_t)resp[2] << 8 | resp[3];
    uint16_t resp_qty  = (uint16_t)resp[4] << 8 | resp[5];

    if (resp_addr != addr || resp_qty != quantity) {
        // printf("Address or quantity mismatch in response\n");
        return MB_ERR_BAD_RESPONSE;
    }

    // printf("Write multiple registers successful: slave %d, start addr 0x%04X, qty %d\n",
    //        slave, addr, quantity);

    return MB_OK;
}