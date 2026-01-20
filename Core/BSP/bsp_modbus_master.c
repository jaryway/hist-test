#include "bsp_modbus_master.h"
#include <stdio.h>
#include <string.h>

/* 可调项：发送后等待 TC 的最大等待（ms） */
#ifndef MODBUS_TC_WAIT_MS
#define MODBUS_TC_WAIT_MS 100
#endif

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

/* 发送封装：确保在任何情况下都关闭 DE
 * timeout_ms 用于 HAL_UART_Transmit 的超时（可为传入的 timeout_ms）
 */
static MB_Status_t _modbus_send(uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    if (!mb_huart) return MB_ERR_HW;

    _rs485_de_enable();
    if (HAL_UART_Transmit(mb_huart, data, len, timeout_ms) != HAL_OK) {
        _rs485_de_disable();
        return MB_ERR_HW;
    }

    if (_wait_for_tc(MODBUS_TC_WAIT_MS) != HAL_OK) {
        _rs485_de_disable();
        return MB_ERR_HW;
    }
    _rs485_de_disable();
    return MB_OK;
}

/* 接收封装：根据整体请求开始时间和 timeout_ms 计算剩余超时并执行 HAL_UART_Receive */
static MB_Status_t _recv_remaining(uint8_t *buf, uint16_t len, uint32_t t_start, uint32_t timeout_ms)
{
    if (!mb_huart) return MB_ERR_HW;
    uint32_t elapsed = HAL_GetTick() - t_start;
    if (elapsed >= timeout_ms) return MB_ERR_TIMEOUT;
    uint32_t remaining = timeout_ms - elapsed;
    if (HAL_UART_Receive(mb_huart, buf, len, remaining) != HAL_OK) {
        return MB_ERR_TIMEOUT;
    }
    return MB_OK;
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
 * @brief  03功能码：Modbus读取保持寄存器功能码（分段接收，整体超时）
 */
MB_Status_t modbus_read_holding_registers(uint8_t slave, uint16_t addr, uint16_t quantity, uint16_t *dest, uint32_t timeout_ms)
{
    if (!mb_huart || !dest) return MB_ERR_PARAM;
    if (quantity == 0 || quantity > 125) return MB_ERR_PARAM; // Modbus 单次最多125寄存器

    uint8_t tx_data[8];
    uint16_t crc;

    // 1.构造请求帧
    tx_data[0] = slave;                  // 从站地址
    tx_data[1] = 0x03;                   // 功能码
    tx_data[2] = (addr >> 8) & 0xFF;     // 起始地址高 8 位
    tx_data[3] = addr & 0xFF;            // 起始地址低 8 位
    tx_data[4] = (quantity >> 8) & 0xFF; // 寄存器数量高8位
    tx_data[5] = quantity & 0xFF;        // 寄存器数量低8位

    // 2.计算CRC
    crc        = _modbus_crc16(tx_data, 6);
    tx_data[6] = crc & 0xFF;        // CRC低字节
    tx_data[7] = (crc >> 8) & 0xFF; // CRC高字节

#if MODBUS_DEBUG
    printf("TX(03): ");
    for (int i = 0; i < 8; i++) printf("%02X ", tx_data[i]);
    printf("\n");
#endif

    // 3.发送请求（使用 timeout_ms 作为发送超时）
    if (_modbus_send(tx_data, 8, timeout_ms) != MB_OK) return MB_ERR_HW;

    // 下面使用分段接收：先接收 Slave+Func（2字节），再按异常/正常读取剩余
    uint8_t resp[MODBUS_MAX_ADU_LEN];
    memset(resp, 0, sizeof(resp));

    uint32_t t_start = HAL_GetTick();
    MB_Status_t r    = _recv_remaining(resp, 2, t_start, timeout_ms);
    if (r != MB_OK) return r;

    // 可选：立即检测从站地址不匹配（不过仍按帧读取以保持总线上同步）
    // if (resp[0] != slave) return MB_ERR_BAD_RESPONSE;

    uint8_t func       = resp[1];
    uint16_t total_len = 0;

    if (func & 0x80) {
        // 异常应答：剩余 ExceptionCode(1) + CRC(2)
        r = _recv_remaining(resp + 2, 3, t_start, timeout_ms);
        if (r != MB_OK) return r;
        total_len = 5;
    } else {
        // 正常应答：接收 ByteCount (1)
        r = _recv_remaining(resp + 2, 1, t_start, timeout_ms);
        if (r != MB_OK) return r;
        uint8_t byte_count = resp[2];
        // 校验长度合理性
        if (byte_count != 2 * quantity) return MB_ERR_BAD_RESPONSE;
        // 再接收 data + CRC
        uint16_t need = (uint16_t)byte_count + 2;
        if (need + 3 > MODBUS_MAX_ADU_LEN) return MB_ERR_BAD_RESPONSE;
        r = _recv_remaining(resp + 3, need, t_start, timeout_ms);
        if (r != MB_OK) return r;
        total_len = 3 + need;
    }

#if MODBUS_DEBUG
    printf("RX(03): ");
    for (int i = 0; i < total_len; i++) printf("%02X ", resp[i]);
    printf("\n");
#endif

    // CRC 校验
    if (total_len < 5) return MB_ERR_BAD_RESPONSE;
    uint16_t resp_crc = (uint16_t)resp[total_len - 2] | ((uint16_t)resp[total_len - 1] << 8);
    uint16_t calc_crc = _modbus_crc16(resp, total_len - 2);
    if (resp_crc != calc_crc) return MB_ERR_CRC;

    if (resp[1] & 0x80) {
        // 从站返回异常码在 resp[2]
        return MB_ERR_EXCEPTION;
    }

    if (resp[1] != 0x03) return MB_ERR_BAD_RESPONSE;
    uint8_t byte_count = resp[2];
    if (byte_count != 2 * quantity) return MB_ERR_BAD_RESPONSE;

    // 解析数据（大端）到 dest
    for (uint16_t i = 0; i < quantity; i++) {
        uint16_t hi = resp[3 + 2 * i];
        uint16_t lo = resp[3 + 2 * i + 1];
        dest[i]     = (uint16_t)((hi << 8) | lo);
    }

    return MB_OK;
}

/**
 * @brief  06功能码：写单路保持寄存器（分段接收）
 * @param  slave 从站地址
 * @param  addr 从站地址
 * @param  value 待写入的值
 * @param  timeout_ms 超时时间（ms）
 */
MB_Status_t modbus_write_single_register(uint8_t slave, uint16_t addr, uint16_t value, uint32_t timeout_ms)
{
    if (!mb_huart) return MB_ERR_PARAM;

    uint8_t tx_data[8];
    uint16_t crc;

    // 构造请求帧
    tx_data[0] = slave;
    tx_data[1] = 0x06;
    tx_data[2] = (addr >> 8) & 0xFF;
    tx_data[3] = addr & 0xFF;
    tx_data[4] = (value >> 8) & 0xFF; // 寄存器高字节
    tx_data[5] = value & 0xFF;        // 寄存器高字节
    crc        = _modbus_crc16(tx_data, 6);
    tx_data[6] = crc & 0xFF;
    tx_data[7] = (crc >> 8) & 0xFF;

#if MODBUS_DEBUG
    printf("TX(06): ");
    for (int i = 0; i < 8; i++) printf("%02X ", tx_data[i]);
    printf("\n");
#endif

    if (_modbus_send(tx_data, 8, timeout_ms) != MB_OK) return MB_ERR_HW;

    uint8_t resp[MODBUS_MAX_ADU_LEN];
    memset(resp, 0, sizeof(resp));
    uint32_t t_start = HAL_GetTick();
    MB_Status_t r    = _recv_remaining(resp, 2, t_start, timeout_ms);
    if (r != MB_OK) return r;

    uint8_t func       = resp[1];
    uint16_t total_len = 0;

    if (func & 0x80) {
        r = _recv_remaining(resp + 2, 3, t_start, timeout_ms);
        if (r != MB_OK) return r;
        total_len = 5;
    } else {
        // 完整回显为 8 字节
        r = _recv_remaining(resp + 2, 6, t_start, timeout_ms);
        if (r != MB_OK) return r;
        total_len = 8;
    }

#if MODBUS_DEBUG
    printf("RX(06): ");
    for (int i = 0; i < total_len; i++) printf("%02X ", resp[i]);
    printf("\n");
#endif

    if (total_len < 5) return MB_ERR_BAD_RESPONSE;
    uint16_t resp_crc = (uint16_t)resp[total_len - 2] | ((uint16_t)resp[total_len - 1] << 8);
    uint16_t calc_crc = _modbus_crc16(resp, total_len - 2);
    if (resp_crc != calc_crc) return MB_ERR_CRC;

    if (resp[1] & 0x80) return MB_ERR_EXCEPTION;

    if (resp[1] != 0x06) return MB_ERR_BAD_RESPONSE;
    uint16_t raddr = (uint16_t)resp[2] << 8 | resp[3];
    uint16_t rval  = (uint16_t)resp[4] << 8 | resp[5];
    if (raddr != addr || rval != value) return MB_ERR_BAD_RESPONSE;

    return MB_OK;
}

/**
 * @brief  16功能码：写多个保持寄存器（分段接收）
 * @param slave 从站地址
 * @param addr 寄存器起始地址
 * @param quantity 寄存器数量
 * @param values 待写入的寄存器值
 * @param timeout_ms 超时时间（ms）
 */
MB_Status_t modbus_write_multiple_registers(uint8_t slave, uint16_t addr, uint16_t quantity, const uint16_t *values, uint32_t timeout_ms)
{
    if (!mb_huart || !values) return MB_ERR_PARAM;
    if (quantity == 0 || quantity > 123) return MB_ERR_PARAM; // 常见限制：最多123寄存器

    /* 构造请求：长度检查 */
    uint16_t data_bytes = quantity * 2;
    uint16_t req_len    = 1 + 1 + 2 + 2 + 1 + data_bytes + 2; // Slave + Func + Addr(2) + Qty(2) + ByteCount + Data + CRC
    if (req_len > MODBUS_MAX_ADU_LEN) return MB_ERR_PARAM;

    uint8_t tx_data[MODBUS_MAX_ADU_LEN];
    uint16_t idx   = 0;
    tx_data[idx++] = slave;
    tx_data[idx++] = 0x10;
    tx_data[idx++] = (addr >> 8) & 0xFF;     // 寄存器起始地址高 8 位
    tx_data[idx++] = addr & 0xFF;            // 寄存器起始地址低 8 位
    tx_data[idx++] = (quantity >> 8) & 0xFF; // 寄存器个数高 8 位
    tx_data[idx++] = quantity & 0xFF;        // 寄存器个数低 8 位
    tx_data[idx++] = (uint8_t)data_bytes;    // 数据字节数=寄存器个数的 2 倍

    for (uint16_t i = 0; i < quantity; i++) {
        tx_data[idx++] = (values[i] >> 8) & 0xFF; // 第 n 个数据高8位
        tx_data[idx++] = values[i] & 0xFF;        // 第 n 个数据低8位
    }

    uint16_t crc   = _modbus_crc16(tx_data, idx);
    tx_data[idx++] = crc & 0xFF;
    tx_data[idx++] = (crc >> 8) & 0xFF;

#if MODBUS_DEBUG
    printf("TX(10): ");
    for (int i = 0; i < idx; i++) printf("%02X ", tx_data[i]);
    printf("\n");
#endif

    if (_modbus_send(tx_data, idx, timeout_ms) != MB_OK) return MB_ERR_HW;

    // 正常应答长度固定为 8（Slave(1)+Func(1)+Addr(2)+Qty(2)+CRC(2)）
    uint8_t resp[MODBUS_MAX_ADU_LEN];
    memset(resp, 0, sizeof(resp));
    uint32_t t_start = HAL_GetTick();
    MB_Status_t r    = _recv_remaining(resp, 2, t_start, timeout_ms);
    if (r != MB_OK) return r;

    uint8_t func       = resp[1];
    uint16_t total_len = 0;
    if (func & 0x80) {
        r = _recv_remaining(resp + 2, 3, t_start, timeout_ms);
        if (r != MB_OK) return r;
        total_len = 5;
    } else {
        r = _recv_remaining(resp + 2, 6, t_start, timeout_ms);
        if (r != MB_OK) return r;
        total_len = 8;
    }

#if MODBUS_DEBUG
    printf("RX(10): ");
    for (int i = 0; i < total_len; i++) printf("%02X ", resp[i]);
    printf("\n");
#endif

    if (total_len < 5) return MB_ERR_BAD_RESPONSE;
    uint16_t resp_crc = (uint16_t)resp[total_len - 2] | ((uint16_t)resp[total_len - 1] << 8);
    uint16_t calc_crc = _modbus_crc16(resp, total_len - 2);
    if (resp_crc != calc_crc) return MB_ERR_CRC;

    if (resp[1] & 0x80) return MB_ERR_EXCEPTION;

    if (resp[1] != 0x10) return MB_ERR_BAD_RESPONSE;
    uint16_t raddr = (uint16_t)resp[2] << 8 | resp[3];
    uint16_t rqty  = (uint16_t)resp[4] << 8 | resp[5];
    if (raddr != addr || rqty != quantity) return MB_ERR_BAD_RESPONSE;

    return MB_OK;
}