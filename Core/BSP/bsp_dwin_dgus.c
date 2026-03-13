#include "bsp_dwin_dgus.h"
#include <stdio.h>
#include <string.h>
#include "debug.h"
#include "helper.h"

/* 可调项：发送后等待 TC 的最大等待（ms） */
#ifndef DWIN_DGUS_TC_WAIT_MS
#define DWIN_DGUS_TC_WAIT_MS 100
#endif

#define DWIN_DGUS_HEADER1 0x5A
#define DWIN_DGUS_HEADER2 0xA5

static UART_HandleTypeDef *dd_huart = NULL;

static DD_Status_t _check_crc16(const uint8_t *buf, uint16_t len)
{
    uint16_t resp_crc = (uint16_t)buf[len - 2] | ((uint16_t)buf[len - 1] << 8);
    uint16_t calc_crc = _calc_crc16(&buf[2], len - 2);
    if (resp_crc == calc_crc) return DD_OK;
    return DD_ERR_CRC;
}

static DD_Status_t _check_header_and_len(const uint8_t *resp, uint16_t read_len)
{
    // 验证帧头
    if (resp[0] != DWIN_DGUS_HEADER1 || resp[1] != DWIN_DGUS_HEADER2) {
        return DD_ERR_BAD_RESPONSE;
    }

    // 验证长度字段
    if (resp[2] != (1 + read_len)) { // 长度字段应该 = 指令(1) + 数据长度
        return DD_ERR_BAD_RESPONSE;
    }
    return DD_OK;
}

static DD_Status_t _cmd_send(uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    if (!dd_huart) return DD_ERR_HW;

    if (HAL_UART_Transmit(dd_huart, data, len, timeout_ms) != HAL_OK) {
        return DD_ERR_HW;
    }
    return DD_OK;
}

/* 接收封装：根据整体请求开始时间和 timeout_ms 计算剩余超时并执行 HAL_UART_Receive */
static DD_Status_t _cmd_recive(uint8_t *buf, uint16_t len, uint32_t t_start, uint32_t timeout_ms)
{
    if (!dd_huart) return DD_ERR_HW;
    uint32_t elapsed = HAL_GetTick() - t_start;
    if (elapsed >= timeout_ms) return DD_ERR_TIMEOUT;
    uint32_t remaining = timeout_ms - elapsed;
    if (HAL_UART_Receive(dd_huart, buf, len, remaining) != HAL_OK) {
        return DD_ERR_TIMEOUT;
    }
    return DD_OK;
}

void bsp_dwin_dgus_init(UART_HandleTypeDef *huart)
{
    dd_huart = huart;
}

/* 0x80功能码：从指定地址开始写数据串到寄存器。 */
DD_Status_t bsp_dwin_dgus_write_regs(uint8_t page_addr, uint8_t reg_addr, uint16_t *data, uint16_t data_len, uint8_t need_crc, uint32_t timeout_ms)
{
    DD_Status_t ret;
    uint16_t crc;
    uint8_t req[DWIN_DGUS_MAX_DATA_LEN];

    // 数据帧=帧头+数据长度+指令+数据
    // 数据长度=1(指令) + 1(页面) + 1(寄存器) + 写入数据的字节长度
    // 数据=页面+寄存器+写入的数据

    int idx    = 0;
    req[idx++] = DWIN_DGUS_HEADER1;
    req[idx++] = DWIN_DGUS_HEADER2;
    req[idx++] = 3 + data_len * 2; // 数据长度=1(指令) + 1(页面) + 1(寄存器) + 写入数据的字节长度
    req[idx++] = 0x80;             // 指令
    req[idx++] = page_addr;        // 寄存器页面
    req[idx++] = reg_addr;         // 寄存器地址

    /* 写入的数据 */
    for (uint16_t i = 0; i < data_len; i++) {
        req[idx++] = (data[i] >> 8) & 0xFF; // 第 n 个数据高8位
        req[idx++] = data[i] & 0xFF;        // 第 n 个数据低8位
    }

    if (need_crc) {
        crc        = _calc_crc16(&req[2], idx - 2);
        req[idx++] = crc & 0xFF;        // CRC低字节
        req[idx++] = (crc >> 8) & 0xFF; // CRC高字节
    }

#if DWIN_DGUS_DEBUG
    PRINT_DEBUG("TX(0x80): ");
    for (int i = 0; i < idx; i++) PRINT_DEBUG("%02X ", req[i]);
    PRINT_DEBUG("\n");
#endif

    ret = _cmd_send(req, idx, timeout_ms);

    return ret;
}

/* 0x81功能码：从指定寄存器开始读数据。 */
DD_Status_t bsp_dwin_dgus_read_regs(uint8_t page_addr, uint8_t reg_addr, uint8_t read_len, uint16_t *dest, uint8_t need_crc, uint32_t timeout_ms)
{
    // 数据帧=帧头+数据长度+指令+数据
    // 数据长度=1(指令) + 1(页面) + 1(寄存器) + 1(读取数据字节长度(0x01-0xFB))
    // 数据=页面+寄存器+读取的数据长度

    if (!dest || read_len == 0 || read_len > 0xFB || timeout_ms == 0) {
        return DD_ERR_PARAM;
    }

    DD_Status_t ret;
    uint16_t crc;
    uint8_t req[DWIN_DGUS_MAX_DATA_LEN];

    int idx    = 0;
    req[idx++] = DWIN_DGUS_HEADER1;
    req[idx++] = DWIN_DGUS_HEADER2;
    req[idx++] = 4;         // 数据长度
    req[idx++] = 0x81;      // 指令
    req[idx++] = page_addr; // 寄存器页面
    req[idx++] = reg_addr;  // 寄存器地址
    req[idx++] = read_len;  // 读取数据字节长度(0x01-0xFB)

    if (need_crc) {
        crc        = _calc_crc16(&req[2], idx - 2);
        req[idx++] = crc & 0xFF;        // CRC低字节
        req[idx++] = (crc >> 8) & 0xFF; // CRC高字节
    }

#if DWIN_DGUS_DEBUG
    PRINT_DEBUG("TX(0x81): ");
    for (int i = 0; i < idx; i++) PRINT_DEBUG("%02X ", req[i]);
    PRINT_DEBUG("\n");
#endif

    // 发送数据
    ret = _cmd_send(req, idx, timeout_ms);
    if (ret != DD_OK) return ret;

    // 接收数据
    uint8_t resp[DWIN_DGUS_MAX_DATA_LEN];
    memset(resp, 0, sizeof(resp));

    uint32_t t_start = HAL_GetTick();
    uint16_t len     = read_len + 4 + (need_crc ? 2 : 0);
    ret              = _cmd_recive(resp, len, t_start, timeout_ms);
    if (ret != DD_OK) return ret;

    ret = _check_header_and_len(resp, read_len);
    if (ret != DD_OK) return ret;

    if (need_crc) {
        ret = _check_crc16(resp, len);
        if (ret != DD_OK) return ret;
    }

    // 解析数据（大端）到 dest
    for (uint16_t i = 0; i < read_len; i++) {
        uint16_t hi = resp[4 + 2 * i];
        uint16_t lo = resp[4 + 2 * i + 1];
        dest[i]     = (uint16_t)((hi << 8) | lo);
    }

    return ret;
}

/* 0x82功能码：从指定地址开始写数据串(字数据)到变量空间。 */
DD_Status_t bsp_dwin_dgus_write_var_regs(uint16_t var_addr, uint16_t *data, uint16_t data_len, uint8_t need_crc, uint32_t timeout_ms)
{
    DD_Status_t ret;
    uint16_t crc;
    uint8_t req[DWIN_DGUS_MAX_DATA_LEN];

    // 数据帧=帧头+数据长度+指令+数据
    // 数据长度=1(指令) + 2(变量空间首地址) + 写入数据的字节长度
    // 数据=变量空间首地址+写入的数据

    int idx    = 0;
    req[idx++] = DWIN_DGUS_HEADER1;
    req[idx++] = DWIN_DGUS_HEADER2;
    req[idx++] = 3 + data_len * 2;       // 数据长度=1(指令) + 1(页面) + 1(寄存器) + data_len*2
    req[idx++] = 0x82;                   // 指令
    req[idx++] = (var_addr >> 8) & 0xFF; // 变量空间首地址高 8 位
    req[idx++] = var_addr & 0xFF;        // 变量空间首地址低 8 位

    /* 写入的数据 */
    for (uint16_t i = 0; i < data_len; i++) {
        req[idx++] = (data[i] >> 8) & 0xFF; // 第 n 个数据高8位
        req[idx++] = data[i] & 0xFF;        // 第 n 个数据低8位
    }

    if (need_crc) {
        crc        = _calc_crc16(&req[2], idx - 2);
        req[idx++] = crc & 0xFF;        // CRC低字节
        req[idx++] = (crc >> 8) & 0xFF; // CRC高字节
    }

#if DWIN_DGUS_DEBUG
    PRINT_DEBUG("TX(0x82): ");
    for (int i = 0; i < idx; i++) PRINT_DEBUG("%02X ", req[i]);
    PRINT_DEBUG("\n");
#endif

    ret = _cmd_send(req, idx, timeout_ms);
    if (ret != DD_OK) return ret;

    return ret;
}

/* 0x83功能码：从变量空间指定地址开始读指定长度字数据。  */
DD_Status_t bsp_dwin_dgus_read_var_regs(uint16_t var_addr, uint8_t read_len, uint16_t *dest, uint8_t need_crc, uint32_t timeout_ms)
{
    // 数据帧=帧头+数据长度+指令+数据
    // 数据长度=1(指令) + 2(变量空间首地址) + 1(读取数据字长度(0x01-0x7D))
    // 数据=页面+寄存器+读取的数据长度

    if (!dest || read_len == 0 || read_len > 0x7D || timeout_ms == 0) {
        return DD_ERR_PARAM;
    }

    DD_Status_t ret;
    uint16_t crc;
    uint8_t req[DWIN_DGUS_MAX_DATA_LEN];

    int idx    = 0;
    req[idx++] = DWIN_DGUS_HEADER1;
    req[idx++] = DWIN_DGUS_HEADER2;
    req[idx++] = 4;                      // 数据长度
    req[idx++] = 0x83;                   // 指令
    req[idx++] = (var_addr >> 8) & 0xFF; // 变量空间首地址高 8 位
    req[idx++] = var_addr & 0xFF;        // 变量空间首地址低 8 位
    req[idx++] = read_len;               // 读取数据字长度(0x01-0x7D)

    if (need_crc) {
        crc        = _calc_crc16(&req[2], idx - 2);
        req[idx++] = crc & 0xFF;        // CRC低字节
        req[idx++] = (crc >> 8) & 0xFF; // CRC高字节
    }

#if DWIN_DGUS_DEBUG
    PRINT_DEBUG("TX(0x83): ");
    for (int i = 0; i < idx; i++) PRINT_DEBUG("%02X ", req[i]);
    PRINT_DEBUG("\n");
#endif

    ret = _cmd_send(req, idx, timeout_ms);
    if (ret != DD_OK) return ret;

    // 接收数据
    uint8_t resp[DWIN_DGUS_MAX_DATA_LEN];
    memset(resp, 0, sizeof(resp));
    uint32_t t_start = HAL_GetTick();
    uint16_t len     = read_len + 4 + (need_crc ? 2 : 0);
    ret              = _cmd_recive(resp, len, t_start, timeout_ms);

    if (ret != DD_OK) return ret;

    ret = _check_header_and_len(resp, read_len);
    if (ret != DD_OK) return ret;

    if (need_crc) {
        ret = _check_crc16(resp, len);
        if (ret != DD_OK) return ret;
    }

    // 解析数据（大端）到 dest
    for (uint16_t i = 0; i < read_len; i++) {
        uint16_t hi = resp[4 + 2 * i];
        uint16_t lo = resp[4 + 2 * i + 1];
        dest[i]     = (uint16_t)((hi << 8) | lo);
    }

    return ret;
}
