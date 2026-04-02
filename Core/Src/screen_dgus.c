
#include <stdio.h>
#include <string.h>
#include "debug.h"
#include "helper.h"
#include "screen.h"

/* 可调项：发送后等待 TC 的最大等待（ms） */
#ifndef DGUS_TC_WAIT_MS
#define DGUS_TC_WAIT_MS 100
#endif

#define DGUS_HEADER1 0x5A
#define DGUS_HEADER2 0xA5

static SN_Status_t _check_crc16(const uint8_t *buf, uint16_t len)
{
    uint16_t resp_crc = (uint16_t)buf[len - 2] | ((uint16_t)buf[len - 1] << 8);
    uint16_t calc_crc = _calc_crc16(&buf[2], len - 2);
    if (resp_crc == calc_crc) return SN_OK;
    return SN_ERR_CRC;
}

static SN_Status_t _check_header_and_len(const uint8_t *resp, uint16_t read_len)
{
    // 验证帧头
    if (resp[0] != DGUS_HEADER1 || resp[1] != DGUS_HEADER2) {
        return SN_ERR_BAD_RESPONSE;
    }

    // 验证长度字段
    if (resp[2] != (1 + read_len)) { // 长度字段应该 = 指令(1) + 数据长度
        return SN_ERR_BAD_RESPONSE;
    }
    return SN_OK;
}

static SN_Status_t _cmd_send(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    if (!huart) return SN_ERR_HW;

    if (HAL_UART_Transmit(huart, data, len, timeout_ms) != HAL_OK) {
        return SN_ERR_HW;
    }
    return SN_OK;
}

/* 接收封装：根据整体请求开始时间和 timeout_ms 计算剩余超时并执行 HAL_UART_Receive */
static SN_Status_t _cmd_recive(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t len, uint32_t t_start, uint32_t timeout_ms)
{
    if (!huart) return SN_ERR_HW;
    uint32_t elapsed = HAL_GetTick() - t_start;
    if (elapsed >= timeout_ms) return SN_ERR_TIMEOUT;
    uint32_t remaining = timeout_ms - elapsed;
    if (HAL_UART_Receive(huart, buf, len, remaining) != HAL_OK) {
        return SN_ERR_TIMEOUT;
    }
    return SN_OK;
}

// /* 0x80功能码：从指定地址开始写数据串到寄存器。 */
// static SN_Status_t _dgus_write_regs(UART_HandleTypeDef *huart,
//                                     uint8_t page_addr,
//                                     uint8_t reg_addr,
//                                     uint8_t *data,
//                                     uint16_t data_len,
//                                     uint8_t need_crc,
//                                     uint32_t timeout_ms)
// {
//     SN_Status_t ret;
//     uint16_t crc;
//     uint8_t req[DWIN_DGUS_MAX_DATA_LEN];

//     // 数据帧=帧头+数据长度+指令+数据
//     // 数据长度=1(指令) + 1(页面) + 1(寄存器) + 写入数据的字节长度
//     // 数据=页面+寄存器+写入的数据

//     int idx    = 0;
//     req[idx++] = DGUS_HEADER1;
//     req[idx++] = DGUS_HEADER2;
//     req[idx++] = 3 + data_len * 2; // 数据长度=1(指令) + 1(页面) + 1(寄存器) + 写入数据的字节长度
//     req[idx++] = 0x80;             // 指令
//     req[idx++] = page_addr;        // 寄存器页面
//     req[idx++] = reg_addr;         // 寄存器地址

//     /* 写入的数据 */
//     for (uint16_t i = 0; i < data_len; i++) {
//         req[idx++] = data[i];
//     }

//     if (need_crc) {
//         crc        = _calc_crc16(&req[2], idx - 2);
//         req[idx++] = crc & 0xFF;        // CRC低字节
//         req[idx++] = (crc >> 8) & 0xFF; // CRC高字节
//     }

// #if SCREEN_DEBUG
//     PRINT_DEBUG("TX(0x80): ");
//     for (int i = 0; i < idx; i++) PRINT_DEBUG("%02X ", req[i]);
//     PRINT_DEBUG("\n");
// #endif

//     ret = _cmd_send(huart, req, idx, timeout_ms);

//     return ret;
// }

// /* 0x81功能码：从指定寄存器开始读数据。 */
// static SN_Status_t _dgus_read_regs(UART_HandleTypeDef *huart,
//                                    uint8_t page_addr,
//                                    uint8_t reg_addr,
//                                    uint8_t read_len,
//                                    uint16_t *dest,
//                                    uint8_t need_crc,
//                                    uint32_t timeout_ms)
// {
//     // 数据帧=帧头+数据长度+指令+数据
//     // 数据长度=1(指令) + 1(页面) + 1(寄存器) + 1(读取数据字节长度(0x01-0xFB))
//     // 数据=页面+寄存器+读取的数据长度

//     if (!dest || read_len == 0 || read_len > 0xFB || timeout_ms == 0) {
//         return SN_ERR_PARAM;
//     }

//     SN_Status_t ret;
//     uint16_t crc;
//     uint8_t req[DWIN_DGUS_MAX_DATA_LEN];

//     int idx    = 0;
//     req[idx++] = DGUS_HEADER1;
//     req[idx++] = DGUS_HEADER2;
//     req[idx++] = 4;         // 数据长度
//     req[idx++] = 0x81;      // 指令
//     req[idx++] = page_addr; // 寄存器页面
//     req[idx++] = reg_addr;  // 寄存器地址
//     req[idx++] = read_len;  // 读取数据字节长度(0x01-0xFB)

//     if (need_crc) {
//         crc        = _calc_crc16(&req[2], idx - 2);
//         req[idx++] = crc & 0xFF;        // CRC低字节
//         req[idx++] = (crc >> 8) & 0xFF; // CRC高字节
//     }

// #if SCREEN_DEBUG
//     PRINT_DEBUG("TX(0x81): ");
//     for (int i = 0; i < idx; i++) PRINT_DEBUG("%02X ", req[i]);
//     PRINT_DEBUG("\n");
// #endif

//     // 发送数据
//     ret = _cmd_send(huart, req, idx, timeout_ms);
//     if (ret != SN_OK) return ret;

//     // 接收数据
//     uint8_t resp[DWIN_DGUS_MAX_DATA_LEN];
//     memset(resp, 0, sizeof(resp));

//     uint32_t t_start = HAL_GetTick();
//     uint16_t len     = read_len + 4 + (need_crc ? 2 : 0);
//     ret              = _cmd_recive(huart, resp, len, t_start, timeout_ms);
//     if (ret != SN_OK) return ret;

//     ret = _check_header_and_len(resp, read_len);
//     if (ret != SN_OK) return ret;

//     if (need_crc) {
//         ret = _check_crc16(resp, len);
//         if (ret != SN_OK) return ret;
//     }

//     // // 解析数据（大端）到 dest
//     // for (uint16_t i = 0; i < read_len; i++) {
//     //     uint16_t hi = resp[4 + 2 * i];
//     //     uint16_t lo = resp[4 + 2 * i + 1];
//     //     dest[i]     = (uint16_t)((hi << 8) | lo);
//     // }

//     memcpy(dest, &resp[4], read_len);

//     return ret;
// }

/* 0x82功能码：从指定地址开始写数据串(字数据)到变量空间。 */
static SN_Status_t _dgus_write_var_regs(UART_HandleTypeDef *huart,
                                        uint16_t var_addr,
                                        uint8_t *data,
                                        uint16_t data_len,
                                        uint8_t need_crc,
                                        uint32_t timeout_ms)
{
    SN_Status_t ret;
    uint16_t crc;
    uint8_t req[DWIN_DGUS_MAX_DATA_LEN];

    // 数据帧=帧头+数据长度+指令+数据
    // 数据长度=1(指令) + 2(变量空间首地址) + 写入数据的字节长度
    // 数据=变量空间首地址+写入的数据

    int idx    = 0;
    req[idx++] = DGUS_HEADER1;
    req[idx++] = DGUS_HEADER2;
    req[idx++] = 3 + data_len;           // 数据长度=1(指令) + 1(页面) + 1(寄存器) + data_len
    req[idx++] = 0x82;                   // 指令
    req[idx++] = (var_addr >> 8) & 0xFF; // 变量空间首地址高 8 位
    req[idx++] = var_addr & 0xFF;        // 变量空间首地址低 8 位

    /* 写入的数据 */
    for (uint16_t i = 0; i < data_len; i++) {
        req[idx++] = data[i];
    }

    if (need_crc) {
        crc        = _calc_crc16(&req[2], idx - 2);
        req[idx++] = crc & 0xFF;        // CRC低字节
        req[idx++] = (crc >> 8) & 0xFF; // CRC高字节
    }

#if SCREEN_DEBUG
    PRINT_DEBUG("TX(0x82): ");
    for (int i = 0; i < idx; i++) PRINT_DEBUG("%02X ", req[i]);
    PRINT_DEBUG("\n");
#endif

    ret = _cmd_send(huart, req, idx, timeout_ms);
    if (ret != SN_OK) return ret;

    return ret;
}

/* 0x83功能码：从变量空间指定地址开始读指定长度字数据。 */
static SN_Status_t _dgus_read_var_regs(UART_HandleTypeDef *huart,
                                       uint16_t var_addr,
                                       uint8_t read_len,
                                       uint8_t *dest,
                                       uint8_t need_crc,
                                       uint32_t timeout_ms)
{
    // 数据帧=帧头+数据长度+指令+数据
    // 数据长度=1(指令) + 2(变量空间首地址) + 1(读取数据字长度(0x01-0x7D))
    // 数据=页面+寄存器+读取的数据长度

    if (!dest || read_len == 0 || read_len > 0x7D || timeout_ms == 0) {
        return SN_ERR_PARAM;
    }

    SN_Status_t ret;
    uint16_t crc;
    uint8_t req[DWIN_DGUS_MAX_DATA_LEN];

    int idx    = 0;
    req[idx++] = DGUS_HEADER1;
    req[idx++] = DGUS_HEADER2;
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

#if SCREEN_DEBUG
    PRINT_DEBUG("TX(0x83): ");
    for (int i = 0; i < idx; i++) PRINT_DEBUG("%02X ", req[i]);
    PRINT_DEBUG("\n");
#endif

    ret = _cmd_send(huart, req, idx, timeout_ms);
    if (ret != SN_OK) return ret;

    // 接收数据
    uint8_t resp[DWIN_DGUS_MAX_DATA_LEN];
    memset(resp, 0, sizeof(resp));
    uint32_t t_start = HAL_GetTick();
    uint16_t len     = read_len + 4 + (need_crc ? 2 : 0);
    ret              = _cmd_recive(huart, resp, len, t_start, timeout_ms);

    if (ret != SN_OK) return ret;

    ret = _check_header_and_len(resp, read_len);
    if (ret != SN_OK) return ret;

    if (need_crc) {
        ret = _check_crc16(resp, len);
        if (ret != SN_OK) return ret;
    }

    // // 解析数据（大端）到 dest
    // for (uint16_t i = 0; i < read_len; i++) {
    //     uint16_t hi = resp[4 + 2 * i];
    //     uint16_t lo = resp[4 + 2 * i + 1];
    //     dest[i]     = (uint16_t)((hi << 8) | lo);
    // }

    memcpy(dest, &resp[4], read_len);

    return ret;
}
uint8_t screen_init(Screen_t *screen, UART_HandleTypeDef *huart)
{
    screen->huart = huart;
    return 1;
}
void screen_receive_callback(Screen_t *screen)
{
}
uint8_t *screen_read_data(Screen_t *screen, uint16_t var_addr, uint8_t len)
{
    static uint8_t _regs[DWIN_DGUS_MAX_DATA_LEN]; // static 关键字，数据在静态区

    SN_Status_t res = _dgus_read_var_regs(screen->huart, var_addr, len, _regs, 0, 1000);
    if (res != SN_OK) {
        return NULL; // 读取失败返回 NULL
    }

    return _regs;
}

/**
 * @brief
 */
SN_Status_t screen_write_text(Screen_t *screen, uint16_t var_addr, const char *text)
{
    uint8_t data[DWIN_DGUS_MAX_DATA_LEN];
    uint16_t len = strlen(text);

    if (len + 2 > DWIN_DGUS_MAX_DATA_LEN) {
        return SN_ERR_PARAM;
    }

    // 复制文本
    memcpy(data, text, len);

    // 添加 0xFFFF 结束符
    data[len++] = 0xFF;
    data[len++] = 0xFF;

    return _dgus_write_var_regs(screen->huart, var_addr, data, len, 0, 1000);
}

SN_Status_t screen_write_str_bytes(Screen_t *screen, uint16_t var_addr, const uint8_t *bytes, uint8_t len, uint8_t clear_bg)
{
    (void)clear_bg;
    uint8_t data[DWIN_DGUS_MAX_DATA_LEN];

    for (uint16_t i = 0; i < len && i < DWIN_DGUS_MAX_DATA_LEN; i++) {
        data[i] = bytes[i]; // 高8位为0，低8位为ASCII码
    }

    return _dgus_write_var_regs(screen->huart, var_addr, data, len, 0, 1000);
}
SN_Status_t screen_switch_page(Screen_t *screen, uint8_t page)
{

    uint16_t var_addr = 0x84; // 变量地址
    uint8_t data[DWIN_DGUS_MAX_DATA_LEN];
    uint8_t len = 0;
    data[len++] = 0x5A;                   // D3 启动一次页面处理
    data[len++] = 0x01;                   // D2 页面切换
    data[len++] = (uint8_t)(page >> 8);   // D1 页面地址高8位
    data[len++] = (uint8_t)(page & 0xFF); // D0 页面地址低8位

    return _dgus_write_var_regs(screen->huart, var_addr, data, len, 0, 1000);
}
/**/
SN_Status_t screen_set_config(Screen_t *screen, uint8_t config)
{

    uint16_t var_addr = 0x80; // 变量地址-System_Config
    uint8_t data[DWIN_DGUS_MAX_DATA_LEN];
    uint8_t len = 0;

    /*
     D0 系统状态设置如下：
    .7：串口 CRC 校验设置，1=开启，0=关闭，只读。
    .6：保留，写 0。
    .5：上电加载 22 文件初始化变量空间 1=加载 0=不加载，只读。
    .4：变量自动上传设置 1=开启，0=关闭，读写。
    .3：触摸屏伴音控制 1=开启 0=关闭，读写。
    .2：触摸屏背光待机控制 1=开启 0=关闭，读写。
    .1-.0：显示方向 00=0° 01=90° 10=180° 11=270°，读写。
    */
    // uint8_t cof = 0b00000000;

    data[len++] = 0x5A;   // D3 启动一次页面处理
    data[len++] = 0x00;   // D2 触摸屏灵敏度配置值，只读
    data[len++] = 0x00;   // D1 触摸屏模式配置值，只读
    data[len++] = config; // D0 系统状态设置

    return _dgus_write_var_regs(screen->huart, var_addr, data, len, 0, 1000);
}

SN_Status_t screen_get_page(Screen_t *screen)
{
    uint16_t var_addr = 0x14; // 变量地址-当前显示页面ID
    uint8_t dest[DWIN_DGUS_MAX_DATA_LEN];
    uint8_t read_len = 1;

    return _dgus_read_var_regs(screen->huart, var_addr, read_len, dest, 0, 1000);
}