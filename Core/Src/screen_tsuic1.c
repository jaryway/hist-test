
#include <stdio.h>
#include <string.h>
#include "debug.h"
#include "helper.h"
#include "screen.h"

#ifndef SCREEN_TYPE
#define SCREEN_TYPE 2 /* 1: DGUS 2: TSUIC1*/
#endif

// 指令定义
#define TSUIC1_HEADER        0xAA
#define TSUIC1_FOOTER        0xCC33C33C

#define CMD_HANDSHAKE        0x00
#define CMD_BACKLIGHT        0x30
#define CMD_CLEAR_SCREEN     0x01
#define CMD_DRAW_STRING      0x11
#define CMD_SHOW_JPEG        0x22
#define CMD_SHOW_QRCODE      0x21
#define TSUIC1_COLOR_BLACK   0x0000
#define COLOR0               0x00
#define COLOR1               0x2945
#define COLOR2               0x528A
#define TSUIC1_COLOR_RED     0xF800
#define C_GREEN              0x19E0
#define TSUIC1_COLOR_BLUE    0x001F
#define TSUIC1_COLOR_YELLOW  0xFFE0
#define TSUIC1_COLOR_CYAN    0x07FF
#define TSUIC1_COLOR_MAGENTA 0xF81F

typedef struct {
    uint16_t addr;     // 变量地址
    uint8_t mode;      // 显示模式
    uint16_t color;    // 颜色
    uint16_t bg_color; // 背景颜色
    uint16_t x;        // X坐标
    uint16_t y;        // Y坐标
    uint16_t width;    // 宽度
    uint16_t height;   // 高度
} Widget_t;

static uint8_t rx_buffer[256];
static uint8_t rx_index;
static uint8_t received_byte;
static uint8_t frame_started = 0;


// 控件数组
static Widget_t widgets[] = {
    {SN_REG_ADDR_COMPANY_NAME, 128 + 3, COLOR1, COLOR0, 100, 16, 280, 24},
    {SN_REG_ADDR_SLOGAN, 128 + 3, COLOR1, COLOR0, 114, 48, 252, 24},

    {SN_REG_ADDR_TOT_CNT_TXT, 128 + 2, COLOR1, COLOR0, 24, 97, 130, 120},
    {SN_REG_ADDR_TOT_CNT_VAL, 128 + 4, COLOR1, COLOR0, 100, 94, 130, 120},

    {SN_REG_ADDR_CUR_CNT_TXT, 128 + 2, COLOR1, COLOR0, 24, 143, 130, 120},
    {SN_REG_ADDR_CUR_CNT_VAL, 128 + 4, COLOR1, COLOR0, 100, 140, 130, 120},

    // {SN_REG_ADDR_STA_TXT, 128 + 2, COLOR1, COLOR0, 24, 189, 130, 120},
    // {SN_REG_ADDR_STA_VAL, 128 + 3, COLOR1, COLOR0, 100, 186, 130, 120},

    {SN_REG_ADDR_RPM_TXT, 128 + 2, COLOR1, COLOR0, 259, 97, 130, 120},
    {SN_REG_ADDR_RPM_VAL, 128 + 4, COLOR1, COLOR0, 336, 94, 126, 120},

    {SN_REG_ADDR_POS_TXT, 128 + 2, COLOR1, COLOR0, 259, 143, 130, 120},
    {SN_REG_ADDR_POS_VAL, 128 + 4, COLOR1, COLOR0, 336, 140, 130, 120},

     {SN_REG_ADDR_MODE_TXT, 128 + 2, COLOR1, COLOR0, 24, 189, 130, 120},
    {SN_REG_ADDR_MODE_VAL, 128 + 3, COLOR1, COLOR0, 100, 186, 130, 120},

    {SN_REG_ADDR_LOAD_RATE_TXT, 128 + 2, COLOR1, COLOR0, 259, 189, 130, 120},
    {SN_REG_ADDR_LOAD_RATE_VAL, 128 + 4, COLOR1, COLOR0, 336, 186, 130, 120},

    {SN_REG_ADDR_DN_TXT, 128 + 1, COLOR1, COLOR0, 18, 230, 130, 120},
    {SN_REG_ADDR_DN_VAL, 128 + 1, COLOR1, COLOR0, 142, 230, 130, 120},

    {SN_REG_ADDR_UP_TXT, 128 + 1, COLOR1, COLOR0, 174, 230, 130, 120},
    {SN_REG_ADDR_UP_VAL, 128 + 1, COLOR1, COLOR0, 298, 230, 130, 120},

    {SN_REG_ADDR_LD_TXT, 128 + 1, COLOR1, COLOR0, 330, 230, 130, 120},
    {SN_REG_ADDR_LD_VAL, 128 + 1, COLOR1, COLOR0, 454, 230, 130, 120},

    {SN_REG_ADDR_HOMING, 128 + 3, COLOR2, COLOR0, 195, 146, 130, 120},

};

/******************************* 辅助函数 *************** */
// 根据地址查找控件
static Widget_t *_find_widget_by_addr(uint16_t addr)
{
    for (uint8_t i = 0; i < sizeof(widgets) / sizeof(widgets[0]); i++) {
        if (widgets[i].addr == addr) {
            return &widgets[i];
        }
    }
    return NULL; // 未找到
}
// 发送指令（私有函数）
static void _send_command(Screen_t *screen, uint8_t cmd, const uint8_t *data, uint16_t len)
{
    if (screen == NULL || screen->huart == NULL)
        return;

    uint8_t buffer[256];
    uint16_t index = 0;

    buffer[index++] = TSUIC1_HEADER;
    buffer[index++] = cmd;

    if (data != NULL && len > 0) {
        memcpy(&buffer[index], data, len);
        index += len;
    }

    // 添加帧尾
    buffer[index++] = 0xCC;
    buffer[index++] = 0x33;
    buffer[index++] = 0xC3;
    buffer[index++] = 0x3C;

    // PRINT_DEBUG("[TSUIC1] Sending-999: ");
    // PRINT_DEBUG("%s", buffer);

    HAL_UART_Transmit(screen->huart, buffer, index, 1000);
}

static uint8_t _is_connected(Screen_t *screen)
{
    if (screen == NULL)
        return 0;
    return (screen->state == SN_STATE_CONNECTED);
}

static uint8_t _clear_screen(Screen_t *screen, uint16_t color)
{
    if (screen == NULL || !_is_connected(screen)) {
        PRINT_DEBUG("Error: Cannot clear screen - TSUIC1 not connected\r\n");
        return 0;
    }

    uint8_t data[DWIN_DGUS_MAX_DATA_LEN];
    uint16_t idx = 0;

    data[idx++] = (uint8_t)(color >> 8);   // 颜色高字节
    data[idx++] = (uint8_t)(color & 0xFF); // 颜色低字节

    _send_command(screen, CMD_CLEAR_SCREEN, data, idx);
    return 1;
}

// 处理接收到的数据帧
static void _process_received_frame(Screen_t *screen, uint8_t *frame, uint8_t length)
{
    if (screen == NULL || frame == NULL || length < 6)
        return;

    uint8_t command = frame[1];

    switch (command) {
        case CMD_HANDSHAKE: // 握手指令应答
            if (length >= 6 && frame[2] == 0x4F && frame[3] == 0x4B) {
                PRINT_DEBUG("Received handshake ACK\r\n");
                screen->state = SN_STATE_CONNECTED;
            }
            break;

        default:
            PRINT_DEBUG("Received command: 0x%02X\r\n", command);
            break;
    }
}

/******************************* 辅助函数 end *************** */

uint8_t screen_init(Screen_t *screen, UART_HandleTypeDef *huart)
{
    if (screen == NULL || huart == NULL) {
        return 0;
    }

    screen->huart = huart;
    screen->state = SN_STATE_DISCONNECTED;

    PRINT_DEBUG("[TSUIC1] Initting\r\n");
    PRINT_DEBUG("[TSUIC1] UART: 0x%08lX, Baudrate: %lu\r\n",
                (uint32_t)screen->huart,
                screen->huart->Init.BaudRate);

    screen->state = SN_STATE_CONNECTING;
    HAL_UART_Receive_IT(screen->huart, &received_byte, 1);

    PRINT_DEBUG("[TSUIC1] Sending: AA 00 CC 33 C3 3C\r\n");
    _send_command(screen, CMD_HANDSHAKE, NULL, 0);

    // 等待响应
    uint32_t wait_start = HAL_GetTick();
    PRINT_DEBUG("[TSUIC1] Waiting for response...\r\n");
    while ((HAL_GetTick() - wait_start) < 3000) {
        if (screen->state == SN_STATE_CONNECTED) {
            PRINT_DEBUG("[TSUIC1] Handshake successful!\r\n");
            return 1;
        }
        HAL_Delay(100);
    }

    PRINT_DEBUG("[TSUIC1] Handshake timeout\r\n");
    screen->state = SN_STATE_ERROR;
    return 0;
}

void screen_receive_callback(Screen_t *screen)
{
    if ((received_byte == 0xAA) && !frame_started) {
        frame_started         = 1;
        rx_index              = 0;
        rx_buffer[rx_index++] = received_byte;
    } else if (frame_started) {
        // PRINT_DEBUG("[TSUIC1] Received byte: %02X\r\n", received_byte);
        rx_buffer[rx_index++] = received_byte;
        // PRINT_DEBUG("[TSUIC1] Received byte: %s\r\n", rx_buffer);
        if (rx_index >= 6 &&                   //
            rx_buffer[rx_index - 4] == 0xCC && //
            rx_buffer[rx_index - 3] == 0x33 &&
            rx_buffer[rx_index - 2] == 0xC3 && //
            rx_buffer[rx_index - 1] == 0x3C) {

            _process_received_frame(screen, rx_buffer, rx_index);
            // PRINT_DEBUG("[TSUIC1] Received frame: ");
            // PRINT_DEBUG("%s\r\n", rx_buffer);
            frame_started = 0;
            rx_index      = 0;
        }
    }

    // 接收下一个数据
    HAL_UART_Receive_IT(screen->huart, &received_byte, 1);
}

uint8_t *screen_read_data(Screen_t *screen, uint16_t var_addr, uint8_t len)
{
    (void)screen;
    (void)var_addr;
    (void)len;
    return NULL;
}

/**
 * @brief
 */
SN_Status_t screen_write_text(Screen_t *screen, uint16_t var_addr, const char *text)
{
    if (screen == NULL || !_is_connected(screen)) {
        PRINT_DEBUG("Error: Cannot draw string - TSUIC1 not connected\r\n");
        return 0;
    }

    if (text == NULL) {
        PRINT_DEBUG("Error: text is null\r\n");
        return 0;
    }

    uint8_t data[100];
    uint16_t len     = 0;
    uint16_t str_len = strlen(text);
    Widget_t *widget = _find_widget_by_addr(var_addr);

    data[len++] = widget->mode;
    data[len++] = (uint8_t)(widget->color >> 8);      // 颜色高字节
    data[len++] = (uint8_t)(widget->color & 0xFF);    // 颜色低字节
    data[len++] = (uint8_t)(widget->bg_color >> 8);   // 背景色高字节
    data[len++] = (uint8_t)(widget->bg_color & 0xFF); // 背景色低字节
    data[len++] = (uint8_t)(widget->x >> 8);          // 坐标x高字节
    data[len++] = (uint8_t)(widget->x & 0xFF);        // 坐标x低字节
    data[len++] = (uint8_t)(widget->y >> 8);          // 坐标y高字节
    data[len++] = (uint8_t)(widget->y & 0xFF);        // 坐标y低字节

    for (uint8_t i = 0; i < str_len; i++) {
        data[len++] = text[i];
    }
    _send_command(screen, CMD_DRAW_STRING, data, len);
    _send_command(screen, CMD_DRAW_STRING, data, len);
    return 1;
}

SN_Status_t screen_write_str_bytes(Screen_t *screen, uint16_t var_addr, const uint8_t *bytes, uint8_t len)
{
    uint8_t data[DWIN_DGUS_MAX_DATA_LEN];
    uint16_t idx     = 0;
    Widget_t *widget = _find_widget_by_addr(var_addr);

    data[idx++] = widget->mode;
    data[idx++] = (uint8_t)(widget->color >> 8);   // 颜色高字节
    data[idx++] = (uint8_t)(widget->color & 0xFF); // 颜色低字节
    data[idx++] = (uint8_t)(00 >> 8);              // 背景色高字节
    data[idx++] = (uint8_t)(00 & 0xFF);            // 背景色低字节
    data[idx++] = (uint8_t)(widget->x >> 8);       // 坐标x高字节
    data[idx++] = (uint8_t)(widget->x & 0xFF);     // 坐标x低字节
    data[idx++] = (uint8_t)(widget->y >> 8);       // 坐标y高字节
    data[idx++] = (uint8_t)(widget->y & 0xFF);     // 坐标y低字节

    for (uint16_t i = 0; i < len && i < DWIN_DGUS_MAX_DATA_LEN; i++) {
        data[idx++] = bytes[i]; // 高8位为0，低8位为ASCII码
    }
    _send_command(screen, CMD_DRAW_STRING, data, idx);
    return 1;
}
SN_Status_t screen_switch_page(Screen_t *screen, uint8_t page)
{
    _clear_screen(screen, COLOR0);
    uint8_t data[DWIN_DGUS_MAX_DATA_LEN];
    uint16_t idx = 0;

    data[idx++] = (uint8_t)(page >> 8);   // 页面高字节
    data[idx++] = (uint8_t)(page & 0xFF); // 页面低字节

    _send_command(screen, CMD_SHOW_JPEG, data, idx);

    SN_Status_t res;

    // HAL_Delay(10);
    res = screen_write_str_bytes(screen, SN_REG_ADDR_COMPANY_NAME, company_name, 20);
    res = screen_write_str_bytes(screen, SN_REG_ADDR_SLOGAN, slogan, 18);
    if (page == 0x00) {
        // 显示电机回零中
        res = screen_write_str_bytes(screen, SN_REG_ADDR_HOMING, motor_init_msg, 13);
    } else if (page == 0x01) {
        res = screen_write_str_bytes(screen, SN_REG_ADDR_TOT_CNT_TXT, tot_cnt_txt, 4);
        res = screen_write_str_bytes(screen, SN_REG_ADDR_CUR_CNT_TXT, cur_cnt_txt, 4);
        res = screen_write_str_bytes(screen, SN_REG_ADDR_MODE_TXT, mode_txt, 4);
        res = screen_write_str_bytes(screen, SN_REG_ADDR_RPM_TXT, const_rpm, 4);
        res = screen_write_str_bytes(screen, SN_REG_ADDR_POS_TXT, const_pos, 4);
        res = screen_write_str_bytes(screen, SN_REG_ADDR_LOAD_RATE_TXT, const_load_rate, 4);
        res = screen_write_str_bytes(screen, SN_REG_ADDR_DN_TXT, const_sensor_dn, 6);
        res = screen_write_str_bytes(screen, SN_REG_ADDR_UP_TXT, const_sensor_up, 6);
        res = screen_write_str_bytes(screen, SN_REG_ADDR_LD_TXT, const_sensor_ld, 4);
    }

    return res;
}
