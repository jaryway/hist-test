
#include <stdio.h>
#include <string.h>
#include "debug.h"
#include "helper.h"
#include "screen.h"
// 指令定义
#define TSUIC1_HEADER 0xAA
#define TSUIC1_FOOTER 0xCC33C33C

#define CMD_HANDSHAKE 0x00
// #define CMD_BACKLIGHT        0x30
#define CMD_CLEAR_SCREEN 0x01
#define CMD_DRAW_STRING  0x11
#define CMD_SHOW_JPEG    0x22
#define CMD_DRAW_RECT    0x05
// #define CMD_SHOW_QRCODE      0x21

#define COLOR0 0xD6FD // 背景色 0xD6FD 0x19E0
#define COLOR1 0x2945 // 字体颜色
#define COLOR2 0x528A // 字体颜色2

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
    {SN_REG_ADDR_COMPANY_NAME, 128 + 3, COLOR1, COLOR0, 100, 16, 50, 24},
    {SN_REG_ADDR_SLOGAN, 128 + 3, COLOR1, COLOR0, 114, 48, 110, 24},

    {SN_REG_ADDR_TOT_CNT_TXT, 128 + 2, COLOR1, COLOR0, 24, 97, 50, 24},
    {SN_REG_ADDR_TOT_CNT_VAL, 128 + 4, COLOR1, COLOR0, 100, 94, 110, 24},

    {SN_REG_ADDR_CUR_CNT_TXT, 128 + 2, COLOR1, COLOR0, 24, 143, 50, 24},
    {SN_REG_ADDR_CUR_CNT_VAL, 128 + 4, COLOR1, COLOR0, 100, 140, 110, 24},

    {SN_REG_ADDR_MODE_TXT, 128 + 2, COLOR1, COLOR0, 24, 189, 50, 24},
    {SN_REG_ADDR_MODE_VAL, 128 + 3, COLOR1, COLOR0, 100, 186, 110, 24},

    {SN_REG_ADDR_RPM_TXT, 128 + 2, COLOR1, COLOR0, 259, 97, 50, 24},
    {SN_REG_ADDR_RPM_VAL, 128 + 4, COLOR1, COLOR0, 336, 94, 110, 24},

    {SN_REG_ADDR_POS_TXT, 128 + 2, COLOR1, COLOR0, 259, 143, 50, 24},
    {SN_REG_ADDR_POS_VAL, 128 + 4, COLOR1, COLOR0, 336, 140, 110, 24},

    {SN_REG_ADDR_LOAD_RATE_TXT, 128 + 2, COLOR1, COLOR0, 259, 189, 50, 24},
    {SN_REG_ADDR_LOAD_RATE_VAL, 128 + 4, COLOR1, COLOR0, 336, 186, 110, 24},

    {SN_REG_ADDR_DN_TXT, 128 + 1, COLOR1, COLOR0, 18, 230, 50, 16},
    {SN_REG_ADDR_DN_VAL, 128 + 1, COLOR1, COLOR0, 142, 230, 10, 16},

    {SN_REG_ADDR_UP_TXT, 128 + 1, COLOR1, COLOR0, 174, 230, 50, 16},
    {SN_REG_ADDR_UP_VAL, 128 + 1, COLOR1, COLOR0, 298, 230, 10, 16},

    {SN_REG_ADDR_LD_TXT, 128 + 1, COLOR1, COLOR0, 330, 230, 50, 16},
    {SN_REG_ADDR_LD_VAL, 128 + 1, COLOR1, COLOR0, 454, 230, 10, 16},

    {SN_REG_ADDR_HOMING, 128 + 3, COLOR2, COLOR0, 195, 146, 100, 24},

};

/******************************* 辅助函数 *************** */
// 根据地址查找控件
static Widget_t *_find_widget_by_addr(uint16_t addr)
{
    switch (addr) {
        case SN_REG_ADDR_COMPANY_NAME:
            return &widgets[0];
        case SN_REG_ADDR_SLOGAN:
            return &widgets[1];
        case SN_REG_ADDR_TOT_CNT_TXT:
            return &widgets[2];
        case SN_REG_ADDR_TOT_CNT_VAL:
            return &widgets[3];
        case SN_REG_ADDR_CUR_CNT_TXT:
            return &widgets[4];
        case SN_REG_ADDR_CUR_CNT_VAL:
            return &widgets[5];
        case SN_REG_ADDR_MODE_TXT:
            return &widgets[6];
        case SN_REG_ADDR_MODE_VAL:
            return &widgets[7];
        case SN_REG_ADDR_RPM_TXT:
            return &widgets[8];
        case SN_REG_ADDR_RPM_VAL:
            return &widgets[9];
        case SN_REG_ADDR_POS_TXT:
            return &widgets[10];
        case SN_REG_ADDR_POS_VAL:
            return &widgets[11];
        case SN_REG_ADDR_LOAD_RATE_TXT:
            return &widgets[12];
        case SN_REG_ADDR_LOAD_RATE_VAL:
            return &widgets[13];
        case SN_REG_ADDR_DN_TXT:
            return &widgets[14];
        case SN_REG_ADDR_DN_VAL:
            return &widgets[15];
        case SN_REG_ADDR_UP_TXT:
            return &widgets[16];
        case SN_REG_ADDR_UP_VAL:
            return &widgets[17];
        case SN_REG_ADDR_LD_TXT:
            return &widgets[18];
        case SN_REG_ADDR_LD_VAL:
            return &widgets[19];
        case SN_REG_ADDR_HOMING:
            return &widgets[20];
        default:
            return NULL;
    }
    // for (uint8_t i = 0; i < sizeof(widgets) / sizeof(widgets[0]); i++) {
    //     if (widgets[i].addr == addr) {
    //         return &widgets[i];
    //     }
    // }
    // return NULL; // 未找到

    // // 建立地址到索引的映射表
    // static const uint8_t addr_to_idx[] = {
    //     [SN_REG_ADDR_COMPANY_NAME] = 0,
    //     [SN_REG_ADDR_SLOGAN] = 1,
    //     [SN_REG_ADDR_TOT_CNT_TXT] = 2,
    //     [SN_REG_ADDR_TOT_CNT_VAL] = 3,
    //     [SN_REG_ADDR_CUR_CNT_TXT] = 4,
    //     [SN_REG_ADDR_CUR_CNT_VAL] = 5,
    //     [SN_REG_ADDR_MODE_TXT] = 6,
    //     [SN_REG_ADDR_MODE_VAL] = 7,
    //     [SN_REG_ADDR_RPM_TXT] = 8,
    //     [SN_REG_ADDR_RPM_VAL] = 9,
    //     [SN_REG_ADDR_POS_TXT] = 10,
    //     [SN_REG_ADDR_POS_VAL] = 11,
    //     [SN_REG_ADDR_LOAD_RATE_TXT] = 12,
    //     [SN_REG_ADDR_LOAD_RATE_VAL] = 13,
    //     [SN_REG_ADDR_DN_TXT] = 14,
    //     [SN_REG_ADDR_DN_VAL] = 15,
    //     [SN_REG_ADDR_UP_TXT] = 16,
    //     [SN_REG_ADDR_UP_VAL] = 17,
    //     [SN_REG_ADDR_LD_TXT] = 18,
    //     [SN_REG_ADDR_LD_VAL] = 19,
    //     [SN_REG_ADDR_HOMING] = 20,
    // };

    // if (addr < sizeof(addr_to_idx) / sizeof(addr_to_idx[0])) {
    //     uint8_t idx = addr_to_idx[addr];
    //     if (idx != 0 || addr == SN_REG_ADDR_COMPANY_NAME) {
    //         return &widgets[idx];
    //     }
    // }
    // return NULL;
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

/**
 * @brief 绘制矩形
 * @param screen screen句柄
 * @param mode 绘制模式
 * 0x00=Color 颜色显示矩形框。
 * 0x01=Color 颜色填充矩形区域。
 * 0x02=Color XOR 矩形区域数据，多用于菜单选中/不选中着色。
 * @param color 颜色
 * @param xs 起始坐标x
 * @param ys 起始坐标y
 * @param xe 结束坐标x
 * @param ye 结束坐标y
 */
void _draw_rect(Screen_t *screen, uint16_t mode, uint16_t color, uint16_t xs, uint16_t ys, uint16_t xe, uint16_t ye)
{
    if (screen == NULL || !_is_connected(screen)) {
        PRINT_DEBUG("Error: Cannot draw rectangle - TSUIC1 not connected\r\n");
        return;
    }

    uint8_t data[DWIN_DGUS_MAX_DATA_LEN];
    uint16_t idx = 0;

    data[idx++] = mode;
    data[idx++] = (uint8_t)(color >> 8);   // 颜色高字节
    data[idx++] = (uint8_t)(color & 0xFF); // 颜色低字节
    data[idx++] = (uint8_t)(xs >> 8);      // 起始坐标x高字节
    data[idx++] = (uint8_t)(xs & 0xFF);    // 起始坐标x低字节
    data[idx++] = (uint8_t)(ys >> 8);      // 起始坐标y高字节
    data[idx++] = (uint8_t)(ys & 0xFF);    // 起始坐标y低字节
    data[idx++] = (uint8_t)(xe >> 8);      // 结束坐标x高字节
    data[idx++] = (uint8_t)(xe & 0xFF);    // 结束坐标x低字节
    data[idx++] = (uint8_t)(ye >> 8);      // 结束坐标y高字节
    data[idx++] = (uint8_t)(ye & 0xFF);    // 结束坐标y低字节

    _send_command(screen, CMD_DRAW_RECT, data, idx);
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
SN_Status_t screen_write_text(Screen_t *screen, uint16_t var_addr, const char *text, uint8_t clear_bg)
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

    if (clear_bg) {
        uint16_t xe = (widget->x + widget->width);
        uint16_t ye = (widget->y + widget->height);
        _draw_rect(screen, 0x01, COLOR0, widget->x, widget->y, xe, ye);
    }
    _send_command(screen, CMD_DRAW_STRING, data, len);
    return 1;
}

SN_Status_t screen_write_str_bytes(Screen_t *screen, uint16_t var_addr, const uint8_t *bytes, uint8_t len, uint8_t clear_bg)
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

    if (clear_bg) {
        uint16_t xe = (widget->x + widget->width);
        uint16_t ye = (widget->y + widget->height);
        _draw_rect(screen, 0x01, COLOR0, widget->x, widget->y, xe, ye);
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

    _send_command(screen, CMD_SHOW_JPEG, data, idx); // 更改背景图片

    SN_Status_t res;
    res = screen_write_str_bytes(screen, SN_REG_ADDR_COMPANY_NAME, company_name, 20, 0);
    res = screen_write_str_bytes(screen, SN_REG_ADDR_SLOGAN, slogan, 18, 0);
    if (page == 0x00) {
        res = screen_write_str_bytes(screen, SN_REG_ADDR_HOMING, motor_init_msg, 13, 0); // 显示电机回零中
    } else if (page == 0x01) {
        res = screen_write_str_bytes(screen, SN_REG_ADDR_TOT_CNT_TXT, tot_cnt_txt, 4, 0);
        res = screen_write_str_bytes(screen, SN_REG_ADDR_CUR_CNT_TXT, cur_cnt_txt, 4, 0);
        res = screen_write_str_bytes(screen, SN_REG_ADDR_MODE_TXT, mode_txt, 4, 0);
        res = screen_write_str_bytes(screen, SN_REG_ADDR_RPM_TXT, const_rpm, 4, 0);
        res = screen_write_str_bytes(screen, SN_REG_ADDR_POS_TXT, const_pos, 4, 0);
        res = screen_write_str_bytes(screen, SN_REG_ADDR_LOAD_RATE_TXT, const_load_rate, 4, 0);
        res = screen_write_str_bytes(screen, SN_REG_ADDR_DN_TXT, const_sensor_dn, 6, 0);
        res = screen_write_str_bytes(screen, SN_REG_ADDR_UP_TXT, const_sensor_up, 6, 0);
        res = screen_write_str_bytes(screen, SN_REG_ADDR_LD_TXT, const_sensor_ld, 4, 0);
    }

    return res;
}
