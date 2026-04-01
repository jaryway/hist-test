
#include "debug.h"
#include "lcd.h"
#include <string.h>
#include <stdio.h>

static uint8_t last_sensor_dn_state = 2;
static uint8_t last_sensor_up_state = 2;
static uint8_t last_sensor_ld_state = 2;
static int16_t last_rpm             = 0;
static int32_t last_pos             = 0;
static float last_load_rate         = 0.0f;
// static uint8_t last_has_cargo       = 0;
static uint8_t first_sensor_init = 1;
static uint8_t first_motor_init  = 1;

void lcd_init(LCD_t *lcd, UART_HandleTypeDef *huart, Screen_t *screen)
{
    lcd->huart  = huart;
    lcd->screen = screen;
}
uint8_t lcd_begin(LCD_t *lcd)
{
    return screen_init(lcd->screen, lcd->huart);
}
void lcd_receive_byte_callback(LCD_t *lcd)
{
    screen_receive_callback(lcd->screen);
}
void lcd_show_motor_init(LCD_t *lcd)
{
    screen_switch_page(lcd->screen, 0x0);
}
void lcd_show_machine_info(LCD_t *lcd)
{
    screen_switch_page(lcd->screen, 0x01);
}
void lcd_update_counter(LCD_t *lcd, const char *total_count, const char *current_count)
{
    screen_write_text(lcd->screen, SN_REG_ADDR_TOT_CNT_VAL, total_count);   // 更新总计数
    screen_write_text(lcd->screen, SN_REG_ADDR_CUR_CNT_VAL, current_count); // 更新当前计数
}
void lcd_update_btn_state(LCD_t *lcd, const uint8_t mode)
{

    const uint8_t *mode_str = mode == 0   ? shangyan
                              : mode == 1 ? xiayan
                                          : pause;
    // const uint8_t *stat_str = stat == 0 ? pause : running;

    uint8_t mode_str_len = sizeof(mode_str) / sizeof(uint8_t);
    // uint8_t stat_str_len    = sizeof(stat_str) / sizeof(uint8_t);

    screen_write_str_bytes(lcd->screen, SN_REG_ADDR_MODE_VAL, mode_str, mode_str_len); // 更新模式
    // screen_write_str_bytes(lcd->screen, SN_REG_ADDR_STA_VAL, stat_str, stat_str_len);  // 更新状态
}
void lcd_update_motor_state(LCD_t *lcd, int16_t rpm, int32_t pos, float load_rate)
{
    char rpm_str[16];
    char pos_str[16];
    char load_rate_str[16];

    if (last_rpm != rpm || first_motor_init) {
        last_rpm = rpm;
        sprintf(rpm_str, "%d", rpm);
        screen_write_text(lcd->screen, SN_REG_ADDR_RPM_VAL, rpm_str);
    }

    if (last_pos != pos || first_motor_init) {
        last_pos = pos;
        sprintf(pos_str, "%ld", pos);
        screen_write_text(lcd->screen, SN_REG_ADDR_POS_VAL, rpm_str);
    }

    if (last_load_rate != load_rate || first_motor_init) {
        last_load_rate = load_rate;
        sprintf(load_rate_str, "%.1f", load_rate / 10.0f);
        screen_write_text(lcd->screen, SN_REG_ADDR_LOAD_RATE_VAL, load_rate_str);
    }
    first_motor_init = 0;
}
void lcd_update_sensor_state(LCD_t *lcd, uint8_t sensor_dn_state, uint8_t sensor_up_state, uint8_t sensor_ld_state)
{
    char sensor_dn_str[2] = {sensor_dn_state ? '1' : '0', '\0'};
    char sensor_up_str[2] = {sensor_up_state ? '1' : '0', '\0'};
    char sensor_ld_str[2] = {sensor_ld_state ? '1' : '0', '\0'};

    if (last_sensor_dn_state != sensor_dn_state || first_sensor_init) {
        last_sensor_dn_state = sensor_dn_state;
        screen_write_text(lcd->screen, SN_REG_ADDR_DN_VAL, sensor_dn_str);
    }
    if (last_sensor_up_state != sensor_up_state || first_sensor_init) {
        last_sensor_up_state = sensor_up_state;
        screen_write_text(lcd->screen, SN_REG_ADDR_UP_VAL, sensor_up_str);
    }
    if (last_sensor_ld_state != sensor_ld_state || first_sensor_init) {
        last_sensor_ld_state = sensor_ld_state;
        screen_write_text(lcd->screen, SN_REG_ADDR_LD_VAL, sensor_ld_str);
    }

    first_sensor_init = 0;
}
