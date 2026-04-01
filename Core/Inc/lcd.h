#ifndef __LCD_H
#define __LCD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "screen.h"
#include "main.h"

typedef struct {
    UART_HandleTypeDef *huart;
    Screen_t *screen;
} LCD_t;

void lcd_init(LCD_t *lcd, UART_HandleTypeDef *huart, Screen_t *screen);
uint8_t lcd_begin(LCD_t *lcd);

void lcd_receive_byte_callback(LCD_t *lcd);

void lcd_show_motor_init(LCD_t *lcd);
void lcd_show_machine_info(LCD_t *lcd);

void lcd_update_counter(LCD_t *lcd, const char *total_count, const char *current_count);
void lcd_update_btn_state(LCD_t *lcd, const uint8_t mode);
void lcd_update_motor_state(LCD_t *lcd, int16_t rpm, int32_t pos, float load_rate);
void lcd_update_sensor_state(LCD_t *lcd, uint8_t sensor_dn_state, uint8_t sensor_up_state, uint8_t sensor_ld_state);

#ifdef __cplusplus
}
#endif

#endif /* __LCD_H */