#ifndef __CONFIG_H
#define __CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/*************************引脚定义******************************/
//// v1
//// 按钮
// #define BTN_UP_PIN    GPIO_PIN_7 // 上货按钮
// #define BTN_DN_PIN  GPIO_PIN_6 // 下货按钮
// #define BTN_ST_PIN  GPIO_PIN_5 // 急停按钮
// #define BTN_PORT      GPIOA
// #define BTN_SPEED_PIN    GPIO_PIN_0 // 档位 SIG8
// // LED
// #define BTN_LED_UP_PIN   GPIO_PIN_15 // 上货按钮 LED
// #define BTN_LED_DN_PIN GPIO_PIN_14 // 下货按钮 LED
// #define BTN_LED_ST_PIN GPIO_PIN_13 // 急停按钮 LED

// #define BTN_LED_PORT  GPIOC
// // 传感器
// #define SENSOR_UP_PIN    GPIO_PIN_4 // 上限位
// #define SENSOR_DN_PIN  GPIO_PIN_3 // 下限位
// #define SENSOR_LOAD1_PIN GPIO_PIN_2 // 装载感应1
// #define SENSOR_LD2_PIN GPIO_PIN_1 // 装载感应2
// #define SENSOR_PORT      GPIOA

// // 电机
// #define ENAB_PIN      GPIO_PIN_9 // 42电机 EN
// #define STEP_PIN      GPIO_PIN_9 // 42电机 STEP
// #define DIRE_PIN      GPIO_PIN_8 // 42电机 DIR

// #define ENA_PIN      GPIO_PIN_9 // 86电机 ENA+
// #define PUL_PIN      GPIO_PIN_9 // 86电机 PUL+
// #define DIR_PIN      GPIO_PIN_8 // 86电机 DIR+

/*********** V2.0 ************/
// 按钮
#define BTN_UP_PIN     GPIO_PIN_1 // 上货按钮 PB1
#define BTN_DN_PIN     GPIO_PIN_0 // 下货按钮 PB0
#define BTN_ST_PIN     GPIO_PIN_7 // 急停按钮 PA7
#define BTN_UP_PORT    GPIOB
#define BTN_DN_PORT    GPIOB
#define BTN_ST_PORT    GPIOA
#define BTN_SPEED_PIN  GPIO_PIN_0 // 档位 PA0
#define BTN_SPEED_PORT GPIOA      // 档位 PA0
// LED
#define BTN_LED_UP_PIN GPIO_PIN_15 // 上货按钮 LED PC15
#define BTN_LED_DN_PIN GPIO_PIN_14 // 下货按钮 LED PC14
#define BTN_LED_ST_PIN GPIO_PIN_13 // 急停按钮 LED PC13
#define BTN_LED_PORT   GPIOC

// 传感器
#define SENSOR_UP_PIN  GPIO_PIN_6 // 上限位 PA6
#define SENSOR_DN_PIN  GPIO_PIN_5 // 下限位 PA5
#define SENSOR_LD1_PIN GPIO_PIN_4 // 装载感应1 PA4
#define SENSOR_LD2_PIN GPIO_PIN_1 // 装载感应2 PA1
#define SENSOR_PORT    GPIOA

// 电机
#define ENA_PIN  GPIO_PIN_3 // 86电机 ENA+ PB3
#define DIR_PIN  GPIO_PIN_5 // 86电机 DIR+ PB5
#define PUL_PIN  GPIO_PIN_4 // 86电机 PUL+ PB4
#define ENA_PORT GPIOB
#define DIR_PORT GPIOB
#define PUL_PORT GPIOB
// #define MOTOR86_STEPS_PER_MM      1280 // 导轨走1毫米电机转动的步数
// #define MOTOR86_STEPS_PER_MM      460 // 导轨走1毫米电机转动的步数
#define MOTOR86_STEPS_PER_MM      80 // 导轨走1毫米电机转动的步数
#define MOTOR86_PWM_TIMER         TIM3
#define MOTOR86_PWM_TIMER_CHANNEL TIM_CHANNEL_1 // 86电机脉冲通道 TIM3 TIM_CHANNEL1

#define ENAB_PIN                  GPIO_PIN_7 // 42电机 EN   PB7
#define DIRE_PIN                  GPIO_PIN_8 // 42电机 DIR  PB8
#define STEP_PIN                  GPIO_PIN_9 // 42电机 STEP PB9
#define ENAB_PORT                 GPIOB
#define DIRE_PORT                 GPIOB
#define STEP_PORT                 GPIOB
#define MOTOR42_STEPS_PER_MM      80
#define MOTOR42_PWM_TIMER         TIM4
#define MOTOR42_PWM_TIMER_CHANNEL TIM_CHANNEL_4 // 42电机脉冲通道 TIM4 TIM_CHANNEL4
#define MOTOR42_ENABLED           0             // 是否启用42电机

/************************* 引脚定义 end******************************/

/*************************用户定义段******************************/

#define SERIAL_UART           &huart3 // 串口监视器使用的USART
#define TOTAL_DISTANCE        430     // 导轨有效行程
#define TOTAL_DISTANCE_MARGIN 0.1f    // 回零时的余量，确保能移动到限位处
#define HOME_DISTANCE         100     // 二次回零距离
#define WIATING_TIME          1000

#define LOAD_SPEED            100; // 负载速度
#define LOAD_ACCEL_VAL        100; // 负载加速度
#define NO_LOAD_SPEED         110; // 空载速度
#define NO_LOAD_ACCEL_VAL     100; // 空载加速度

#define DEBOUNCE_DELAY        20      // 20ms
#define MICRO_STEP            16      // 电机细分数
#define T1_FREQ               1000000 // 时钟分频后频率 1MHz = 1us

/*************************结构体定义段*****************************/

#ifdef __cplusplus
}
#endif
#endif