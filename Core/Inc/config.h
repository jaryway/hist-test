#ifndef __CONFIG_H
#define __CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#define V2_0            0
#define V2_3            1
#define V2_4            2
#define V2_5            3

#define CURRENT_VERSION V2_5

/*************************引脚定义******************************/

/*********** V2.0 ************/

#if CURRENT_VERSION == V2_0
// 按钮
#define BTN_UP_PIN  GPIO_PIN_1 // 上货按钮 SIG1 PB1
#define BTN_DN_PIN  GPIO_PIN_0 // 下货按钮 SIG2 PB0
#define BTN_ST_PIN  GPIO_PIN_7 // 急停按钮 SIG3 PA7
#define BTN_SP_PIN  GPIO_PIN_0 // 档位 SIG8 PA0
#define BTN_UP_PORT GPIOB
#define BTN_DN_PORT GPIOB
#define BTN_ST_PORT GPIOA
#define BTN_SP_PORT GPIOA // 档位 PA0
// LED
#define BTN_LED_UP_PIN  GPIO_PIN_15 // 上货按钮 LED1 PC15
#define BTN_LED_DN_PIN  GPIO_PIN_14 // 下货按钮 LED2 PC14
#define BTN_LED_ST_PIN  GPIO_PIN_13 // 急停按钮 LED3 PC13
#define BTN_LED_UP_PORT GPIOC
#define BTN_LED_DN_PORT GPIOC
#define BTN_LED_ST_PORT GPIOC
// 传感器
#define SENSOR_UP_PIN   GPIO_PIN_6 // 上限位 SIG4 PA6
#define SENSOR_DN_PIN   GPIO_PIN_5 // 下限位 SIG5 PA5
#define SENSOR_LD1_PIN  GPIO_PIN_4 // 装载感应1 SIG6 PA4
#define SENSOR_LD2_PIN  GPIO_PIN_1 // 装载感应2 SIG7 PA1
#define SENSOR_UP_PORT  GPIOA
#define SENSOR_DN_PORT  GPIOA
#define SENSOR_LD1_PORT GPIOA
#define SENSOR_LD2_PORT GPIOA
// 电机
#define ENA_PIN                   GPIO_PIN_3 // 86电机 ENA+ PB3
#define DIR_PIN                   GPIO_PIN_5 // 86电机 DIR+ PB5
#define PUL_PIN                   GPIO_PIN_4 // 86电机 PUL+ PB4
#define ENA_PORT                  GPIOB
#define DIR_PORT                  GPIOB
#define PUL_PORT                  GPIOB
#define MOTOR86_PWM_TIMER         TIM3
#define MOTOR86_PWM_TIMER_CHANNEL TIM_CHANNEL_1 // 86电机脉冲通道 TIM3 TIM_CHANNEL1

#define ENAB_PIN                  GPIO_PIN_7 // 42电机 EN   PB7
#define DIRE_PIN                  GPIO_PIN_8 // 42电机 DIR  PB8
#define STEP_PIN                  GPIO_PIN_9 // 42电机 STEP PB9
#define ENAB_PORT                 GPIOB
#define DIRE_PORT                 GPIOB
#define STEP_PORT                 GPIOB
#define MOTOR42_PWM_TIMER         TIM4
#define MOTOR42_PWM_TIMER_CHANNEL TIM_CHANNEL_4 // 42电机脉冲通道 TIM4 TIM_CHANNEL4

#elif CURRENT_VERSION == V2_3
// 按钮
#define BTN_UP_PIN                  GPIO_PIN_1 // 上货按钮 PB1
#define BTN_DN_PIN                  GPIO_PIN_0 // 下货按钮 PB0
#define BTN_ST_PIN                  GPIO_PIN_7 // 急停按钮 PA7
#define BTN_SP_PIN                  GPIO_PIN_0 // 档位 PA0
#define BTN_UP_PORT                 GPIOB
#define BTN_DN_PORT                 GPIOB
#define BTN_ST_PORT                 GPIOA
#define BTN_SP_PORT                 GPIOA
// LED
#define BTN_LED_UP_PIN              GPIO_PIN_15 // 上货按钮 LED1 PC15
#define BTN_LED_DN_PIN              GPIO_PIN_14 // 下货按钮 LED2 PC14
#define BTN_LED_ST_PIN              GPIO_PIN_13 // 急停按钮 LED3 PC13
#define BTN_LED_UP_PORT             GPIOC
#define BTN_LED_DN_PORT             GPIOC
#define BTN_LED_ST_PORT             GPIOC
// 传感器
#define SENSOR_UP_PIN               GPIO_PIN_6 // 上限位 PA6
#define SENSOR_DN_PIN               GPIO_PIN_5 // 下限位 PA5
#define SENSOR_LD1_PIN              GPIO_PIN_4 // 装载感应1 PA4
#define SENSOR_LD2_PIN              GPIO_PIN_1 // 装载感应2 PA1
#define SENSOR_UP_PORT              GPIOA
#define SENSOR_DN_PORT              GPIOA
#define SENSOR_LD1_PORT             GPIOA
#define SENSOR_LD2_PORT             GPIOA
// 电机
#define ENA_PIN                     GPIO_PIN_7 // 86电机 ENA+ PB7
#define DIR_PIN                     GPIO_PIN_9 // 86电机 DIR+ PB9
#define PUL_PIN                     GPIO_PIN_8 // 86电机 PUL+ PB8 TIM4_CH3
#define ENA_PORT                    GPIOB
#define DIR_PORT                    GPIOB
#define PUL_PORT                    GPIOB
#define MOTOR86_PWM_TIMER           TIM4
#define MOTOR86_PWM_TIMER_CHANNEL   TIM_CHANNEL_3 // 86电机脉冲通道 TIM4 TIM_CHANNEL3

#define PUL1_PIN                    GPIO_PIN_6 // 86电机 PUL1+ PB6 TIM4_CH1
#define PUL1_PORT                   GPIOB
#define MOTOR86_2_PWM_TIMER         TIM4
#define MOTOR86_2_PWM_TIMER_CHANNEL TIM_CHANNEL_1 // 86电机脉冲通道 TIM4 TIM_CHANNEL1

#define ENAB_PIN                    GPIO_PIN_3 // 42电机 EN   PB3
#define DIRE_PIN                    GPIO_PIN_4 // 42电机 DIR  PB4
#define STEP_PIN                    GPIO_PIN_5 // 42电机 STEP PB5
#define ENAB_PORT                   GPIOB
#define DIRE_PORT                   GPIOB
#define STEP_PORT                   GPIOB
#define MOTOR42_PWM_TIMER           TIM3
#define MOTOR42_PWM_TIMER_CHANNEL   TIM_CHANNEL_2 // 42电机脉冲通道 TIM3 TIM_CHANNEL2

#elif CURRENT_VERSION == V2_4
// 按钮
#define BTN_UP_PIN                  GPIO_PIN_1 // 上货按钮 SIG1 PB1
#define BTN_DN_PIN                  GPIO_PIN_0 // 下货按钮 SIG2 PB0
#define BTN_ST_PIN                  GPIO_PIN_7 // 急停按钮 SIG3 PA7
#define BTN_SP_PIN                  GPIO_PIN_0 // 档位 SIG8 PA0
#define BTN_UP_PORT                 GPIOB
#define BTN_DN_PORT                 GPIOB
#define BTN_ST_PORT                 GPIOA
#define BTN_SP_PORT                 GPIOA
// LED
#define BTN_LED_UP_PIN              GPIO_PIN_15 // 上货按钮 LED1 PC15
#define BTN_LED_DN_PIN              GPIO_PIN_14 // 下货按钮 LED2 PC14
#define BTN_LED_ST_PIN              GPIO_PIN_13 // 急停按钮 LED3 PC13
#define BTN_LED_UP_PORT             GPIOC
#define BTN_LED_DN_PORT             GPIOC
#define BTN_LED_ST_PORT             GPIOC
// 传感器
#define SENSOR_DN_PIN               GPIO_PIN_5 // 下限位 SIG5 PA5
#define SENSOR_LD1_PIN              GPIO_PIN_4 // 装载感应1 SIG6 PA4
#define SENSOR_LD2_PIN              GPIO_PIN_1 // 装载感应2 SIG7 PA1
#define SENSOR_DN_PORT              GPIOA
#define SENSOR_LD1_PORT             GPIOA
#define SENSOR_LD2_PORT             GPIOA
// 电机
#define ENA_PIN                     GPIO_PIN_7 // 86电机 ENA PB7
#define DIR_PIN                     GPIO_PIN_9 // 86电机 DIR PB9
#define PUL_PIN                     GPIO_PIN_8 // 86电机 PUL PB8 TIM4_CH3
#define ENA_PORT                    GPIOB
#define DIR_PORT                    GPIOB
#define PUL_PORT                    GPIOB
#define MOTOR86_PWM_TIMER           TIM4
#define MOTOR86_PWM_TIMER_CHANNEL   TIM_CHANNEL_3 // 86电机脉冲通道

#define PUL1_PIN                    GPIO_PIN_6 // 86电机 PUL1+ PB6 TIM4_CH1
#define PUL1_PORT                   GPIOB
#define MOTOR86_2_PWM_TIMER         TIM4
#define MOTOR86_2_PWM_TIMER_CHANNEL TIM_CHANNEL_1 // 86电机脉冲通道

#define ENAB_PIN                    GPIO_PIN_3 // 42电机 ENAB PB3
#define DIRE_PIN                    GPIO_PIN_4 // 42电机 DIRE PB4
#define STEP_PIN                    GPIO_PIN_5 // 42电机 STEP PB5 TIM3_CH2
#define ENAB_PORT                   GPIOB
#define DIRE_PORT                   GPIOB
#define STEP_PORT                   GPIOB
#define MOTOR42_PWM_TIMER           TIM3
#define MOTOR42_PWM_TIMER_CHANNEL   TIM_CHANNEL_2 // 42电机脉冲通道

// MODBUS
#define RS485_RE_PIN                GPIO_PIN_11   // RS485_RE PA11
#define RS485_RE_PORT               GPIOA
#define RS485_BAUD_RATE             19200

#elif CURRENT_VERSION == V2_5
// 按钮
#define BTN_UP_PIN                  GPIO_PIN_12 // 上货按钮 SIG1 PB12
#define BTN_DN_PIN                  GPIO_PIN_13 // 下货按钮 SIG2 PB13
#define BTN_ST_PIN                  GPIO_PIN_14 // 急停按钮 SIG3 PB14
#define BTN_SP_PIN                  GPIO_PIN_9  // 档位 SIG8 PB9
#define BTN_UP_PORT                 GPIOB
#define BTN_DN_PORT                 GPIOB
#define BTN_ST_PORT                 GPIOB
#define BTN_SP_PORT                 GPIOB
// LED
#define BTN_LED_UP_PIN              GPIO_PIN_15 // 上货按钮 LED1 PB15
#define BTN_LED_DN_PIN              GPIO_PIN_8  // 下货按钮 LED2 PA8
#define BTN_LED_ST_PIN              GPIO_PIN_12 // 急停按钮 LED3 PA12
#define BTN_LED_UP_PORT             GPIOB
#define BTN_LED_DN_PORT             GPIOA
#define BTN_LED_ST_PORT             GPIOA
// 传感器
// #define SENSOR_UP_PIN               GPIO_PIN_5  // 上限位 PA5
#define SENSOR_DN_PIN               GPIO_PIN_5  // 下限位 SIG5 PA5
#define SENSOR_LD1_PIN              GPIO_PIN_14 // 装载感应1 SIG6 PC14
#define SENSOR_LD2_PIN              GPIO_PIN_13 // 装载感应2 SIG7 PC13
// #define SENSOR_UP_PORT              GPIOA
#define SENSOR_DN_PORT              GPIOA
#define SENSOR_LD1_PORT             GPIOC
#define SENSOR_LD2_PORT             GPIOC
// 电机
#define ENA_PIN                     GPIO_PIN_15 // 86电机 ENA PC15
#define DIR_PIN                     GPIO_PIN_4  // 86电机 DIR PA4
#define PUL_PIN                     GPIO_PIN_0  // 86电机 PUL PA0 TIM2_CH1
#define ENA_PORT                    GPIOC
#define DIR_PORT                    GPIOA
#define PUL_PORT                    GPIOA
#define MOTOR86_PWM_TIMER           TIM2
#define MOTOR86_PWM_TIMER_CHANNEL   TIM_CHANNEL_1 // 86电机脉冲通道

#define PUL1_PIN                    GPIO_PIN_1 // 86电机 PUL1+ PA1 TIM2_CH2
#define PUL1_PORT                   GPIOA
#define MOTOR86_2_PWM_TIMER         TIM2
#define MOTOR86_2_PWM_TIMER_CHANNEL TIM_CHANNEL_2 // 86电机脉冲通道

#define ENAB_PIN                    GPIO_PIN_7 // 42电机 ENAB PA7
#define DIRE_PIN                    GPIO_PIN_0 // 42电机 DIRE PB0
#define STEP_PIN                    GPIO_PIN_1 // 42电机 STEP PB1 TIM3_CH4
#define ENAB_PORT                   GPIOA
#define DIRE_PORT                   GPIOB
#define STEP_PORT                   GPIOB
#define MOTOR42_PWM_TIMER           TIM3
#define MOTOR42_PWM_TIMER_CHANNEL   TIM_CHANNEL_4 // 42电机脉冲通道

// MODBUS
#define RS485_RE_PIN                GPIO_PIN_11   // RS485_RE PA11
#define RS485_RE_PORT               GPIOA
#define RS485_BAUD_RATE             19200

#endif

#define MOTOR42_ENABLED 0 // 是否启用42电机
/************************* 引脚定义 end******************************/

/*************************用户定义段******************************/

#define MODBUS_HUART &huart1 // modbus 通信
#define LCD_HUART    &huart2 // lcd 屏幕通信
#define DEBUG_HUART  &huart3 // 串口调试

// #define SERIAL_UART &huart3 // 串口监视器使用的USART
// #define TOTAL_DISTANCE        1650    // 导轨有效行程
// #define TOTAL_DISTANCE_MARGIN 0.1f    // 回零时的余量，确保能移动到限位处
#define WIATING_TIME 1000

// #define MAX_RPM               3000    // 转速
// #define PULSES_PER_REV        3200    // 每转脉冲数
// #define LOAD_SPEED            MAX_RPM // 负载速度
// #define LOAD_ACCEL_PULSES     100000  // 负载加速脉冲数
// #define LOAD_DECEL_PULSES     100000  // 负载加速脉冲数
// #define NO_LOAD_SPEED         MAX_RPM // 空载速度
// #define NO_LOAD_ACCEL_PULSES  10000   // 空载加速度脉冲数
// #define NO_LOAD_DECEL_PULSES  10000   // 空载减速度脉冲数
// #define DECEL_RATIO           12      // 减速比
// #define L                     125     // 同步轮一转走的距离

// #define TOTAL_PULSES          TOTAL_DISTANCE / L * DECEL_RATIO * PULSES_PER_REV // 总脉冲数

#define DEBOUNCE_DELAY 20 // 20ms
// #define MICRO_STEP            4      // 电机细分数
// #define T1_FREQ               1000000 // 时钟分频后频率 1MHz = 1us

/*************************结构体定义段*****************************/

#ifdef __cplusplus
}
#endif
#endif