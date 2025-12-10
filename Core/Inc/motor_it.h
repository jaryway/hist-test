#ifndef __MOTOR_IT_H
#define __MOTOR_IT_H

#include "stm32f1xx.h"
// #include "tim.h"
extern TIM_HandleTypeDef htim3;

/*************************用户定义段******************************/
// #define MICRO_STEP              16      // 细分数
// #define T1_FREQ                 1000000 // 时钟分频后频率

#define Motor_PWM_Timer_Handle &htim3
#define Motor_PWM_Timer TIM3
#define Motor_PWM_Timer_Channel TIM_CHANNEL_1

#define Motor_DIR_Port GPIOB // 方向使能引脚
#define Motor_DIR_Pin GPIO_PIN_5

#define Motor_ENA_Port GPIOB // 脱机使能引脚
#define Motor_ENA_Pin GPIO_PIN_3

// #define ENA_PIN  GPIO_PIN_3 // 86电机 ENA+ PB3
// #define DIR_PIN  GPIO_PIN_5 // 86电机 DIR+ PB5
// #define PUL_PIN  GPIO_PIN_4 // 86电机 PUL+ PB4
// #define ENA_PORT GPIOB
// #define DIR_PORT GPIOB
// #define PUL_PORT GPIOB

/*************************结构体定义段*****************************/

typedef enum MotorDirection_t
{
    MOTOR_DIR_CCW = 0, // 反转
    MOTOR_DIR_CW = 1   // 正转
} MotorDirection_t;    // 电机旋转方向定义

typedef enum MotorState_t
{
    MOTOR_STOP = 0,  // 停止状态
    MOTOR_ACCEL = 1, // 加速状态
    MOTOR_DECEL = 2, // 减速状态
    MOTOR_RUN = 3    // 匀速状态
};

typedef struct
{
    GPIO_TypeDef *en_port;
    GPIO_TypeDef *step_port;
    GPIO_TypeDef *dir_port;

    uint16_t en_pin;
    uint16_t step_pin;
    uint16_t dir_pin;

    uint8_t has_en_pin; // or bool has_en_pin;

    uint8_t reversed_dir; // 是否需要反转方向
    uint16_t pulses_per_mm;

    TIM_HandleTypeDef *htim;
    uint32_t tim_channel; // 定时器通道
    TIM_HandleTypeDef *monitor_htim;

    uint8_t state;        /*当前电机状态*/
    MotorDirection_t dir; /*旋转方向*/

    uint32_t total_pulses;
    uint32_t accel_pulses;
    uint32_t decel_pulses;
    uint32_t timer_clock_hz;
    uint32_t pulse_count;
    uint32_t max_rpm;

    uint8_t use_en; // 使用使能，1=使用使能，0=不使用使能，使用使能时，电机停止之后，电机将释放
} Motor_t;          // 梯形加减速变量

/**************************用户函数*******************************/

void Motor_init(Motor_t *motor, uint8_t use_en);
void Motor_attach(Motor_t *motor, GPIO_TypeDef *en_port, uint16_t en_pin, GPIO_TypeDef *step_port, uint16_t step_pin, GPIO_TypeDef *dir_port, uint16_t dir_pin);
void Motor_attach_basic(Motor_t *motor, GPIO_TypeDef *step_port, uint16_t step_pin, GPIO_TypeDef *dir_port, uint16_t dir_pin);
void Motor_attach_timer(Motor_t *motor, TIM_HandleTypeDef *htim, uint32_t tim_channel, TIM_HandleTypeDef *monitor_htim);
void Motor_set_steps_per_mm(Motor_t *motor, uint16_t pulses_per_mm);
void Motor_set_reversed_dir(Motor_t *motor);

void Motor_move(Motor_t *motor, int32_t total_pulses, uint32_t accel_pulses, uint32_t decel_pulses, uint32_t speed);
void Motor_stop(Motor_t *motor);
void Motor_count_pusle_in_it(Motor_t *motor);
uint8_t Motor_update_pwm_freq_in_it(Motor_t *motor);
uint8_t Motor_is_running(Motor_t *motor);

#endif