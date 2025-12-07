#ifndef __STEPPER_H
#define __STEPPER_H

#include "stm32f1xx.h"
// #include "tim.h"
extern TIM_HandleTypeDef htim3;

/*************************用户定义段******************************/
// #define MICRO_STEP              16      // 细分数
// #define T1_FREQ                 1000000 // 时钟分频后频率

#define Motor_PWM_Timer_Handle  &htim3
#define Motor_PWM_Timer         TIM3
#define Motor_PWM_Timer_Channel TIM_CHANNEL_1

#define Motor_DIR_Port          GPIOB // 方向使能引脚
#define Motor_DIR_Pin           GPIO_PIN_5

#define Motor_ENA_Port          GPIOB // 脱机使能引脚
#define Motor_ENA_Pin           GPIO_PIN_3

// #define ENA_PIN  GPIO_PIN_3 // 86电机 ENA+ PB3
// #define DIR_PIN  GPIO_PIN_5 // 86电机 DIR+ PB5
// #define PUL_PIN  GPIO_PIN_4 // 86电机 PUL+ PB4
// #define ENA_PORT GPIOB
// #define DIR_PORT GPIOB
// #define PUL_PORT GPIOB

/*************************结构体定义段*****************************/

typedef enum direction_t {
    DIR_CCW = 0, // 反转
    DIR_CW  = 1  // 正转
} direction_t;   // 电机旋转方向定义

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
    uint16_t steps_per_mm;

    TIM_HandleTypeDef *htim;
    uint32_t tim_channel; // 定时器通道

    uint8_t run_state;    /*当前电机状态*/
    direction_t dir;      /*旋转方向*/
    int step_delay;       /*脉冲间隔*/
    uint32_t decel_start; /*减速位置*/
    int decel_val;        /*减速步数*/
    int min_delay;        /*最小间隔*/
    int accel_count;      /*加速步数*/

    uint16_t last_accel_delay; // 加速过程中最后一次脉冲周期
    uint32_t step_count;       // 总移动步数计数器
    int32_t rest;              // 清零余值
    uint8_t interrupt_count;   // 中断计数值
    uint8_t use_en;            // 使用使能，1=使用使能，0=不使用使能，使用使能时，电机停止之后，电机将释放
} StepperTypeDef;              // 梯形加减速变量

/**************************用户函数*******************************/

void Stepper_init(StepperTypeDef *stepper, uint8_t use_en);
void Stepper_attach(StepperTypeDef *stepper, GPIO_TypeDef *en_port, uint16_t en_pin, GPIO_TypeDef *step_port, uint16_t step_pin, GPIO_TypeDef *dir_port, uint16_t dir_pin);
void Stepper_attach_basic(StepperTypeDef *stepper, GPIO_TypeDef *step_port, uint16_t step_pin, GPIO_TypeDef *dir_port, uint16_t dir_pin);
void Stepper_attach_timer(StepperTypeDef *stepper, TIM_HandleTypeDef *htim, uint32_t tim_channel);
void Stepper_set_steps_per_mm(StepperTypeDef *stepper, uint16_t steps_per_mm);
void Stepper_set_reversed_dir(StepperTypeDef *stepper);

void Stepper_move(StepperTypeDef *stepper, int32_t steps, uint32_t accel, uint32_t decel, uint32_t speed);
void Stepper_process(StepperTypeDef *stepper);
void Stepper_stop(StepperTypeDef *stepper);
uint8_t Stepper_is_running(StepperTypeDef *stepper);

/*@brief 生成脉冲表
@param total_pulses 总脉冲数
@param accel 加速步数 脉冲数/s²
@param speed 目标速度 脉冲数/s
*/
void generate_pulse_table(uint32_t total_pulses, uint32_t accel, uint32_t speed);
#endif