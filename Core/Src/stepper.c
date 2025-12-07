
#include "stepper.h"
#include "math.h"
#include "config.h"
#include <stdio.h>

/********************* 宏定义段 *****************************/

#define PI                     3.1415926
#define MOTOR_STEP_ANGLE       1.8                                                   // 步进电机步距角，1.8°
#define MOTOR_MICRO_STEP_ANGLE ((float)(MOTOR_STEP_ANGLE * PI) / (360 * MICRO_STEP)) // 细分步距角，弧度
#define A_T_x10                ((float)(10 * MOTOR_MICRO_STEP_ANGLE * T1_FREQ))
#define T1_FREQ_148            ((float)((T1_FREQ * 0.676) / 10))
#define A_SQ                   ((float)(2 * 100000 * MOTOR_MICRO_STEP_ANGLE))
#define A_x200                 ((float)(200 * MOTOR_MICRO_STEP_ANGLE))

#define STEPPER_STOP           0 // 停止状态
#define STEPPER_ACCEL          1 // 加速状态
#define STEPPER_DECEL          2 // 减速状态
#define STEPPER_RUN            3 // 匀速状态

// AccelData stepper; // 系统加减速结构体

/******************* 驱动函数定义 ***************************/

/**
 * @brief 电机旋转方向设置
 * @param dir   方向，CW正转，CCW反转
 */
static void Stepper_dir_set(StepperTypeDef *stepper)
{
    printf("stepper->dir=%d\r\n", stepper->dir);
    if (stepper->dir) {
        GPIO_PinState pin_state = (GPIO_PinState)(stepper->reversed_dir ^ GPIO_PIN_SET);
        printf("stepper->dir1=%d\r\n", pin_state);
        HAL_GPIO_WritePin(stepper->dir_port, stepper->dir_pin, pin_state);
    } else {
        GPIO_PinState pin_state = (GPIO_PinState)(stepper->reversed_dir ^ GPIO_PIN_RESET);
        printf("stepper->dir2=%d\r\n", pin_state);
        HAL_GPIO_WritePin(stepper->dir_port, stepper->dir_pin, pin_state);
    }
}

/**
 * @brief   电机启动函数
 */
static void Stepper_start(StepperTypeDef *stepper)
{
    if (stepper->has_en_pin) {
        HAL_GPIO_WritePin(stepper->en_port, stepper->en_pin, GPIO_PIN_RESET);
    }

    HAL_TIM_OC_Start_IT(stepper->htim, stepper->tim_channel);
}

/******************* 用户函数定义 ***************************/

/**
 * @brief 电机初始化
 */
void Stepper_init(StepperTypeDef *stepper, uint8_t use_en)
{
    (void)stepper;
    // 加速过程中最后一次脉冲周期
    stepper->last_accel_delay = 0;
    stepper->step_count       = 0; // 总移动步数计数器
    stepper->rest             = 0;
    stepper->use_en           = use_en;
}

/**
 * @brief  初始化电机引脚，包含使能引脚
 */
void Stepper_attach(StepperTypeDef *stepper, GPIO_TypeDef *en_port, uint16_t en_pin, GPIO_TypeDef *step_port, uint16_t step_pin, GPIO_TypeDef *dir_port, uint16_t dir_pin)
{
    stepper->en_port    = en_port;
    stepper->en_pin     = en_pin;
    stepper->has_en_pin = 1;
    Stepper_attach_basic(stepper, step_port, step_pin, dir_port, dir_pin);
}

/**
 * @brief 初始化电机引脚
 */
void Stepper_attach_basic(StepperTypeDef *stepper, GPIO_TypeDef *step_port, uint16_t step_pin, GPIO_TypeDef *dir_port, uint16_t dir_pin)
{
    stepper->step_port = step_port;
    stepper->step_pin  = step_pin;
    stepper->dir_port  = dir_port;
    stepper->dir_pin   = dir_pin;
}
/**
 * @brief 关联定时器
 */
void Stepper_attach_timer(StepperTypeDef *stepper, TIM_HandleTypeDef *htim, uint32_t tim_channel)
{
    stepper->htim        = htim;
    stepper->tim_channel = tim_channel;
}

void Stepper_set_steps_per_mm(StepperTypeDef *stepper, uint16_t steps_per_mm)
{
    stepper->steps_per_mm = steps_per_mm;
}

void Stepper_set_reversed_dir(StepperTypeDef *stepper)
{
    stepper->reversed_dir = !stepper->reversed_dir;
}

/** @brief  以给定的步数移动步进电机
 *  @note   通过计算加速到最大速度，以给定的步数开始减速，如果加速度和减速度很小，步进电机会移动很慢，还没达到最大速度就要开始减速
 *  @param steps   移动的步数 (正数为顺时针，负数为逆时针).
 *  @param accel  加速加速度（10倍）
 *  @param decel  减速加速度（10倍）
 *  @param speed  最大速度（10倍）
 */
void Stepper_move(StepperTypeDef *stepper, int32_t steps, uint32_t accel, uint32_t decel, uint32_t speed)
{
    stepper->last_accel_delay = 0;
    stepper->step_count       = 0; // 总移动步数计数器
    stepper->rest             = 0;
    // 达到最大速度时的步数.
    unsigned int max_s_lim;
    // 必须开始减速的步数
    unsigned int accel_lim;

    /* 根据步数和正负判断 */
    if (steps == 0) {
        return;
    } else if (steps < 0) // 逆时针
    {
        stepper->dir = DIR_CCW;
        steps        = -steps;
    } else // 顺时针
    {
        stepper->dir = DIR_CW;
    }
    // 输出电机方向
    Stepper_dir_set(stepper);

    // 如果只移动一步
    if (steps == 1) {
        stepper->accel_count = -1;            // 只移动一步
        stepper->run_state   = STEPPER_DECEL; // 减速状态
        stepper->step_delay  = 1000;          // 短延时
    }
    // 步数不为零才移动
    else if (steps != 0) {
        stepper->min_delay  = (int32_t)(A_T_x10 / speed);                         // 设置最大速度极限, 计算得到min_delay用于定时器的计数器的值。
        stepper->step_delay = (int32_t)((T1_FREQ_148 * sqrt(A_SQ / accel)) / 10); // 第一个脉冲计算
        max_s_lim           = (uint32_t)(speed * speed / (A_x200 * accel / 10));  // 计算多少步之后达到最大速度的限制
        if (max_s_lim == 0) {
            max_s_lim = 1; // 如果达到最大速度小于0.5步，将四舍五入为0，但必须移动至少一步才能达到想要的速度
        }
        accel_lim = (uint32_t)(steps * decel / (accel + decel)); // 计算多少步之后开始减速
        if (accel_lim == 0) {
            accel_lim = 1; // 加速至少1步才能才能开始减速.
        }
        // 计算第一次开始减速的位置
        if (accel_lim <= max_s_lim) {
            stepper->decel_val = accel_lim - steps;
        } else {
            stepper->decel_val = -(max_s_lim * accel / decel);
        }
        // 只剩下一步必须减速
        if (stepper->decel_val == 0) {
            stepper->decel_val = -1;
        }
        stepper->decel_start = steps + stepper->decel_val; // 计算开始减速时的步数
        if (stepper->step_delay <= stepper->min_delay) {
            stepper->step_delay = stepper->min_delay; // 如果最大速度很慢，不需要进行加速运动
            stepper->run_state  = STEPPER_RUN;
        } else {
            stepper->run_state = STEPPER_ACCEL;
        }
        // 复位加速度计数值
        stepper->accel_count = 0;
    }
    /* 获取当前计数值 */
    int tim_count = __HAL_TIM_GET_COUNTER(stepper->htim);
    /* 在当前计数值基础上设置定时器比较值 */
    __HAL_TIM_SET_COMPARE(stepper->htim, stepper->tim_channel, tim_count + stepper->step_delay);
    /* 启动输出 */
    // HAL_TIM_OC_Start_IT(stepper->htim, stepper->tim_channel);
    Stepper_start(stepper);
}

/**
 * @brief 定时器中断回调函数
 */
void Stepper_process(StepperTypeDef *stepper)
{
    uint32_t tim_count      = 0;
    uint32_t tmp            = 0;
    uint16_t new_step_delay = 0; // 保存下一个脉冲周期

    // 设置比较值
    tim_count = __HAL_TIM_GET_COUNTER(stepper->htim);
    tmp       = tim_count + stepper->step_delay / 2;
    __HAL_TIM_SET_COMPARE(stepper->htim, stepper->tim_channel, tmp);
    stepper->interrupt_count++;

    if (stepper->interrupt_count != 2)
        return;

    // if (stepper->interrupt_count == 2)
    // {
    stepper->interrupt_count = 0; // 清零定时器中断次数计数值
    switch (stepper->run_state) {
        case STEPPER_STOP:           /* 步进电机停止状态 */
            stepper->step_count = 0; // 清零步数计数器
            stepper->rest       = 0; // 清零余值
            // HAL_TIM_OC_Stop_IT(stepper->htim, stepper->tim_channel);
            Stepper_stop(stepper);
            break;
        case STEPPER_ACCEL: /* 步进电机加速状态 */
            stepper->step_count++;
            stepper->accel_count++;

            new_step_delay = stepper->step_delay - (((2 * stepper->step_delay) + stepper->rest) / (4 * stepper->accel_count + 1)); // 计算新(下)一步脉冲周期(时间间隔)
            stepper->rest  = ((2 * stepper->step_delay) + stepper->rest) % (4 * stepper->accel_count + 1);                         // 计算余数，下次计算补上余数，减少误差

            // 检查是够应该开始减速
            if (stepper->step_count >= stepper->decel_start) {
                stepper->accel_count = stepper->decel_val;
                stepper->run_state   = STEPPER_DECEL;
            }
            // 检查是否到达期望的最大速度
            else if (new_step_delay <= stepper->min_delay) {
                stepper->last_accel_delay = new_step_delay;
                new_step_delay            = stepper->min_delay;
                stepper->rest             = 0;
                stepper->run_state        = STEPPER_RUN;
            }
            break;
        case STEPPER_RUN: /* 步进电机最大速度运行状态 */
            stepper->step_count++;
            new_step_delay = stepper->min_delay;

            // 检查是否需要开始减速
            if (stepper->step_count >= stepper->decel_start) {
                stepper->accel_count = stepper->decel_val;
                new_step_delay       = stepper->last_accel_delay; // 以最后一次加速的延时作为开始减速的延时
                stepper->run_state   = STEPPER_DECEL;
            }
            break;
        case STEPPER_DECEL: /* 步进电机减速状态 */
            stepper->step_count++;
            stepper->accel_count++;
            new_step_delay = stepper->step_delay - (((2 * stepper->step_delay) + stepper->rest) / (4 * stepper->accel_count + 1)); // 计算新(下)一步脉冲周期(时间间隔)
            stepper->rest  = ((2 * stepper->step_delay) + stepper->rest) % (4 * stepper->accel_count + 1);                         // 计算余数，下次计算补上余数，减少误差
            // 检查是否为最后一步
            if (stepper->accel_count >= 0) {
                stepper->run_state = STEPPER_STOP;
            }
            break;
    }
    /* 求得下一次间隔时间 */
    stepper->step_delay = new_step_delay;
    // }
}

/**
 * @brief 电机停止函数
 */
void Stepper_stop(StepperTypeDef *stepper)
{
    HAL_TIM_OC_Stop_IT(stepper->htim, stepper->tim_channel);

    if (stepper->has_en_pin && stepper->use_en) {
        HAL_GPIO_WritePin(stepper->en_port, stepper->en_pin, GPIO_PIN_SET);
    }
}

uint8_t Stepper_is_running(StepperTypeDef *stepper)
{
    return stepper->run_state != STEPPER_STOP;
}
