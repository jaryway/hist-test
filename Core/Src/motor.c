#include "motor.h"
#include "dma_db.h"
#include <math.h>
#include <stdio.h>

// extern uint32_t start_time = 0;
extern uint32_t start_time;

static void _motor_stop(Motor_t *motor)
{
    if (motor->run_mode == OC_IT) {
        HAL_TIM_OC_Stop_IT(motor->htim, motor->tim_channel);
        // ST1_EN(EN_OFF);
        motor->motion_sta = 0; /* 电机为停止状态  */
        printf("Motor stopped. Total run time: %lu ms\r\n", HAL_GetTick() - start_time);
    }
    // 打印运行耗时
}

/**
 * @brief 电机旋转方向设置
 * @param dir   方向，CW正转，CCW反转
 */
static void _motor_dir_set(Motor_t *motor)
{
    if (motor->dir == CW) {
        GPIO_PinState pin_state = (GPIO_PinState)(motor->reversed_dir ^ GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, pin_state);
    } else {
        GPIO_PinState pin_state = (GPIO_PinState)(motor->reversed_dir ^ GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, pin_state);
    }
}

// static void _calc_step_delay(Motor_t *motor)
// {
//     // __IO uint32_t tim_count = 0;
//     // __IO uint32_t tmp = 0;

//     // __IO static uint16_t last_accel_delay = 0; /* 加速过程中最后一次延时（脉冲周期） */
//     // __IO static uint32_t step_count = 0; /* 总移动步数计数器*/
//     // __IO static int32_t rest = 0; /* 记录new_step_delay中的余数，提高下一步计算的精度 */
//     // __IO static uint8_t ___i = 0;    /* 定时器使用翻转模式，需要进入两次中断才输出一个完整脉冲 */

//     // tim_count = __HAL_TIM_GET_COUNTER(&g_atimx_handle);
//     // tmp = tim_count + motor->step_delay / 2; /* 整个C值里边是需要翻转两次的所以需要除以2 */
//     //
//     // __HAL_TIM_SET_COMPARE(&g_atimx_handle, ATIM_TIMX_PWM_CH1, tmp);

//     motor->___i++; /* 定时器中断次数计数值 */

//     if (motor->___i != 2)
//         return;

//     uint16_t new_step_delay = 0; /* 保存新（下）一个延时周期 */

//     motor->___i = 0;          /* 清零定时器中断次数计数值 */
//     switch (motor->run_state) /* 加减速曲线阶段 */
//     {
//         case STOP:
//             motor->step_count = 0; /* 清零步数计数器 */
//             motor->rest       = 0; /* 清零余值 */
//             /* 关闭通道*/
//             // HAL_TIM_OC_Stop_IT(&g_atimx_handle, ATIM_TIMX_PWM_CH1);
//             // ST1_EN(EN_OFF);
//             // motor->motion_sta = 0; /* 电机为停止状态  */
//             _motor_stop(motor);

//             break;

//         case ACCEL:
//             motor->add_pulse_count++; /* 只用于记录相对位置转动了多少度 */
//             motor->step_count++;      /* 步数加1*/
//             if (motor->dir == CW) {
//                 motor->step_position++; /* 绝对位置加1  记录绝对位置转动多少度*/
//             } else {
//                 motor->step_position--; /* 绝对位置减1*/
//             }
//             motor->accel_count++;                                                                                          /* 加速计数值加1*/
//             new_step_delay = motor->step_delay - (((2 * motor->step_delay) + motor->rest) / (4 * motor->accel_count + 1)); /* 计算新(下)一步脉冲周期(时间间隔) */
//             motor->rest    = ((2 * motor->step_delay) + motor->rest) % (4 * motor->accel_count + 1);                       /* 计算余数，下次计算补上余数，减少误差 */
//             if (motor->step_count >= motor->decel_start)                                                                   /* 检查是否到了需要减速的步数 */
//             {
//                 motor->accel_count = motor->decel_val; /* 加速计数值为减速阶段计数值的初始值 */
//                 motor->run_state   = DECEL;            /* 下个脉冲进入减速阶段 */
//             } else if (new_step_delay <= motor->min_delay) {
//                 /* 检查是否到达期望的最大速度 计数值越小速度越快，当你的速度和最大速度相等或更快就进入匀速*/
//                 motor->last_accel_delay = new_step_delay;   /* 保存加速过程中最后一次延时（脉冲周期）*/
//                 new_step_delay          = motor->min_delay; /* 使用min_delay（对应最大速度speed）*/
//                 motor->rest             = 0;                /* 清零余值 */
//                 motor->run_state        = RUN;              /* 设置为匀速运行状态 */
//             }
//             break;

//         case RUN:
//             motor->add_pulse_count++;
//             motor->step_count++; /* 步数加1 */
//             if (motor->dir == CW) {
//                 motor->step_position++; /* 绝对位置加1 */
//             } else {
//                 motor->step_position--; /* 绝对位置减1*/
//             }
//             new_step_delay = motor->min_delay;           /* 使用min_delay（对应最大速度speed）*/
//             if (motor->step_count >= motor->decel_start) /* 需要开始减速 */
//             {
//                 motor->accel_count = motor->decel_val;        /* 减速步数做为加速计数值 */
//                 new_step_delay     = motor->last_accel_delay; /* 加阶段最后的延时做为减速阶段的起始延时(脉冲周期) */
//                 motor->run_state   = DECEL;                   /* 状态改变为减速 */
//             }
//             break;

//         case DECEL:
//             motor->step_count++; /* 步数加1 */
//             motor->add_pulse_count++;
//             if (motor->dir == CW) {
//                 motor->step_position++; /* 绝对位置加1 */
//             } else {
//                 motor->step_position--; /* 绝对位置减1 */
//             }
//             motor->accel_count++;
//             new_step_delay = motor->step_delay - (((2 * motor->step_delay) + motor->rest) / (4 * motor->accel_count + 1)); /* 计算新(下)一步脉冲周期(时间间隔) */
//             motor->rest    = ((2 * motor->step_delay) + motor->rest) % (4 * motor->accel_count + 1);                       /* 计算余数，下次计算补上余数，减少误差 */

//             /* 检查是否为最后一步 */
//             if (motor->accel_count >= 0) /* 判断减速步数是否从负值加到0是的话 减速完成 */
//             {
//                 motor->run_state  = STOP; /* 设置状态为停止 */
//                 motor->step_count = 0;    /* 清零步数计数器 */
//                 motor->rest       = 0;    /* 清零余值 */
//                 // motor->motion_sta = 0; /* 电机为停止状态  */
//                 _motor_stop(motor);
//             }
//             break;
//     }
//     motor->step_delay = new_step_delay; /* 为下个(新的)延时(脉冲周期)赋值 */
// }

static void _motor_advance_one_step(Motor_t *motor)
{
    uint16_t new_step_delay = motor->step_delay;

    switch (motor->run_state) {
        case STOP:
            motor->step_count = 0;
            motor->rest       = 0;
            _motor_stop(motor);
            return;

        case ACCEL:
            motor->add_pulse_count++;
            motor->step_count++;

            if (motor->dir == CW)
                motor->step_position++;
            else
                motor->step_position--;

            motor->accel_count++;

            new_step_delay = motor->step_delay - (((2 * motor->step_delay) + motor->rest) / (4 * motor->accel_count + 1));
            motor->rest    = ((2 * motor->step_delay) + motor->rest) % (4 * motor->accel_count + 1);

            if (motor->step_count >= motor->decel_start) {
                motor->accel_count = motor->decel_val;
                motor->run_state   = DECEL;
            } else if (new_step_delay <= motor->min_delay) {
                motor->last_accel_delay = new_step_delay;
                new_step_delay          = motor->min_delay;
                motor->rest             = 0;
                motor->run_state        = RUN;
            }
            break;

        case RUN:
            motor->add_pulse_count++;
            motor->step_count++;

            if (motor->dir == CW)
                motor->step_position++;
            else
                motor->step_position--;

            new_step_delay = motor->min_delay;

            if (motor->step_count >= motor->decel_start) {
                motor->accel_count = motor->decel_val;
                new_step_delay     = motor->last_accel_delay;
                motor->run_state   = DECEL;
            }
            break;

        case DECEL:
            motor->step_count++;
            motor->add_pulse_count++;

            if (motor->dir == CW)
                motor->step_position++;
            else
                motor->step_position--;

            motor->accel_count++;

            new_step_delay = motor->step_delay - (((2 * motor->step_delay) + motor->rest) / (4 * motor->accel_count + 1));
            motor->rest    = ((2 * motor->step_delay) + motor->rest) % (4 * motor->accel_count + 1);

            if (motor->accel_count >= 0) {
                motor->run_state  = STOP;
                motor->step_count = 0;
                motor->rest       = 0;
                _motor_stop(motor);
                return;
            }
            break;
    }

    motor->step_delay = new_step_delay;
}

static void _motor_on_half_event(Motor_t *motor)
{
    motor->half_phase ^= 1;

    // 选择在某个相位推进一步：例如 half_phase==0 时推进
    if (motor->half_phase == 0) {
        _motor_advance_one_step(motor);
    }
}

/*
 * @brief       生成梯形运动控制参数
 * @param       pulses 移动的步数 (正数为顺时针，负数为逆时针).
 * @param       accel  加速度,实际值为accel*0.1*rad/sec^2  10倍并且2个脉冲算一个完整的周期
 * @param       decel  减速度,实际值为decel*0.1*rad/sec^2
 * @param       speed  最大速度,实际值为speed*0.1*rad/sec
 * @retval      无
 */
static void _motor_create_t_ctrl_param(Motor_t *motor)
{
    // __IO uint16_t tim_count; /* 达到最大速度时的步数*/
    __IO uint32_t max_s_lim; /* 必须要开始减速的步数（如果加速没有达到最大速度）*/
    __IO uint32_t accel_lim;
    if (motor->motion_sta != 0) /* 只允许步进电机在停止的时候才继续*/
        return;

    __IO int32_t pulses = motor->pulses;
    __IO uint32_t accel = motor->accel;     /* 加速度*/
    __IO uint32_t decel = motor->decel;     /* 减速度*/
    __IO uint32_t speed = motor->max_speed; /* 最大速度*/

    if (pulses < 0) /* 步数为负数 */
    {
        motor->dir = CCW;     /* 逆时针方向旋转 */
        pulses     = -pulses; /* 获取步数绝对值 */
    } else {
        motor->dir = CW; /* 顺时针方向旋转 */
    }

    _motor_dir_set(motor);
    motor->total_pulses = pulses;

    if (pulses == 1) /* 步数为1 */
    {
        motor->accel_count = -1;    /* 只移动一步 */
        motor->run_state   = DECEL; /* 减速状态. */
        motor->step_delay  = 1000;  /* 默认速度 */
    } else if (pulses != 0)         /* 如果目标运动步数不为0*/
    {
        /*设置最大速度极限, 计算得到min_delay用于定时器的计数器的值 min_delay = (alpha / t)/ w*/
        motor->min_delay = (int32_t)(A_T_x10 / speed); /* 匀速运行时的计数值 */

        /*
        通过计算第一个(c0) 的步进延时来设定加速度，其中accel单位为0.1rad/sec^2
         step_delay = 1/tt * sqrt(2*alpha/accel)
         step_delay = ( tfreq*0.69/10 )*10 * sqrt( (2*alpha*100000) / (accel*10) )/100
         */

        motor->step_delay = (int32_t)((T1_FREQ_148 * sqrt(A_SQ / accel)) / 10); /* c0 */

        max_s_lim = (uint32_t)(speed * speed / (A_x200 * accel / 10)); /* 计算多少步之后达到最大速度的限制 max_s_lim = speed^2 / (2*alpha*accel) */

        if (max_s_lim == 0) /* 如果达到最大速度小于0.5步，我们将四舍五入为0,但实际我们必须移动至少一步才能达到想要的速度 */
        {
            max_s_lim = 1;
        }
        accel_lim = (uint32_t)(pulses * decel / (accel + decel)); /* 这里不限制最大速度 计算多少步之后我们必须开始减速 n1 = (n1+n2)decel / (accel + decel) */

        if (accel_lim == 0) /* 不足一步 按一步处理*/
        {
            accel_lim = 1;
        }
        if (accel_lim <= max_s_lim) /* 加速阶段到不了最大速度就得减速。。。使用限制条件我们可以计算出减速阶段步数 */
        {
            motor->decel_val = accel_lim - pulses; /* 减速段的步数 */
        } else {
            motor->decel_val = -(max_s_lim * accel / decel); /* 减速段的步数 */
        }
        if (motor->decel_val == 0) /* 不足一步 按一步处理 */
        {
            motor->decel_val = -1;
        }
        motor->decel_start = pulses + motor->decel_val; /* 计算开始减速时的步数 */

        if (motor->step_delay <= motor->min_delay) /* 如果一开始c0的速度比匀速段速度还大，就不需要进行加速运动，直接进入匀速 */
        {
            motor->step_delay = motor->min_delay;
            motor->run_state  = RUN;
        } else {
            motor->run_state = ACCEL;
        }
        motor->accel_count = 0; /* 复位加减速计数值 */
    }

    motor->motion_sta = 1; /* 电机为运动状态 */

    printf("=====================================\n");
    printf("speed = %lu (rad/sec * 10)\n", speed);
    printf("accel = %lu (rad/sec² * 10)\n", accel);
    printf("decel = %lu (rad/sec² * 10)\n", decel);

    printf("accel_lim = %lu\n", accel_lim);
    printf("max_s_lim = %lu\n", max_s_lim);
    printf("decel_val = %ld\n", motor->decel_val);
    printf("min_delay = %ld (tim counts)\n", motor->min_delay);
    printf("step_delay = %ld (tim counts)\n", motor->step_delay);
}

/* 步进电机接口初始化 */
void motor_init(Motor_t *motor)
{
    motor->motion_sta   = 0;
    motor->dir          = CW;
    motor->step_delay   = 0;
    motor->min_delay    = 0;
    motor->accel_count  = 0;
    motor->run_state    = ACCEL;
    motor->total_pulses = 0;

    motor->reversed_dir = 0;
    motor->steps_per_mm = 100; // 默认100步每毫米
}

void motor_attach(
    Motor_t *motor,
    GPIO_TypeDef *step_port,
    uint16_t step_pin,
    GPIO_TypeDef *dir_port,
    uint16_t dir_pin,
    GPIO_TypeDef *en_port,
    uint16_t en_pin)
{
    motor->step_port = step_port;
    motor->step_pin  = step_pin;
    motor->dir_port  = dir_port;
    motor->dir_pin   = dir_pin;
    motor->en_port   = en_port;
    motor->en_pin    = en_pin;
}
void motor_attach_timer(Motor_t *motor, TIM_HandleTypeDef *htim, uint32_t tim_channel)
{
    motor->htim        = htim;
    motor->tim_channel = tim_channel;
}
void motor_set_steps_per_mm(Motor_t *motor, uint16_t steps_per_mm)
{
    motor->steps_per_mm = steps_per_mm;
}
void motor_set_reversed_dir(Motor_t *motor)
{
    motor->reversed_dir = !motor->reversed_dir;
}

void motor_set_pulses(Motor_t *motor, int32_t pulses)
{
    motor->pulses = pulses;
}
void motor_set_accel(Motor_t *motor, uint32_t accel_rad_per_sec2_x10)
{
    motor->accel = accel_rad_per_sec2_x10;
}
void motor_set_decel(Motor_t *motor, uint32_t decel_rad_per_sec2_x10)
{
    motor->decel = decel_rad_per_sec2_x10;
}
void motor_set_max_speed(Motor_t *motor, uint32_t speed_rad_per_sec_x10)
{
    motor->max_speed = speed_rad_per_sec_x10;
}

uint32_t motor_get_sent_steps_dma(Motor_t *motor, const DMA_DB_t *db)
{
    (void)motor;
    const uint32_t events_per_step = 2u;

    uint32_t sent_events = dma_db_get_sent_elements(db);
    return sent_events / events_per_step;
}

uint32_t motor_get_remaining_steps_dma(Motor_t *motor, const DMA_DB_t *db)
{
    // motor->total_pulses 是目标步数（完整脉冲数）
    uint32_t sent_steps = motor_get_sent_steps_dma(motor, db);
    if (sent_steps >= (uint32_t)motor->total_pulses) return 0;
    return (uint32_t)motor->total_pulses - sent_steps;
}

// 添加回调函数实现
void motor_oc_dma_transfer_complete(void *context)
{
    Motor_t *motor = (Motor_t *)context;

    // 停止电机操作
    // HAL_TIM_OC_Stop_DMA(motor->htim, motor->tim_channel);
    motor->motion_sta = 0;
    motor->run_state  = STOP;
    motor->run_mode   = OC_DMA;
    printf("Motor stopped. Total run time: %lu ms\r\n", HAL_GetTick() - start_time);
    printf("DMA transfer completed for motor\r\n");
}

// 添加回调函数实现
int32_t motor_oc_dma_on_fill_buffer(void *context)
{
    Motor_t *motor = (Motor_t *)context;
    // _calc_step_delay(motor);
    _motor_on_half_event(motor);
    return motor->step_delay / 2;
    // return 20;
}

/* 开启步进电机 */
void motor_oc_start_dma(Motor_t *motor, DMA_DB_t *dma_db)
{

    _motor_create_t_ctrl_param(motor);

    dma_db->mode                       = OC_CCR;
    dma_db->hdma_id                    = TIM_DMA_ID_CC1;
    dma_db->htim                       = motor->htim;
    dma_db->tim_channel                = motor->tim_channel;
    dma_db->total_data                 = motor->total_pulses * 2;
    dma_db->transfered_data            = 0;
    dma_db->callback_context           = motor;
    dma_db->transfer_complete_callback = motor_oc_dma_transfer_complete;
    dma_db->on_fill_buffer             = motor_oc_dma_on_fill_buffer;

    motor->run_mode = OC_DMA;
    // start_time      = HAL_GetTick();
    dma_db_start(dma_db);
}

void motor_oc_stop_dma(Motor_t *motor, DMA_DB_t *dma_db)
{
    motor->motion_sta = 0;
    dma_db_stop(dma_db);
}

void motor_oc_start_it(Motor_t *motor)
{
    start_time = HAL_GetTick();
    _motor_create_t_ctrl_param(motor);
    motor->motion_sta = 1;
    motor->run_mode   = OC_IT;

    HAL_TIM_OC_Start_IT(motor->htim, motor->tim_channel);
}

void motor_oc_stop_it(Motor_t *motor)
{
    HAL_TIM_OC_Stop_IT(motor->htim, motor->tim_channel);
    motor->motion_sta = 0;
}

void motor_oc_it_cb_handle(Motor_t *motor)
{
    uint32_t tim_count = __HAL_TIM_GET_COUNTER(motor->htim);
    uint32_t tmp       = tim_count + motor->step_delay / 2; /* 整个C值里边是需要翻转两次的所以需要除以2 */
    __HAL_TIM_SET_COMPARE(motor->htim, motor->tim_channel, tmp);

    // _calc_step_delay(motor);
    _motor_on_half_event(motor);
}

uint8_t motor_is_stopped(Motor_t *motor)
{
    return (motor->motion_sta == 0);
}