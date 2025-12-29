#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "dma_db.h"

/* 梯形加减速相关参数设置 */
#define TIM_FREQ       72000000U           /* 定时器主频 */
#define MAX_STEP_ANGLE 0.1125              /* 最小步距(1.8/MICRO_STEP) */
#define PAI            3.1415926           /* 圆周率*/
#define FSPR           200                 /* 步进电机单圈步数 */
#define MICRO_STEP     16                  /* 步进电机驱动器细分数 */
#define T1_FREQ        (TIM_FREQ / 72)     /* 频率ft值 */
#define SPR            (FSPR * MICRO_STEP) /* 旋转一圈需要的脉冲数 */

/* 数学常数 */
#define ALPHA       ((float)(2 * PAI / SPR)) /* α = 2*pi/spr */
#define A_T_x10     ((float)(10 * ALPHA * T1_FREQ))
#define T1_FREQ_148 ((float)((T1_FREQ * 0.69) / 10)) /* 0.69为误差修正值 */
#define A_SQ        ((float)(2 * 100000 * ALPHA))
#define A_x200      ((float)(200 * ALPHA)) /* 2*10*10*a/10 */

typedef enum {
    OC_IT   = 0,
    OC_DMA  = 1,
    PWM_IT  = 2,
    PWM_DMA = 3
} RunMode_t;
typedef struct
{
    __IO uint8_t run_state;         /* 电机旋转状态 */
    __IO uint8_t dir;               /* 电机旋转方向 */
    __IO int32_t step_delay;        /* 下个脉冲周期（时间间隔），启动时为加速度 */
    __IO uint32_t decel_start;      /* 开始减速位置 */
    __IO int32_t decel_val;         /* 减速阶段步数 */
    __IO int32_t min_delay;         /* 速度最快，计数值最小的值(最大速度，即匀速段速度) */
    __IO int32_t accel_count;       /* 加减速阶段计数值 */
    __IO int32_t step_position;     /* 当前位置 */
    __IO uint8_t motion_sta;        /* 是否在运动？0：停止，1：运动 */
    __IO uint32_t add_pulse_count;  /* 脉冲个数累计 */
    __IO uint16_t last_accel_delay; /* 加速过程中最后一次延时（脉冲周期） */
    __IO uint32_t step_count;       /* 总移动步数计数器*/
    __IO int32_t rest;              /* 记录new_step_delay中的余数，提高下一步计算的精度 */
    // __IO uint8_t ___i;              /* 定时器使用翻转模式，需要进入两次中断才输出一个完整脉冲 */

    GPIO_TypeDef *en_port;
    GPIO_TypeDef *step_port;
    GPIO_TypeDef *dir_port;

    uint16_t en_pin;
    uint16_t step_pin;
    uint16_t dir_pin;

    uint8_t has_en_pin;

    uint8_t reversed_dir; // 是否需要反转方向
    uint16_t steps_per_mm;

    TIM_HandleTypeDef *htim;
    uint32_t tim_channel; // 定时器通道

    RunMode_t run_mode;    // 运行模式
    uint32_t total_pulses; /* 目标移动总步数 */

    int32_t pulses; /* 带方向的目标移动总步数 */
    uint32_t accel;
    uint32_t decel;
    uint32_t max_speed;

    uint8_t half_phase;

} Motor_t;

enum STA {
    STOP = 0, /* 加减速曲线状态：停止*/
    ACCEL,    /* 加减速曲线状态：加速阶段*/
    DECEL,    /* 加减速曲线状态：减速阶段*/
    RUN       /* 加减速曲线状态：匀速阶段*/
};

enum DIR {
    CW = 0, /* 顺时针 */
    CCW     /* 逆时针 */
};

enum EN {
    EN_ON = 0, /* 失能脱机引脚 */
    EN_OFF     /* 使能脱机引脚 使能后电机停止旋转 */
};

// void motor_init(uint16_t arr, uint16_t psc);

void motor_init(Motor_t *motor); /* 步进电机接口初始化 */
void motor_attach(
    Motor_t *motor,
    GPIO_TypeDef *step_port,
    uint16_t step_pin,
    GPIO_TypeDef *dir_port,
    uint16_t dir_pin,
    GPIO_TypeDef *en_port,
    uint16_t en_pin);
void motor_attach_timer(Motor_t *motor, TIM_HandleTypeDef *htim, uint32_t tim_channel);
void motor_set_steps_per_mm(Motor_t *motor, uint16_t steps_per_mm);
void motor_set_reversed_dir(Motor_t *motor);
void motor_set_pulses(Motor_t *motor, int32_t pulses);
void motor_set_accel(Motor_t *motor, uint32_t accel_rad_per_sec2_x10);
void motor_set_decel(Motor_t *motor, uint32_t decel_rad_per_sec2_x10);
void motor_set_max_speed(Motor_t *motor, uint32_t speed_rad_per_sec_x10);

void motor_oc_start_dma(Motor_t *motor, DMA_DB_t *dma_db);
void motor_oc_stop_dma(Motor_t *motor, DMA_DB_t *dma_db);

void motor_oc_start_it(Motor_t *motor);
void motor_oc_stop_it(Motor_t *motor);

// void motor_start(uint8_t motor_num);                                                                      /* 开启步进电机 */
// void motor_stop(uint8_t motor_num);                                                                       /* 关闭步进电机 */
// void motor_create_t_ctrl_param(Motor_t *motor); /* 梯形加减速控制函数 */

void motor_oc_it_cb_handle(Motor_t *motor);

uint8_t motor_is_stopped(Motor_t *motor);

#endif /* __MOTOR_H */