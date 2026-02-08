#ifndef __MOTOR_PWM_H__
#define __MOTOR_PWM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "tim.h"

#define MOTOR_TIMER_FREQ 1000000U                             /**< 定时器时钟频率 (Hz) */
#define MOTOR_MIN_SPEED  100U                                 /**< 最低速度 (steps/s) */
#define MOTOR_MIN_ARR    10U                                  /**< ARR 最小值 */
#define MOTOR_MAX_ARR    (MOTOR_TIMER_FREQ / MOTOR_MIN_SPEED) /**< ARR 最大值 */

/** 电机运动状态 */
typedef enum {
    MOTOR_STA_STOP = 0, /**< 停止 (未启动) */
    MOTOR_STA_ACCEL,    /**< 加速中 */
    MOTOR_STA_CONST,    /**< 匀速中 */
    MOTOR_STA_DECEL,    /**< 减速中 */
    MOTOR_STA_FINISHED  /**< 运动完成 */
} MotorState_t;

/** 电机方向 */
typedef enum {
    MOTOR_DIR_CW = 0, /**< 顺时针 */
    MOTOR_DIR_CCW     /**< 逆时针 */
} MotorDir_t;

/** 运动完成回调函数类型 */
typedef void (*MotorCallback_t)(void *user_data);

/** 电机控制结构体 */
typedef struct {
    /* ── 硬件 ── */
    TIM_HandleTypeDef *htim; /**< 定时器句柄 */
    uint32_t tim_channel;    /**< 定时器通道 (TIM_CHANNEL_x) */
    GPIO_TypeDef *dir_port;  /**< 方向引脚端口 */
    uint16_t dir_pin;        /**< 方向引脚 */
    GPIO_TypeDef *ena_port;  /**< 使能引脚端口 */
    uint16_t ena_pin;        /**< 使能引脚 */

    /* ── 运动参数 ── */
    uint32_t speed;         /**< 目标巡航速度 (steps/s) */
    uint32_t accel;         /**< 加速度 (steps/s²) */
    uint32_t decel;         /**< 减速度 (steps/s²) */
    uint32_t current_speed; /**< 当前实时速度 (steps/s) */

    /* ── 位置 ── */
    int32_t current_pos; /**< 当前位置 (步) */
    int32_t target_pos;  /**< 目标位置 (步) */
    uint32_t step_count; /**< 当前阶段已走步数 */

    /* ── 三段式步数分配 ── */
    uint32_t accel_steps; /**< 变速段步数 (加速或减速到目标速度) */
    uint32_t decel_steps; /**< 停车段步数 (减速到 0) */
    uint32_t const_steps; /**< 匀速段步数 */

    /* ── 当前阶段参数 ── */
    uint32_t phase_start_speed; /**< 阶段起始速度 (steps/s) */
    uint32_t phase_end_speed;   /**< 阶段结束速度 (steps/s) */
    uint64_t phase_start_v2;    /**< 阶段起始速度的平方 (v²) */
    int32_t phase_2a;           /**< 2×加速度 (带符号, 加速为正, 减速为负) */
    uint32_t phase_steps;       /**< 当前阶段目标步数 */

    /* ── 定时器 ── */
    uint32_t arr_value; /**< 当前 ARR 寄存器值 */

    /* ── 异步变更 (主循环写, 中断读) ── */
    volatile uint32_t new_speed;      /**< 待生效的新速度 */
    volatile int32_t new_target_pos;  /**< 待生效的新目标位置 */
    volatile uint8_t speed_changing;  /**< 速度变更标志 */
    volatile uint8_t target_changing; /**< 目标位置变更标志 */
    volatile uint8_t finish_pending;  /**< 运动完成标志, 主循环处理回调 */

    /* ── 状态 ── */
    MotorState_t state;   /**< 当前运动状态 */
    MotorDir_t dir;       /**< 当前方向 */
    uint8_t reversed_dir; /**< 方向是否反转 */

    /* ── 回调 ── */
    MotorCallback_t on_finished; /**< 运动完成回调 */
    void *user_data;             /**< 回调用户数据 */
} MotorPWM_t;

/**
 * @brief  初始化电机
 * @param  m:    电机结构体指针
 * @param  htim: 定时器句柄
 * @param  ch:   定时器通道 (TIM_CHANNEL_1/2/3/4)
 * @param  dp:   方向引脚端口
 * @param  dpin: 方向引脚
 * @param  ep:   使能引脚端口
 * @param  epin: 使能引脚
 */
void motor_pwm_init(MotorPWM_t *m, TIM_HandleTypeDef *htim, uint32_t ch,
                    GPIO_TypeDef *dp, uint16_t dpin, GPIO_TypeDef *ep, uint16_t epin);

/**
 * @brief  翻转电机方向
 * @param  m: 电机结构体指针
 */
void motor_pwm_set_reversed_dir(MotorPWM_t *m);

/**
 * @brief  设置目标速度
 * @param  m:     电机结构体指针
 * @param  speed: 目标速度 (steps/s), 运动中可实时修改
 */
void motor_pwm_set_speed(MotorPWM_t *m, uint32_t speed);

/**
 * @brief  设置加速度
 * @param  m:     电机结构体指针
 * @param  accel: 加速度 (steps/s²)
 */
void motor_pwm_set_accel(MotorPWM_t *m, uint32_t accel);

/**
 * @brief  设置减速度
 * @param  m:     电机结构体指针
 * @param  decel: 减速度 (steps/s²)
 */
void motor_pwm_set_decel(MotorPWM_t *m, uint32_t decel);

/**
 * @brief  设置运动完成回调
 * @param  m:         电机结构体指针
 * @param  cb:        回调函数, 在 motor_pwm_poll() 中被调用
 * @param  user_data: 传递给回调的用户数据
 */
void motor_pwm_set_callback(MotorPWM_t *m, MotorCallback_t cb, void *user_data);

/**
 * @brief  运动到绝对位置
 * @param  m:       电机结构体指针
 * @param  abs_pos: 目标绝对位置 (步), 运动中可实时修改
 */
void motor_pwm_move_to(MotorPWM_t *m, int32_t abs_pos);

/**
 * @brief  相对运动
 * @param  m:       电机结构体指针
 * @param  rel_pos: 相对步数, 正值顺时针, 负值逆时针
 */
void motor_pwm_move(MotorPWM_t *m, int32_t rel_pos);

/**
 * @brief  立即停止电机 (不减速)
 * @param  m: 电机结构体指针
 */
void motor_pwm_stop(MotorPWM_t *m);

/**
 * @brief  主循环轮询, 处理运动完成回调
 * @param  m: 电机结构体指针
 * @note   必须在主循环中周期性调用
 */
void motor_pwm_poll(MotorPWM_t *m);

/**
 * @brief  定时器中断回调, 在 HAL_TIM_PeriodElapsedCallback 中调用
 * @param  m: 电机结构体指针
 * @note   每个定时器周期调用一次, 完成位置更新和速度计算
 */
void motor_pwm_tim_callback(MotorPWM_t *m);

/**
 * @brief  获取当前位置
 * @param  m: 电机结构体指针
 * @retval 当前绝对位置 (步)
 */
int32_t motor_pwm_get_current_pos(MotorPWM_t *m);

/**
 * @brief  获取当前状态
 * @param  m: 电机结构体指针
 * @retval 当前运动状态
 */
MotorState_t motor_pwm_get_state(MotorPWM_t *m);

/**
 * @brief  查询电机是否在运动
 * @param  m: 电机结构体指针
 * @retval 1=运动中, 0=停止或已完成
 */
uint8_t motor_pwm_is_running(MotorPWM_t *m);

#ifdef __cplusplus
}
#endif
#endif