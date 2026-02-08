#include "motor_pwm.h"
#include <stdlib.h>

/* ════════════ 前向声明 ════════════ */

static void start_motion(MotorPWM_t *m);
static void motor_pwm_plan_motion(MotorPWM_t *m, uint32_t remaining, uint32_t target_speed);

/* ════════════ 快速平方根倒数 ════════════ */

/**
 * @brief  快速平方根倒数 (Quake III 算法)
 * @param  x: 输入值
 * @retval 1/sqrt(x) 的近似值, 两次牛顿迭代
 */
static float fast_inv_sqrt(float x)
{
    float xhalf = 0.5f * x;
    int32_t i;
    volatile float tmp = x;
    i                  = *(int32_t *)&tmp;
    i                  = 0x5f3759df - (i >> 1);
    tmp                = *(float *)&i;
    tmp                = tmp * (1.5f - xhalf * tmp * tmp);
    tmp                = tmp * (1.5f - xhalf * tmp * tmp);
    return tmp;
}

/**
 * @brief  从 v² 计算 ARR 值
 * @param  v2: 速度的平方
 * @retval ARR 寄存器值, 已钳位到 [MIN_ARR, MAX_ARR]
 * @note   ARR = FREQ / sqrt(v2) = FREQ * inv_sqrt(v2)
 */
static uint32_t v2_to_arr(uint64_t v2)
{
    if (v2 == 0) return MOTOR_MAX_ARR;
    float inv_v  = fast_inv_sqrt((float)v2);
    uint32_t arr = (uint32_t)((float)MOTOR_TIMER_FREQ * inv_v + 0.5f);
    if (arr > MOTOR_MAX_ARR) arr = MOTOR_MAX_ARR;
    if (arr < MOTOR_MIN_ARR) arr = MOTOR_MIN_ARR;
    return arr;
}

/**
 * @brief  从 v² 计算速度值
 * @param  v2: 速度的平方
 * @retval 速度 (steps/s), 不低于 MIN_SPEED
 * @note   speed = sqrt(v2) = v2 * inv_sqrt(v2)
 */
static uint32_t v2_to_speed(uint64_t v2)
{
    if (v2 == 0) return 0;
    float inv_v    = fast_inv_sqrt((float)v2);
    uint32_t speed = (uint32_t)((float)v2 * inv_v + 0.5f);
    return (speed < MOTOR_MIN_SPEED) ? MOTOR_MIN_SPEED : speed;
}

/* ════════════ 工具 ════════════ */

/**
 * @brief  速度钳位, 不低于 MIN_SPEED
 * @param  s: 输入速度
 * @retval 钳位后的速度
 */
static inline uint32_t clamp_speed(uint32_t s)
{
    return (s < MOTOR_MIN_SPEED) ? MOTOR_MIN_SPEED : s;
}

/**
 * @brief  速度转 ARR 值
 * @param  s: 速度 (steps/s)
 * @retval ARR 寄存器值, 已钳位
 */
static inline uint32_t speed_to_arr(uint32_t s)
{
    if (s < MOTOR_MIN_SPEED) s = MOTOR_MIN_SPEED;
    uint32_t a = MOTOR_TIMER_FREQ / s;
    if (a > MOTOR_MAX_ARR) a = MOTOR_MAX_ARR;
    if (a < MOTOR_MIN_ARR) a = MOTOR_MIN_ARR;
    return a;
}

/**
 * @brief  计算变速所需步数
 * @param  v1: 起始速度 (steps/s)
 * @param  v2: 终止速度 (steps/s)
 * @param  a:  加/减速度 (steps/s²)
 * @retval 步数, 由公式 n = |v2² - v1²| / (2a) 计算
 */
static uint32_t calc_ramp_steps(uint32_t v1, uint32_t v2, uint32_t a)
{
    if (a == 0) return 0;
    uint64_t s1 = (uint64_t)v1 * v1, s2 = (uint64_t)v2 * v2;
    uint64_t d = (s2 > s1) ? (s2 - s1) : (s1 - s2);
    uint32_t n = (uint32_t)(d / (2ULL * a));
    return (n == 0 && v1 != v2) ? 1 : n;
}

/**
 * @brief  64 位整数平方根 (牛顿迭代)
 * @param  n: 输入值
 * @retval floor(sqrt(n))
 */
static uint32_t isqrt64(uint64_t n)
{
    if (n == 0) return 0;
    uint64_t x = n, y = (x + 1) >> 1;
    while (y < x) {
        x = y;
        y = (x + n / x) >> 1;
    }
    return (uint32_t)x;
}

/* ════════════ 硬件 ════════════ */

/**
 * @brief  设置电机方向
 * @param  m:   电机结构体指针
 * @param  dir: 方向
 */
static void set_direction(MotorPWM_t *m, MotorDir_t dir)
{
    m->dir = dir;
    GPIO_PinState p;
    if (m->reversed_dir)
        p = (dir == MOTOR_DIR_CW) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    else
        p = (dir == MOTOR_DIR_CW) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    HAL_GPIO_WritePin(m->dir_port, m->dir_pin, p);
}

/**
 * @brief  设置电机使能
 * @param  m:  电机结构体指针
 * @param  en: 1=使能, 0=失能
 */
static void set_enable(MotorPWM_t *m, uint8_t en)
{
    HAL_GPIO_WritePin(m->ena_port, m->ena_pin, en ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/**
 * @brief  启动 PWM 输出和定时器中断
 * @param  m: 电机结构体指针
 */
static void start_pwm(MotorPWM_t *m)
{
    __HAL_TIM_SET_AUTORELOAD(m->htim, m->arr_value);
    __HAL_TIM_SET_COMPARE(m->htim, m->tim_channel, m->arr_value >> 1);
    __HAL_TIM_SET_COUNTER(m->htim, 0);
    HAL_TIM_PWM_Start(m->htim, m->tim_channel);
    HAL_TIM_Base_Start_IT(m->htim);
}

/**
 * @brief  停止 PWM 输出和定时器中断
 * @param  m: 电机结构体指针
 */
static void stop_pwm(MotorPWM_t *m)
{
    HAL_TIM_PWM_Stop(m->htim, m->tim_channel);
    HAL_TIM_Base_Stop_IT(m->htim);
}

/* ════════════ 运动完成 ════════════ */

/**
 * @brief  运动完成处理
 * @param  m: 电机结构体指针
 * @note   停止 PWM, 清除状态
 *         若未到达目标位置 (方向反转情况), 自动重新启动
 *         若已到达, 设置 finish_pending 标志, 由主循环触发回调
 */
static void motion_done(MotorPWM_t *m)
{
    stop_pwm(m);
    m->current_speed = 0;
    m->state         = MOTOR_STA_FINISHED;
    set_enable(m, 0);

    if (m->target_pos != m->current_pos) {
        start_motion(m);
        return;
    }

    m->finish_pending = 1;
}

/* ════════════ 阶段管理 ════════════ */

/**
 * @brief  进入新阶段
 * @param  m:         电机结构体指针
 * @param  state:     阶段状态 (ACCEL / CONST / DECEL)
 * @param  start_spd: 阶段起始速度 (steps/s)
 * @param  end_spd:   阶段结束速度 (steps/s)
 * @param  a:         加/减速度 (steps/s²), 匀速时传 0
 * @param  steps:     本阶段目标步数
 * @note   重置 step_count, 预算 v² 和 2a, 设置初始 ARR
 */
static void enter_phase(MotorPWM_t *m, MotorState_t state,
                        uint32_t start_spd, uint32_t end_spd,
                        uint32_t a, uint32_t steps)
{
    m->state             = state;
    m->step_count        = 0;
    m->phase_steps       = steps;
    m->phase_start_speed = start_spd;
    m->phase_end_speed   = end_spd;
    m->phase_start_v2    = (uint64_t)start_spd * start_spd;
    m->current_speed     = start_spd;
    m->arr_value         = speed_to_arr(start_spd);

    if (state == MOTOR_STA_ACCEL)
        m->phase_2a = (int32_t)(2U * a);
    else if (state == MOTOR_STA_DECEL)
        m->phase_2a = -(int32_t)(2U * a);
    else
        m->phase_2a = 0;
}

/**
 * @brief  根据 v² 公式更新当前速度和 ARR
 * @param  m: 电机结构体指针
 * @note   公式: v² = start_v² + 2a × step_count
 *         结果钳位到 [MIN_SPEED², end_speed²]
 */
static void update_speed_from_v2(MotorPWM_t *m)
{
    int64_t v2 = (int64_t)m->phase_start_v2 + (int64_t)m->phase_2a * (int64_t)m->step_count;

    uint64_t end_v2 = (uint64_t)m->phase_end_speed * m->phase_end_speed;
    uint64_t min_v2 = (uint64_t)MOTOR_MIN_SPEED * MOTOR_MIN_SPEED;

    if (m->phase_2a > 0) {
        if ((uint64_t)v2 > end_v2) v2 = (int64_t)end_v2;
    } else {
        if (v2 < (int64_t)end_v2) v2 = (int64_t)end_v2;
    }
    if (v2 < (int64_t)min_v2) v2 = (int64_t)min_v2;

    m->arr_value     = v2_to_arr((uint64_t)v2);
    m->current_speed = v2_to_speed((uint64_t)v2);
}

/* ════════════ 阶段切换 ════════════ */

/**
 * @brief  当前阶段步数走完, 推进到下一阶段
 * @param  m: 电机结构体指针
 * @note   根据当前状态决定下一步:
 *         ACCEL 完成 → CONST 或 DECEL 或结束
 *         CONST 完成 → DECEL 或结束
 *         DECEL 完成 → 若为变速段则进 CONST/DECEL, 否则结束
 */
static void advance_phase(MotorPWM_t *m)
{
    switch (m->state) {

        case MOTOR_STA_ACCEL: {
            uint32_t spd     = clamp_speed(m->phase_end_speed);
            m->current_speed = spd;
            m->arr_value     = speed_to_arr(spd);

            if (m->const_steps > 0)
                enter_phase(m, MOTOR_STA_CONST, spd, spd, 0, m->const_steps);
            else if (m->decel_steps > 0)
                enter_phase(m, MOTOR_STA_DECEL, spd, MOTOR_MIN_SPEED, m->decel, m->decel_steps);
            else
                motion_done(m);
            break;
        }

        case MOTOR_STA_CONST:
            if (m->decel_steps > 0)
                enter_phase(m, MOTOR_STA_DECEL, m->current_speed, MOTOR_MIN_SPEED, m->decel, m->decel_steps);
            else
                motion_done(m);
            break;

        case MOTOR_STA_DECEL: {
            uint32_t spd     = clamp_speed(m->phase_end_speed);
            m->current_speed = spd;
            m->arr_value     = speed_to_arr(spd);

            if (spd > MOTOR_MIN_SPEED + 10) {
                if (m->const_steps > 0)
                    enter_phase(m, MOTOR_STA_CONST, spd, spd, 0, m->const_steps);
                else if (m->decel_steps > 0)
                    enter_phase(m, MOTOR_STA_DECEL, spd, MOTOR_MIN_SPEED, m->decel, m->decel_steps);
                else
                    motion_done(m);
            } else {
                motion_done(m);
            }
            break;
        }

        default:
            break;
    }
}

/* ════════════ 运动规划 ════════════ */

/**
 * @brief  规划运动轨迹 (三段式: 变速 → 匀速 → 停车)
 * @param  m:            电机结构体指针
 * @param  remaining:    剩余步数
 * @param  target_speed: 目标巡航速度 (steps/s)
 * @note   根据当前速度、目标速度和剩余距离, 分配三段步数:
 *         accel_steps: 变速段 (加速或减速到目标速度)
 *         const_steps: 匀速段
 *         decel_steps: 停车段 (减速到 0)
 *         距离不足时自动降低峰值速度或直接刹车
 */
static void motor_pwm_plan_motion(MotorPWM_t *m, uint32_t remaining, uint32_t target_speed)
{
    if (remaining == 0) {
        m->state = MOTOR_STA_FINISHED;
        return;
    }

    uint32_t cur = clamp_speed(m->current_speed);
    uint32_t tgt = clamp_speed(target_speed);

    uint32_t stop_steps   = calc_ramp_steps(tgt, 0, m->decel);
    uint32_t urgent_steps = calc_ramp_steps(cur, 0, m->decel);

    /* 距离不够刹车, 全力减速 */
    if (remaining <= urgent_steps) {
        m->accel_steps = 0;
        m->const_steps = 0;
        m->decel_steps = remaining;
        enter_phase(m, MOTOR_STA_DECEL, cur, MOTOR_MIN_SPEED, m->decel, remaining);
        return;
    }

    if (cur < tgt) {
        /* 需要加速 */
        uint32_t change_steps = calc_ramp_steps(cur, tgt, m->accel);

        if (change_steps + stop_steps <= remaining) {
            /* 距离足够: 加速 → 匀速 → 停车 */
            m->accel_steps = change_steps;
            m->const_steps = remaining - change_steps - stop_steps;
            m->decel_steps = stop_steps;

            if (change_steps > 0)
                enter_phase(m, MOTOR_STA_ACCEL, cur, tgt, m->accel, change_steps);
            else
                enter_phase(m, MOTOR_STA_CONST, tgt, tgt, 0, m->const_steps);
        } else {
            /* 距离不够: 算峰值速度, 加速 → 直接减速 */
            uint64_t csq  = (uint64_t)cur * cur;
            uint64_t num  = 2ULL * m->accel * m->decel * remaining + (uint64_t)m->decel * csq;
            uint64_t den  = (uint64_t)m->accel + m->decel;
            uint32_t peak = isqrt64(num / den);

            if (peak <= cur) peak = cur + 1;
            if (peak > tgt) peak = tgt;

            uint32_t up = calc_ramp_steps(cur, peak, m->accel);
            if (up >= remaining) up = remaining - 1;

            m->accel_steps = up;
            m->const_steps = 0;
            m->decel_steps = remaining - up;

            enter_phase(m, MOTOR_STA_ACCEL, cur, peak, m->accel, up);
        }

    } else if (cur > tgt) {
        /* 需要减速到目标速度 */
        uint32_t change_steps = calc_ramp_steps(cur, tgt, m->decel);

        if (change_steps + stop_steps <= remaining) {
            /* 距离足够: 减速到目标 → 匀速 → 停车 */
            m->accel_steps = change_steps;
            m->const_steps = remaining - change_steps - stop_steps;
            m->decel_steps = stop_steps;
            enter_phase(m, MOTOR_STA_DECEL, cur, tgt, m->decel, change_steps);
        } else {
            /* 距离不够: 直接全力刹车 */
            m->accel_steps = 0;
            m->const_steps = 0;
            m->decel_steps = remaining;
            enter_phase(m, MOTOR_STA_DECEL, cur, MOTOR_MIN_SPEED, m->decel, remaining);
        }

    } else {
        /* 已在目标速度 */
        if (stop_steps >= remaining) {
            /* 只够刹车 */
            m->accel_steps = 0;
            m->const_steps = 0;
            m->decel_steps = remaining;
            enter_phase(m, MOTOR_STA_DECEL, cur, MOTOR_MIN_SPEED, m->decel, remaining);
        } else {
            /* 匀速 → 停车 */
            m->accel_steps = 0;
            m->const_steps = remaining - stop_steps;
            m->decel_steps = stop_steps;
            enter_phase(m, MOTOR_STA_CONST, tgt, tgt, 0, m->const_steps);
        }
    }
}

/**
 * @brief  启动运动
 * @param  m: 电机结构体指针
 * @note   设置方向, 使能电机, 从 MIN_SPEED 开始规划运动
 */
static void start_motion(MotorPWM_t *m)
{
    int32_t diff = m->target_pos - m->current_pos;
    if (diff == 0) {
        m->state = MOTOR_STA_FINISHED;
        return;
    }

    set_direction(m, (diff > 0) ? MOTOR_DIR_CW : MOTOR_DIR_CCW);
    set_enable(m, 1);

    m->current_speed = MOTOR_MIN_SPEED;
    m->step_count    = 0;
    motor_pwm_plan_motion(m, (uint32_t)abs(diff), m->speed);
    if (m->state == MOTOR_STA_FINISHED) return;
    m->arr_value = speed_to_arr(m->current_speed);
    start_pwm(m);
}

/* ════════════ API ════════════ */

void motor_pwm_init(MotorPWM_t *m, TIM_HandleTypeDef *htim, uint32_t ch,
                    GPIO_TypeDef *dp, uint16_t dpin, GPIO_TypeDef *ep, uint16_t epin)
{
    m->htim              = htim;
    m->tim_channel       = ch;
    m->dir_port          = dp;
    m->dir_pin           = dpin;
    m->ena_port          = ep;
    m->ena_pin           = epin;
    m->speed             = 1000;
    m->accel             = 5000;
    m->decel             = 5000;
    m->current_speed     = 0;
    m->current_pos       = 0;
    m->target_pos        = 0;
    m->step_count        = 0;
    m->accel_steps       = 0;
    m->decel_steps       = 0;
    m->const_steps       = 0;
    m->phase_start_speed = 0;
    m->phase_end_speed   = 0;
    m->phase_start_v2    = 0;
    m->phase_2a          = 0;
    m->phase_steps       = 0;
    m->arr_value         = 0;
    m->new_speed         = 0;
    m->speed_changing    = 0;
    m->new_target_pos    = 0;
    m->target_changing   = 0;
    m->finish_pending    = 0;
    m->state             = MOTOR_STA_STOP;
    m->dir               = MOTOR_DIR_CW;
    m->reversed_dir      = 0;
    m->on_finished       = NULL;
    m->user_data         = NULL;
    set_enable(m, 0);
}

void motor_pwm_set_reversed_dir(MotorPWM_t *m)
{
    m->reversed_dir = !m->reversed_dir;
}

void motor_pwm_set_accel(MotorPWM_t *m, uint32_t a)
{
    m->accel = a;
}
void motor_pwm_set_decel(MotorPWM_t *m, uint32_t d)
{
    m->decel = d;
}

void motor_pwm_set_callback(MotorPWM_t *m, MotorCallback_t cb, void *user_data)
{
    m->on_finished = cb;
    m->user_data   = user_data;
}

void motor_pwm_set_speed(MotorPWM_t *m, uint32_t speed)
{
    speed = clamp_speed(speed);
    if (m->state == MOTOR_STA_STOP || m->state == MOTOR_STA_FINISHED)
        m->speed = speed;
    else {
        m->new_speed      = speed;
        m->speed_changing = 1;
    }
}

void motor_pwm_move_to(MotorPWM_t *m, int32_t pos)
{
    if (m->state == MOTOR_STA_STOP || m->state == MOTOR_STA_FINISHED) {
        m->target_pos = pos;
        start_motion(m);
    } else {
        m->new_target_pos  = pos;
        m->target_changing = 1;
    }
}

void motor_pwm_move(MotorPWM_t *m, int32_t rel)
{
    if (m->state == MOTOR_STA_STOP || m->state == MOTOR_STA_FINISHED)
        motor_pwm_move_to(m, m->current_pos + rel);
    else
        motor_pwm_move_to(m, m->target_pos + rel);
}

void motor_pwm_stop(MotorPWM_t *m)
{
    stop_pwm(m);
    m->current_speed = 0;
    m->state         = MOTOR_STA_STOP;
    set_enable(m, 0);
}

void motor_pwm_poll(MotorPWM_t *m)
{
    if (m->finish_pending) {
        m->finish_pending = 0;
        if (m->on_finished)
            m->on_finished(m->user_data);
    }
}

/* ════════════ 中断 ════════════ */

void motor_pwm_tim_callback(MotorPWM_t *m)
{
    if (m->state == MOTOR_STA_STOP || m->state == MOTOR_STA_FINISHED) return;

    /* 1. 位置更新 */
    if (m->dir == MOTOR_DIR_CW)
        m->current_pos++;
    else
        m->current_pos--;

    m->step_count++;

    /* 2. 目标位置变更 */
    if (m->target_changing) {
        m->target_changing = 0;
        m->target_pos      = m->new_target_pos;
        int32_t diff       = m->target_pos - m->current_pos;

        if (diff == 0) {
            motion_done(m);
            return;
        }

        MotorDir_t nd = (diff > 0) ? MOTOR_DIR_CW : MOTOR_DIR_CCW;
        uint32_t rem  = (uint32_t)abs(diff);
        if (nd != m->dir) {
            /* 方向反转: 先减速到 0, motion_done 中会重新 start_motion */
            uint32_t dn = calc_ramp_steps(m->current_speed, 0, m->decel);
            if (dn == 0) dn = 1;
            m->accel_steps = 0;
            m->const_steps = 0;
            m->decel_steps = dn;
            enter_phase(m, MOTOR_STA_DECEL, m->current_speed, MOTOR_MIN_SPEED, m->decel, dn);
        } else {
            /* 同方向: 重新规划 */
            motor_pwm_plan_motion(m, rem, m->speed);
        }
    }

    /* 3. 速度变更 */
    if (m->speed_changing) {
        m->speed_changing = 0;
        m->speed          = clamp_speed(m->new_speed);
        int32_t diff      = m->target_pos - m->current_pos;
        uint32_t rem      = (uint32_t)abs(diff);
        if (rem > 0) motor_pwm_plan_motion(m, rem, m->speed);
    }

    /* 4. 状态机 */
    if (m->state == MOTOR_STA_FINISHED) return;

    if (m->step_count >= m->phase_steps) {
        advance_phase(m);
    } else if (m->state == MOTOR_STA_ACCEL || m->state == MOTOR_STA_DECEL) {
        update_speed_from_v2(m);
    }

    /* 5. 写寄存器 (仅在运动中) */
    if (m->state != MOTOR_STA_STOP && m->state != MOTOR_STA_FINISHED) {
        __HAL_TIM_SET_AUTORELOAD(m->htim, m->arr_value);
        __HAL_TIM_SET_COMPARE(m->htim, m->tim_channel, m->arr_value >> 1);
    }
}

/* ════════════ 查询 ════════════ */

int32_t motor_pwm_get_current_pos(MotorPWM_t *m)
{
    return m->current_pos;
}
MotorState_t motor_pwm_get_state(MotorPWM_t *m)
{
    return m->state;
}
uint8_t motor_pwm_is_running(MotorPWM_t *m)
{
    return (m->state != MOTOR_STA_STOP && m->state != MOTOR_STA_FINISHED) ? 1 : 0;
}