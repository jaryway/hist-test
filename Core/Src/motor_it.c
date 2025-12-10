
#include "stm32f1xx_hal.h"
#include "motor_it.h"
#include <stdio.h>

static void Motor_set_dir(Motor_t *motor)
{
    printf("motor->dir=%d\r\n", motor->dir);
    if (motor->dir)
    {
        GPIO_PinState pin_state = (GPIO_PinState)(motor->reversed_dir ^ GPIO_PIN_SET);
        printf("motor->dir1=%d\r\n", pin_state);
        HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, pin_state);
    }
    else
    {
        GPIO_PinState pin_state = (GPIO_PinState)(motor->reversed_dir ^ GPIO_PIN_RESET);
        printf("motor->dir2=%d\r\n", pin_state);
        HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, pin_state);
    }
}

static float generate_trapezoid_freq(Motor_t *motor)
{
    // 运动参数
    uint32_t total_pulses = motor->total_pulses;
    uint32_t accel_pulses = motor->accel_pulses;
    uint32_t decel_pulses = motor->decel_pulses;
    uint32_t cruise_pulses = total_pulses - accel_pulses - decel_pulses;
    uint32_t pulse_index = motor->pulse_count;

    // 频率参数（单位：Hz）
    float start_freq = 1000.0f;                 // 起始频率 1KHz
    float max_freq = 3000.0f / 60.0f * 5000.0f; // 最大频率 = 250KHz
    float end_freq = 1000.0f;                   // 结束频率 1KHz

    // 验证参数
    if (max_freq > 250000.0f)
        max_freq = 250000.0f; // 限制最大频率
    if (start_freq > max_freq)
        start_freq = max_freq;
    if (end_freq > max_freq)
        end_freq = max_freq;

    float current_freq;

    if (pulse_index < accel_pulses)
    {
        // 加速段：线性加速
        float ratio = (float)pulse_index / accel_pulses;
        current_freq = start_freq + (max_freq - start_freq) * ratio; // 使用线性插值
        motor->state = MOTOR_ACCEL;
    }
    else if (pulse_index < accel_pulses + cruise_pulses)
    {
        // 匀速段
        current_freq = max_freq;
        motor->state = MOTOR_RUN;
    }
    else if (pulse_index < total_pulses)
    {
        // 减速段：线性减速
        uint32_t decel_start = accel_pulses + cruise_pulses;
        float ratio = (float)(pulse_index - decel_start) / decel_pulses;
        current_freq = max_freq - (max_freq - end_freq) * ratio; // 从最大频率减速到结束频率
        motor->state = MOTOR_DECEL;
    }

    return current_freq;
}

void Motor_init(Motor_t *motor, uint8_t use_en)
{
    motor->use_en = use_en;
    motor->timer_clock_hz = 72000000U; // 获取定时器时钟频率，假设使用APB1定时器
}
void Motor_attach(Motor_t *motor, GPIO_TypeDef *en_port, uint16_t en_pin, GPIO_TypeDef *step_port, uint16_t step_pin, GPIO_TypeDef *dir_port, uint16_t dir_pin)
{
    motor->en_port = en_port;
    motor->en_pin = en_pin;
    motor->has_en_pin = 1;
    Motor_attach_basic(motor, step_port, step_pin, dir_port, dir_pin);
}
void Motor_attach_basic(Motor_t *motor, GPIO_TypeDef *step_port, uint16_t step_pin, GPIO_TypeDef *dir_port, uint16_t dir_pin)
{
    motor->step_port = step_port;
    motor->step_pin = step_pin;
    motor->dir_port = dir_port;
    motor->dir_pin = dir_pin;
}
void Motor_attach_timer(Motor_t *motor, TIM_HandleTypeDef *htim, uint32_t tim_channel, TIM_HandleTypeDef *monitor_htim)
{
    motor->htim = htim;
    motor->tim_channel = tim_channel;
    motor->monitor_htim = monitor_htim;
}
void Motor_set_steps_per_mm(Motor_t *motor, uint16_t pulses_per_mm)
{
    motor->pulses_per_mm = pulses_per_mm;
}
void Motor_set_reversed_dir(Motor_t *motor)
{
    motor->reversed_dir = !motor->reversed_dir;
}

void Motor_move(Motor_t *motor, int32_t total_pulses, uint32_t accel_pulses, uint32_t decel_pulses, uint32_t max_rpm)
{
    if (total_pulses == 0)
    {
        return;
    }
    else if (total_pulses < 0) // 逆时针
    {
        motor->dir = MOTOR_DIR_CCW;
        total_pulses = -total_pulses;
    }
    else // 顺时针
    {
        motor->dir = MOTOR_DIR_CW;
    }
    // 输出电机方向
    Motor_set_dir(motor);

    motor->total_pulses = total_pulses;
    motor->accel_pulses = accel_pulses;
    motor->decel_pulses = decel_pulses;
    motor->max_rpm = max_rpm;
    motor->state = MOTOR_ACCEL;
    motor->pulse_count = 0;

    HAL_TIM_Base_Start_IT(motor->monitor_htim);
    HAL_TIM_Base_Start_IT(motor->htim);
    HAL_TIM_PWM_Start(motor->htim, motor->tim_channel);
}
void Motor_stop(Motor_t *motor)
{
    HAL_TIM_PWM_Stop(motor->htim, motor->tim_channel);
    HAL_TIM_Base_Stop_IT(motor->monitor_htim);
    HAL_TIM_Base_Stop_IT(motor->htim);
    motor->state = MOTOR_STOP;
}
void Motor_count_pusle_in_it(Motor_t *motor)
{
    if (motor->pulse_count >= motor->total_pulses)
    {
        Motor_stop(motor);
        return;
    }
    motor->pulse_count++;
}
uint8_t Motor_update_pwm_freq_in_it(Motor_t *motor)
{
    float freq_hz = generate_trapezoid_freq(motor);
    uint32_t timer_clock_hz = motor->timer_clock_hz;

    if (freq_hz <= 0.0f || timer_clock_hz == 0)
        return 1;

    /* 期望的总 ticks（PSC+1)*(ARR+1) */
    float ticks_f = (float)timer_clock_hz / freq_hz;
    if (ticks_f < 1.0f)
        ticks_f = 1.0f;

    uint32_t ticks = (uint32_t)(ticks_f + 0.5f); /* 四舍五入 */

    /* 计算 PSC，确保 ARR <= 0xFFFF */
    uint32_t psc = (ticks - 1) / 65536u; /* floor((ticks-1)/65536) */
    if (psc > 0xFFFFu)
        return 2; /* 无法表示，频率太低导致需要过大的 PSC */

    /* 计算 ARR */
    uint32_t arr = ticks / (psc + 1u);
    if (arr == 0)
        arr = 1;
    if (arr > 0xFFFFu)
        arr = 0xFFFFu;
    arr = arr - 1u;

    uint32_t ccr = (arr + 1u) / 2u; /* 50% 占空 */

    /* 安全更新寄存器：先停止计时器（或禁用输出），更新 PSC/ARR/CCR，再发起更新事件 */
    __HAL_TIM_DISABLE(&htim3);

    /* 直接写寄存器 PSC（HAL 没有宏），再用 HAL 宏设置 ARR/CCR */
    htim3.Instance->PSC = (uint16_t)psc;
    __HAL_TIM_SET_AUTORELOAD(&htim3, arr);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->tim_channel, ccr);

    /* 产生更新事件，确保 PSC/ARR 生效（UG） */
    htim3.Instance->EGR = TIM_EGR_UG;

    __HAL_TIM_ENABLE(&htim3);

    return 0;
}
uint8_t Motor_is_running(Motor_t *motor)
{
    return motor->state != MOTOR_STOP;
}