#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"

#define REG_ADDR_CTRL_MODE     0x0200 // P02_00 控制模式
#define REG_ADDR_POS           0x0B07 // P0B_07 绝对位置计数器 int32_t
#define REG_ADDR_BUS_STATE     0x0B04 // P0B_04 总线状态字
#define REG_ADDR_PHASE_CUR     0x0B18 // P0B_24 相电流有效值 float
#define REG_ADDR_BUS_CTRL_MODE 0x0D08 // P0D-08 总线控制字
#define REG_ADDR_RUN_MODE      0x1003 // 运行模式
#define REG_ADDR_TARGET_POS    0x100E // P10_14 PP 模式目标位置 int32_t
#define REG_ADDR_MAX_SPEED     0x1017 // P10_23 最大速度限制 int32_t
#define REG_ADDR_SPEED         0x1019 // P10_25 模式速度  uint32_t
#define REG_ADDR_ACCEL         0x101B // P10_27 PP 模式加速度  uint32_t
#define REG_ADDR_DECEL         0x101D // P10_29 PP 模式减速度  uint32_t
#define REG_ADDR_HOME_MODE     0x1023 // P10_35 回零模式，支持 1~14、17~35

#define SPR                    800                   /* 旋转一圈需要的脉冲数 */
#define RPM                    3000                  // 电机最高转速
#define TOTAL_STEPS            1650 / 125 * 12 * SPR // 总步数= 导轨行程/同步轮周长*减速比*每圈脉冲数
#define SPEED                  RPM / 60 * SPR        // 电机速度
#define ACCEL                  (SPEED / 1.0f)        // 加速度 公式 a=v/t
#define DECEL                  (SPEED / 0.8f)        // 减速度 公式 a=v/t

typedef struct
{
    // __IO uint8_t motion_sta; /* 是否在运动？0：停止，1：运动 */
    // __IO uint8_t run_state;  /* 电机旋转状态 */
    // __IO uint8_t dir;        /* 电机旋转方向 */

    // __IO int32_t step_delay;   /* 下个脉冲周期（时间间隔），启动时为加速度 */
    // __IO uint32_t decel_start; /* 开始减速位置 */
    // __IO int32_t decel_val;    /* 减速阶段步数 */
    // __IO int32_t min_delay;    /* 速度最快，计数值最小的值(最大速度，即匀速段速度) */
    // __IO int32_t accel_count;  /* 加减速阶段计数值 */
    // __IO int32_t current_pos;  /* 当前位置 steps */
    // // __IO int32_t target_position; /* 目标位置 */

    // // __IO uint32_t add_pulse_count;  /* 脉冲个数累计 */
    // __IO uint16_t last_accel_delay; /* 加速过程中最后一次延时（脉冲周期） */
    // __IO uint32_t step_count;       /* 总移动步数计数器*/
    // __IO int32_t rest;              /* 记录new_step_delay中的余数，提高下一步计算的精度 */

    // int32_t pulses; /* 带方向的目标移动总步数 */
    uint8_t reversed_dir; // 是否需要反转方向
    uint32_t accel;
    uint32_t decel;
    uint32_t speed;

    uint32_t target_pos; // 目标位置
    uint8_t slave_addr;  // 从机地址

} MotorMB_t;

void motor_mb_init(MotorMB_t *motor, uint8_t slave_addr);
void motor_mb_set_reversed_dir(MotorMB_t *motor);
void motor_mb_set_speed(MotorMB_t *motor, uint32_t speed);
void motor_mb_set_accel(MotorMB_t *motor, uint32_t accel);
void motor_mb_set_decel(MotorMB_t *motor, uint32_t decel);
int32_t motor_mb_get_current_position(MotorMB_t *motor);
void motor_mb_move_to(MotorMB_t *motor, int32_t abs_position);
void motor_mb_move(MotorMB_t *motor, int32_t rel_position);
/* 检查电机是否运动到位 */
uint8_t motor_mb_check_state(MotorMB_t *motor);
/* 读取相电流有效值 (原始值) */
float motor_mb_get_phase_current(MotorMB_t *motor);
