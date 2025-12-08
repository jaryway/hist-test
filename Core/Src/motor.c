// // motor_driver.h
// #ifndef __DMA_DUOBLEBUFFER_H
// #define __DMA_DUOBLEBUFFER_H

// #include "stm32f1xx_hal.h"

// // ========== 配置参数 ==========
// #define PULSE_BUFFER_SIZE 256  // 每个缓冲区256个脉冲
// #define MIN_CCR_VALUE 10       // 最小CCR值（对应最大频率）
// #define MAX_CCR_VALUE 60000    // 最大CCR值（对应最小频率）
// #define SYS_CLK_HZ 72000000.0f // 系统时钟72MHz
// #define MIN_START_FREQ 100     // 最小启动频率100Hz

// // ========== 数据结构 ==========
// typedef enum
// {
//     // MOTOR_STATE_IDLE = 0, // 空闲
//     // MOTOR_STATE_READY,    // 已初始化
//     // MOTOR_STATE_RUNNING,  // 运行中
//     MOTOR_STATE_ACCEL,    // 加速中
//     MOTOR_STATE_CRUISE,   // 匀速中
//     MOTOR_STATE_DECEL,    // 减速中
//     MOTOR_STATE_COMPLETE, // 完成
//     MOTOR_STATE_ERROR     // 错误
// } MotorState_t;

// typedef enum m_direction_t
// {
//     M_DIR_CCW = 0, // 反转
//     M_DIR_CW = 1   // 正转
// } m_direction_t;   // 电机旋转方向定义

// typedef struct
// {
//     GPIO_TypeDef *en_port;
//     GPIO_TypeDef *step_port;
//     GPIO_TypeDef *dir_port;

//     uint16_t en_pin;
//     uint16_t step_pin;
//     uint16_t dir_pin;

//     m_direction_t dir;
//     uint8_t reversed_dir; // 是否需要反转方向
//     uint16_t pulses_per_mm;
//     // 运动参数
//     uint32_t total_pulses;  // 总脉冲数
//     uint32_t accel_pulses;  // 加速段脉冲数
//     uint32_t cruise_pulses; // 匀速段脉冲数
//     uint32_t decel_pulses;  // 减速段脉冲数

//     // 速度参数（单位：脉冲/秒）
//     uint32_t start_speed;  // 起始速度
//     uint32_t target_speed; // 目标速度
//     uint32_t acceleration; // 加速度（脉冲/秒²）

//     // CCR值
//     uint16_t start_ccr;  // 起始CCR
//     uint16_t target_ccr; // 目标CCR
//     uint16_t end_ccr;    // 结束CCR

//     // 运行状态
//     uint32_t current_pulse; // 当前脉冲索引
//     MotorState_t state;     // 当前状态

//     // DMA缓冲区
//     uint32_t *dma_buf0;       // DMA缓冲区0
//     uint32_t *dma_buf1;       // DMA缓冲区1
//     uint8_t active_buffer;    // 当前活动缓冲区（0或1）
//     uint8_t need_fill_buffer; // 需要填充的缓冲区标识（0或1）

//     // 性能统计
//     uint32_t max_calc_time_us; // 最大计算时间
//     uint32_t last_fill_pulse;  // 上次填充的脉冲数

//     // 硬件相关
//     TIM_HandleTypeDef *htim; // 定时器句柄
//     TIM_TypeDef *tim;        // 定时器
//     uint32_t tim_channel;    // 定时器通道
// } Motor_t;

// #endif /* __DMA_DUOBLEBUFFER_H */