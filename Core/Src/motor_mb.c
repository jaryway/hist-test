#include "motor_mb.h"
#include "bsp_modbus_master.h"
#include <string.h>
#include <stdio.h>

uint16_t regs[2];
uint32_t timeout_ms = 1000;

static void _motor_mb_setup_pp_mode(MotorMB_t *motor);

/* 将 int32 拆为两个 uint16（高字在前 big_end） */
static void int32_to_regs_be(int32_t val, uint16_t regs[2])
{
    regs[0] = (uint16_t)((uint32_t)val >> 16);    // 高字
    regs[1] = (uint16_t)((uint32_t)val & 0xFFFF); // 低字
}

// /* 将 int32 拆为两个 uint16（低字在前）——也就是 word swap */
// static void int32_to_regs_le(int32_t val, uint16_t regs[2])
// {
//     regs[0] = (uint16_t)((uint32_t)val & 0xFFFF); // 低字
//     regs[1] = (uint16_t)((uint32_t)val >> 16);    // 高字
// }

/* 对于 uint32_t 同理 */
static void uint32_to_regs_be(uint32_t val, uint16_t regs[2])
{
    regs[0] = (uint16_t)(val >> 16);
    regs[1] = (uint16_t)(val & 0xFFFF);
}

// /* 把 float 转为两个寄存器（IEEE‑754），常见也有 word swap 情况 */
// static void float_to_regs_be(float f, uint16_t regs[2])
// {
//     uint32_t u;
//     memcpy(&u, &f, sizeof(u)); // 避免未定义行为
//     regs[0] = (uint16_t)(u >> 16);
//     regs[1] = (uint16_t)(u & 0xFFFF);
// }

static void _motor_mb_setup_pp_mode(MotorMB_t *motor)
{
    MB_Status_t res;

    // 0.先禁用使能

    // 1. 设置控制模式为总线模式（P02-00=9）
    res = modbus_write_single_register(motor->slave_addr, P02_00, 9, timeout_ms);

    if (res != MB_OK) {
        printf("Motor %d: Failed to set control mode\n", motor->slave_addr);
        return;
    }

    // 2. 设置运行模式为PP模式（P10-03=1）
    res = modbus_write_single_register(motor->slave_addr, P10_03, 1, timeout_ms);
    if (res != MB_OK) {
        printf("Motor %d: Failed to set run mode\n", motor->slave_addr);
        return;
    }
}

void motor_mb_init(MotorMB_t *motor, uint8_t slave_addr)
{
    motor->slave_addr = slave_addr;

    // motor->speed = 20000;        // 默认速度
    // motor->acceleration = 5000;  // 默认加速度
    // motor->deceleration = 5000;  // 默认减速度

    // _motor_mb_setup_pp_mode(motor);

    uint16_t read_buffer[10];
    uint8_t quantity = 2;
    MB_Status_t res  = modbus_read_holding_registers(motor->slave_addr, P10_25, quantity, read_buffer, timeout_ms);
    if (res != MB_OK) {
        printf("Motor %d: Failed to read speed\n", motor->slave_addr);
        return;
    }

    printf("Motor %d: Max speed: %d\n", motor->slave_addr, read_buffer[0]);
}

void motor_mb_set_reversed_dir(MotorMB_t *motor)
{
    motor->reversed_dir = !motor->reversed_dir;
}
void motor_mb_set_speed(MotorMB_t *motor, uint32_t speed)
{
    motor->speed = speed;
    uint32_to_regs_be(motor->speed, regs);
    MB_Status_t res = modbus_write_multiple_registers(motor->slave_addr, P10_25, 2, regs, timeout_ms);
    if (res != MB_OK) {
        printf("Motor %d: Failed to set max speed\n", motor->slave_addr);
    }
}
void motor_mb_set_accel(MotorMB_t *motor, uint32_t accel)
{
    motor->accel = accel;
    uint32_to_regs_be(motor->accel, regs);
    modbus_write_multiple_registers(motor->slave_addr, P10_27, 2, regs, timeout_ms);
}
void motor_mb_set_decel(MotorMB_t *motor, uint32_t decel)
{
    motor->decel = decel;
    uint32_to_regs_be(motor->decel, regs);
    modbus_write_multiple_registers(motor->slave_addr, P10_29, 2, regs, timeout_ms);
}

/**
 * @brief 获取当前位置
 */
int32_t motor_mb_get_current_position(MotorMB_t *motor)
{

    uint16_t read_buffer[10];
    uint8_t quantity = 4;
    MB_Status_t res  = modbus_read_holding_registers(motor->slave_addr, P0B_07, quantity, read_buffer, timeout_ms);

    if (res != MB_OK) { // 读取失败
        printf("Motor %d: Failed to read position\n", motor->slave_addr);
        // motor->error_code = 0xFFFF;
        return -1;
    }

    // motor->current_position = pos;
    return (int32_t)(read_buffer[0] << 16) | read_buffer[1];
    // return pos;
}

/**
 * @brief 运动至指定位置
 * @param motor      电机结构体
 * @param abs_position 绝对位置
 */
void motor_mb_move_to(MotorMB_t *motor, int32_t abs_position)
{

    // 1. 设置目标位置（P10-14）,是 int32_t 类型，不能使用06功能码
    int32_to_regs_be(abs_position, regs);
    modbus_write_multiple_registers(motor->slave_addr, P10_14, 2, regs, timeout_ms);

    // 2. 触发绝对定位运动（P0D-08 = 7）
    modbus_write_single_register(motor->slave_addr, P0D_08, 0x0007, timeout_ms);
    motor->target_pos = abs_position;
}
/**
 * @brief 运动至指定位置
 * @param motor      电机结构体
 * @param rel_position 相对对位置
 */
void motor_mb_move(MotorMB_t *motor, int32_t rel_position)
{
    // 获取当前位置
    int32_t current_pos = motor_mb_get_current_position(motor);

    if (current_pos <= 0) { // 读取失败
        return;
    }

    // 2. 计算目标位置
    int32_t abs_position = current_pos + rel_position;
    motor_mb_move_to(motor, abs_position);
}

void motor_mb_check_state(MotorMB_t *motor)
{
    // 读取P10-01（16位无符号整数）
    uint16_t read_buffer[2];
    uint8_t quantity = 2;
    MB_Status_t res  = modbus_read_holding_registers(motor->slave_addr,
                                                     P0B_04,
                                                     quantity,
                                                     read_buffer,
                                                     timeout_ms);
    if (res != MB_OK) { // 读取失败
        printf("Motor %d: Failed to read state\n", motor->slave_addr);
        return;
    }
}

/**
 * @brief 读取相电流有效值 (原始值)
 * @return 相电流值 (单位：0.01A)，读取失败返回-1
 */
float motor_mb_get_phase_current(MotorMB_t *motor)
{
    // 读取P0B-24（16位无符号整数）
    uint16_t read_buffer[2];
    uint8_t quantity = 2;
    MB_Status_t res  = modbus_read_holding_registers(motor->slave_addr,
                                                     P0B_24,
                                                     quantity,
                                                     read_buffer,
                                                     timeout_ms);

    if (res == MB_OK) { // 读取失败
        printf("Failed to read phase current from slave %d\n", motor->slave_addr);
        return -1.0f;
    }

    uint32_t raw_value = ((uint32_t)read_buffer[0] << 16) | read_buffer[1];
    // 根据手册，P0B-24单位是0.01A
    float current = (float)raw_value * 0.01f;

    return current;
}