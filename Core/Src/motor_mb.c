#include "motor_mb.h"
#include "bsp_modbus_master.h"
#include "helper.h"
#include <string.h>
#include <stdio.h>
#include "debug.h"

#define CHECK_INTERVAL_MS 100

uint16_t regs[2];
uint32_t timeout_ms = 1000;
// static uint8_t last_check_state = 0;
static uint32_t last_check_time = 0;

static void _motor_mb_setup_pp_mode(MotorMB_t *motor);

static void _motor_mb_setup_pp_mode(MotorMB_t *motor)
{
    MB_Status_t res;

    // 0.先禁用使能
    res = modbus_write_single_register(motor->slave_addr, REG_ADDR_EN, 0, timeout_ms);
    // 1. 设置控制模式为总线模式（P02-00=9）
    res = modbus_write_single_register(motor->slave_addr, REG_ADDR_CTRL_MODE, 9, timeout_ms);

    if (res != MB_OK) {
        PRINT_DEBUG("Motor %d: Failed to set control mode\n", motor->slave_addr);
        return;
    }

    // 2. 设置运行模式为PP模式（P10-03=1）
    res = modbus_write_single_register(motor->slave_addr, REG_ADDR_RUN_MODE, 1, timeout_ms);
    if (res != MB_OK) {
        PRINT_DEBUG("Motor %d: Failed to set run mode\n", motor->slave_addr);
        return;
    }
}

void motor_mb_init(MotorMB_t *motor, uint8_t slave_addr)
{
    motor->slave_addr = slave_addr;
    motor->motion_sta = 0;

    // motor->speed = 20000;        // 默认速度
    // motor->acceleration = 5000;  // 默认加速度
    // motor->deceleration = 5000;  // 默认减速度

    // _motor_mb_setup_pp_mode(motor);
}

void motor_mb_set_reversed_dir(MotorMB_t *motor)
{
    motor->reversed_dir = !motor->reversed_dir;
}
void motor_mb_set_speed(MotorMB_t *motor, uint32_t speed)
{
    motor->speed = speed;
    uint32_to_regs_le(motor->speed, regs); // 将一个32位数转换成两个16位寄存器，低位在前，高位在后
    MB_Status_t res = modbus_write_multiple_registers(motor->slave_addr, REG_ADDR_SPEED, 2, regs, timeout_ms);
    if (res != MB_OK) {
        PRINT_DEBUG("Motor %d: Failed to set max speed\n", motor->slave_addr);
    }
}
void motor_mb_set_accel(MotorMB_t *motor, uint32_t accel)
{
    motor->accel = accel;
    uint32_to_regs_le(motor->accel, regs);
    modbus_write_multiple_registers(motor->slave_addr, REG_ADDR_ACCEL, 2, regs, timeout_ms);
}
void motor_mb_set_decel(MotorMB_t *motor, uint32_t decel)
{
    motor->decel = decel;
    uint32_to_regs_le(motor->decel, regs);
    modbus_write_multiple_registers(motor->slave_addr, REG_ADDR_DECEL, 2, regs, timeout_ms);
}

/**
 * @brief 获取当前位置
 */
int32_t motor_mb_get_current_position(MotorMB_t *motor)
{
    uint16_t buf[2];
    uint8_t quantity = 2;
    MB_Status_t res  = modbus_read_holding_registers(motor->slave_addr, REG_ADDR_POS, quantity, buf, timeout_ms);

    if (res != MB_OK) { // 读取失败
        PRINT_DEBUG("Motor %d: Failed to read position\n", motor->slave_addr);
        return 0;
    }

    return (int32_t)(((uint32_t)buf[1] << 16) | buf[0]);
}

void motor_mb_homing(MotorMB_t *motor)
{

    MB_Status_t res;
    // 1.设置运动模式为 HOME 模式 P10-03 = 6
    // 2.设置回零模式             P10-35 = 6
    // 3.设置回零查询速度          P10-36
    // 4.设置回零速度              P10-38
    // 5.设置回零加减速度          P10-40
    // 6.设置总线控制字为回零运动   P0D-08 =128

    uint8_t quantity = 7;
    uint16_t values[7];
    /**
     * 回零模式 6，以高速向负方向回零，遇到原点感应器信号时，低速回零，然后停止
     */

    res = modbus_write_single_register(motor->slave_addr, REG_ADDR_RUN_MODE, 6, timeout_ms);
    if (res != MB_OK) {
        PRINT_DEBUG("Motor %d: Failed to set run mode\n", motor->slave_addr);
        return;
    }

    uint32_t home_search_speed = motor->speed / 2;
    uint32_t home_speed        = motor->speed / 3;
    uint32_t home_decel        = motor->decel / 2;

    values[0] = 6;                                    // 回零模式
    uint32_to_regs_le(home_search_speed, &values[1]); // 搜索速度（2个寄存器）
    uint32_to_regs_le(home_speed, &values[3]);        // 回零速度（2个寄存器）
    uint32_to_regs_le(home_decel, &values[5]);        // 回零减速度（2个寄存器）

    res = modbus_write_multiple_registers(motor->slave_addr, REG_ADDR_HOME_MODE, quantity, values, timeout_ms);

    if (res != MB_OK) {
        PRINT_DEBUG("Motor %d: Failed to set home mode\n", motor->slave_addr);
        return;
    }

    // 设置总线控制字为回零运动P0D-08=128-回零运动）
    res = modbus_write_single_register(motor->slave_addr, REG_ADDR_BUS_CTRL_MODE, 128, timeout_ms);
    if (res != MB_OK) {
        PRINT_DEBUG("Motor %d: Failed to trigger home\n", motor->slave_addr);
        return;
    }
    // 触发回零成功
    PRINT_DEBUG("Motor %d: Trigger home\n", motor->slave_addr);
}

/**
 * @brief 运动至指定位置
 * @param motor      电机结构体
 * @param abs_position 绝对位置
 */
void motor_mb_move_to(MotorMB_t *motor, int32_t abs_position)
{
    MB_Status_t res;
    // 0. 重置总线控制字
    res = modbus_write_single_register(motor->slave_addr, REG_ADDR_BUS_CTRL_MODE, 0x0, timeout_ms);

    // 1. 设置目标位置（P10-14）,是 int32_t 类型，不能使用06功能码
    int32_to_regs_le(abs_position, regs);
    res = modbus_write_multiple_registers(motor->slave_addr, REG_ADDR_TARGET_POS, 2, regs, timeout_ms);
    if (res != MB_OK) {
        PRINT_DEBUG("Motor %d: Failed to set target position\n", motor->slave_addr);
        return;
    }

    // 2. 触发绝对定位运动（P0D-08 = 7）
    res = modbus_write_single_register(motor->slave_addr, REG_ADDR_BUS_CTRL_MODE, 0x0007, timeout_ms);
    if (res != MB_OK) {
        PRINT_DEBUG("Motor %d: Failed to trigger absolute positioning\n", motor->slave_addr);
        return;
    }
    // modbus_write_single_register(motor->slave_addr, REG_ADDR_BUS_CTRL_MODE, 0x0, timeout_ms);
    // 1. 设置目标位置（P10-14）,是 int32_t 类型，不能使用06功能码
    int32_to_regs_le(abs_position, regs);
    modbus_write_multiple_registers(motor->slave_addr, REG_ADDR_TARGET_POS, 2, regs, timeout_ms);
    // PRINT_DEBUG("Motor %d: Move to %ld\n", motor->slave_addr, abs_position);
    PRINT_DEBUG("write target pos \n");

    // 2. 触发绝对定位运动（P0D-08 = 7）
    res = modbus_write_single_register(motor->slave_addr, REG_ADDR_BUS_CTRL_MODE, 0x0007, timeout_ms);
    PRINT_DEBUG("write ctrl mode\n");

    if (res != MB_OK) {
        PRINT_DEBUG("Motor %d: Failed to set bus control mode,res=%d\n", motor->slave_addr, res);
        return;
    }

    motor->motion_sta = 1;
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
    PRINT_DEBUG("Motor %d: Current position: %ld, \n", motor->slave_addr, current_pos);

    // 2. 计算目标位置
    int32_t abs_position = current_pos + rel_position;
    motor_mb_move_to(motor, abs_position);
}

void motor_mb_stop(MotorMB_t *motor)
{
    MB_Status_t res;
    // 0. 重置总线控制字
    res = modbus_write_single_register(motor->slave_addr, REG_ADDR_BUS_CTRL_MODE, 256, timeout_ms);
    if (res != MB_OK) {
        PRINT_DEBUG("Motor %d: Failed to stop\n", motor->slave_addr);
        return;
    }

    motor->motion_sta = 0;
}

void motor_mb_e_stop(MotorMB_t *motor)
{
    MB_Status_t res;
    // 0. 重置总线控制字
    res = modbus_write_single_register(motor->slave_addr, REG_ADDR_BUS_CTRL_MODE, 512, timeout_ms);
    if (res != MB_OK) {
        PRINT_DEBUG("Motor %d: Failed to stop\n", motor->slave_addr);
        return;
    }

    motor->motion_sta = 0;
}

void motor_mb_process(MotorMB_t *motor)
{
    if (motor->motion_sta == 0 || (HAL_GetTick() - last_check_time < CHECK_INTERVAL_MS))
        return;

    // 读取P10-01（16位无符号整数）
    uint16_t regs[1];
    uint8_t quantity = 1;
    MB_Status_t res;

    PRINT_DEBUG("read bus state \n");
    res = modbus_read_holding_registers(motor->slave_addr,
                                        REG_ADDR_BUS_STATE,
                                        quantity,
                                        regs,
                                        timeout_ms);

    if (res != MB_OK) { // 读取失败
        PRINT_DEBUG("Motor %d: Failed to read state\n", motor->slave_addr);
        return;
    }

    // Bit0：位置到达 1 << 0
    // Bit1：速度到达 1 << 1
    // Bit2：转矩到达 1 << 2
    // Bit3：回零完成 1 << 3

    // 返回的数值，第一位是否为1
    last_check_time  = HAL_GetTick();
    uint8_t pos_sta  = (regs[0] & (1 << 0)) != 0; // 位置到达
    uint8_t home_sta = (regs[0] & (1 << 3)) != 0; // 回零完成

    if (pos_sta == 1 || home_sta == 1) {
        motor->motion_sta = 0;
        // 0.重置总线控制字
        modbus_write_single_register(motor->slave_addr, REG_ADDR_BUS_CTRL_MODE, 0x0, timeout_ms);
    }
}

/**
 * @brief 读取相电流有效值 (原始值)
 * @return 相电流值 (单位：0.01A)，读取失败返回-1
 */
float motor_mb_get_phase_current(MotorMB_t *motor)
{
    // 读取P0B-24（16位无符号整数）
    uint16_t regs[2];
    uint8_t quantity = 2;
    // modbus_lock();
    MB_Status_t res = modbus_read_holding_registers(motor->slave_addr,
                                                    REG_ADDR_PHASE_CUR,
                                                    quantity,
                                                    regs,
                                                    timeout_ms);
    // modbus_unlock();

    if (res == MB_OK) { // 读取失败
        PRINT_DEBUG("Failed to read phase current from slave %d\n", motor->slave_addr);
        return -1.0f;
    }

    return regs_to_float_le(regs);

    // uint32_t raw_value = ((uint32_t)buf[0] << 16) | buf[1];
    // // 根据手册，P0B-24单位是0.01A
    // float current = (float)raw_value * 0.01f;

    // return current;
}

uint8_t motor_mb_is_stopped(MotorMB_t *motor)
{
    return (motor->motion_sta == 0);
}