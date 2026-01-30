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
        return;
    }
}
void motor_mb_set_accel(MotorMB_t *motor, uint32_t accel)
{
    motor->accel = accel;
    uint32_to_regs_le(motor->accel, regs);
    MB_Status_t res = modbus_write_multiple_registers(motor->slave_addr, REG_ADDR_ACCEL, 2, regs, timeout_ms);
    if (res != MB_OK) {
        PRINT_DEBUG("Motor %d: Failed to set accel\n", motor->slave_addr);
        return;
    }
}
void motor_mb_set_decel(MotorMB_t *motor, uint32_t decel)
{
    motor->decel = decel;
    uint32_to_regs_le(motor->decel, regs);
    MB_Status_t res = modbus_write_multiple_registers(motor->slave_addr, REG_ADDR_DECEL, 2, regs, timeout_ms);
    if (res != MB_OK) {
        PRINT_DEBUG("Motor %d: Failed to set decel\n", motor->slave_addr);
        return;
    }
}

/**
 * @brief 获取当前位置
 */
int32_t motor_mb_get_current_pos(MotorMB_t *motor, MB_Status_t *res)
{
    uint16_t buf[2];
    uint8_t quantity = 2;

    res = modbus_read_holding_registers(motor->slave_addr, REG_ADDR_POS, quantity, buf, timeout_ms);

    if (res != MB_OK) { // 读取失败
        PRINT_DEBUG("Motor %d: Failed to read position\n", motor->slave_addr);
        return 0;
    }

    return (int32_t)(((uint32_t)buf[1] << 16) | buf[0]);
}

void motor_mb_homing(MotorMB_t *motor)
{
    // 01.P10-03=6 设置运动模式为 HOME 模式
    // 02.P10-35=6 设置回零模式为 负向回原点
    // 03.P10-36=v 设置回零查询速度
    // 04.P10-38=v 设置回零速度
    // 05.P10-40=v 设置回零加减速度
    // 06.P03-07=1 伺服上电使能
    // 07.P0D-08=128 Home 模式启动

    MB_Status_t res;
    uint8_t quantity = 7;
    uint16_t values[7];
    uint16_t value;

    /* 01.P10-03=6 设置运动模式为 HOME 模式 */
    res = modbus_write_single_register(motor->slave_addr, REG_ADDR_RUN_MODE, 6, timeout_ms);
    if (res != MB_OK) {
        PRINT_DEBUG("Motor %d: Failed to set run mode\n", motor->slave_addr);
        return;
    }

    uint32_t home_search_speed = motor->speed / 2;
    uint32_t home_speed        = motor->speed / 3;
    uint32_t home_decel        = motor->decel / 2;
    values[0]                  = 6; // 设置回零模式为 负向回原点

    uint32_to_regs_le(home_search_speed, &values[1]); // 搜索速度（2个寄存器）
    uint32_to_regs_le(home_speed, &values[3]);        // 回零速度（2个寄存器）
    uint32_to_regs_le(home_decel, &values[5]);        // 回零减速度（2个寄存器）

    res = modbus_write_multiple_registers(motor->slave_addr, REG_ADDR_HOME_MODE, quantity, values, timeout_ms);
    if (res != MB_OK) {
        PRINT_DEBUG("Motor %d: Failed to set home mode\n", motor->slave_addr);
        return;
    }

    /* 06.P03-07=1 伺服上电使能 --------------------------------------*/
    value = 1;
    res   = modbus_write_single_register(1, REG_ADDR_EN, value, 100);
    if (res != MB_OK) {
        PRINT_DEBUG("write P03-07=%d error,res:%d\n", value, res);
        return;
    }

    /* 07.P0D-08=128 Home 模式启动 -----------------------------------*/
    value = 128;
    res   = modbus_write_single_register(1, REG_ADDR_BUS_CTRL_MODE, value, 100);
    if (res != MB_OK) {
        PRINT_DEBUG("write P0D-08=%d error,res:%d\n", value, res);
        return;
    }

    // 触发回零成功
    PRINT_DEBUG("Motor %d: Trigger home\n", motor->slave_addr);
}

/**
 * @brief 运动至指定位置
 * @param motor      电机结构体
 * @param abs_pos 绝对位置
 */
void motor_mb_move_to(MotorMB_t *motor, int32_t abs_pos)
{

    // 01.获取当前位置
    MB_Status_t res;

    int32_t cur_pos = motor_mb_get_current_pos(motor, &res);
    if (res != MB_OK) return;

    uint32_t rel_pos = abs_pos - cur_pos;
    return motor_mb_move(motor, rel_pos);
}
/**
 * @brief 运动至指定位置
 * @param motor   电机结构体
 * @param rel_pos 相对对位置
 */
void motor_mb_move(MotorMB_t *motor, int32_t rel_pos)
{
    // 01.P10-03=1 设置运行模式为PP模式
    // 02.P10-14=v 设置目标位置
    // 03.P0D-08=1 相对定位启动
    // 04.P0D-08=0 复位总线控制字

    MB_Status_t res;
    uint16_t value;

    /* 01.P10-03=1 设置运行模式为PP模式 --------------------------------*/
    value = 1;
    res   = modbus_write_single_register(1, REG_ADDR_RUN_MODE, value, 100);
    if (res != MB_OK) {
        PRINT_DEBUG("write P10-03=%d error,res:%d\n", value, res);
        return;
    }

    /* 02.P10-14=v 设置目标位置 ------------------------------------- */
    uint32_to_regs_le(rel_pos, regs);
    res = modbus_write_multiple_registers(1, REG_ADDR_TARGET_POS, 2, regs, 200);
    if (res != MB_OK) {
        PRINT_DEBUG("write P10-14=%lu error,res:%d\n", rel_pos, res);
        return;
    }

    /* 03.P0D-08=1 相对定位启动 --------------------------------------------- */
    value = 1;
    res   = modbus_write_single_register(1, REG_ADDR_BUS_CTRL_MODE, value, 100);
    if (res != MB_OK) {
        PRINT_DEBUG("write P0D-08=%d error,res:%d\n", value, res);
        return;
    }

    /* 04.P0D-08=0 复位总线控制字 --------------------------------------------- */
    value = 0;
    res   = modbus_write_single_register(1, REG_ADDR_BUS_CTRL_MODE, value, 100);
    if (res != MB_OK) {
        PRINT_DEBUG("write P0D-08=%d error,res:%d\n", value, res);
        return;
    }

    motor->motion_sta = 1;
}

void motor_mb_stop(MotorMB_t *motor)
{
    // 01. P0D-08=256 运动停止指令

    MB_Status_t res;

    res = modbus_write_single_register(motor->slave_addr, REG_ADDR_BUS_CTRL_MODE, 256, timeout_ms);
    if (res != MB_OK) {
        PRINT_DEBUG("Motor %d: Failed to stop\n", motor->slave_addr);
        return;
    }

    motor->motion_sta = 0;
}

void motor_mb_e_stop(MotorMB_t *motor)
{
    // 01. P0D-08=512 运动急停指令

    MB_Status_t res;

    res = modbus_write_single_register(motor->slave_addr, REG_ADDR_BUS_CTRL_MODE, 512, timeout_ms);
    if (res != MB_OK) {
        PRINT_DEBUG("Motor %d: Failed to stop\n", motor->slave_addr);
        return;
    }

    motor->motion_sta = 0;
}

void motor_mb_process(MotorMB_t *motor)
{
    // 01.P0B-00 监控转速单位rpm
    // 02.P0B-02 监控负载率百分比单位
    // 03.P0B-07 监控反馈位置
    // 04.P0B-24 监控相电流
    // 05.P0B-04 监控状态字

    if (motor->motion_sta == 0 || (HAL_GetTick() - last_check_time < CHECK_INTERVAL_MS))
        return;

    MB_Status_t res;

    /* 01.P0B-00 监控转速单位rpm ------------------------------------*/
    res = modbus_read_holding_registers(1, REG_ADDR_RPM, 1, regs, 100);
    if (res != MB_OK) {
        PRINT_DEBUG("read P0B-00 error,res:%d\n", res);
    } else {
        motor->m_rpm = (int16_t)regs[0];
    }

    /* 02.P0B-02 监控负载率百分比单位 --------------------------------*/
    res = modbus_read_holding_registers(1, REG_ADDR_LOAD_RATE, 1, regs, 100);
    if (res != MB_OK) {
        PRINT_DEBUG("read P0B-02 error,res:%d\n", res);
    } else {
        motor->m_load_rate = (int16_t)regs[0];
    }

    /* 03.P0B-07 监控反馈位置 -----------------------------------------*/
    res = modbus_read_holding_registers(1, REG_ADDR_POS, 2, regs, 100);
    if (res != MB_OK) {
        PRINT_DEBUG("read P0B-07 error,res:%d\n", res);
    } else {
        motor->m_pos = regs_to_int32_le(regs);
    }

    /* 04.P0B-24 监控相电流 -----------------------------------------*/
    res = modbus_read_holding_registers(1, REG_ADDR_PHASE_CUR, 2, regs, 100);
    if (res != MB_OK) {
        PRINT_DEBUG("read P0B-24 error,res:%d\n", res);
    } else {
        motor->m_phase_cur = regs_to_float_le(regs);
    }

    /* 05.P0B-04 监控状态字 -------------------------------------------*/
    // Bit0：位置到达
    // Bit1：速度到达
    // Bit2：转矩到达
    // Bit3：回零完成
    res = modbus_read_holding_registers(1, REG_ADDR_BUS_STATE, 2, regs, 100);
    if (res != MB_OK) {
        PRINT_DEBUG("read P0B-04 error,res:%d\n", res);
        return;
    }

    motor->m_pos_sta    = (regs[0] & (1 << 0)) != 0; // 位置到达
    motor->m_homing_sta = (regs[0] & (1 << 3)) != 0; // 回零完成

    if (motor->m_pos_sta == 1 || motor->m_homing_sta == 1) {
        motor->motion_sta = 0;
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

    MB_Status_t res = modbus_read_holding_registers(motor->slave_addr,
                                                    REG_ADDR_PHASE_CUR,
                                                    quantity,
                                                    regs,
                                                    timeout_ms);

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