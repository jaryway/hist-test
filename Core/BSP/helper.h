
#ifndef __HELPER_H
#define __HELPER_H

#ifdef __cplusplus
extern "C" {
#endif

// #include "stm32f1xx_hal.h"
#include "stm32f1xx.h"

// /* 将 int32 拆为两个 uint16（高字在前 big_end） */
// void int32_to_regs_be(int32_t val, uint16_t regs[2]);

/* 将 int32 拆为两个 uint16（低字在前）——也就是 word swap */
void int32_to_regs_le(int32_t val, uint16_t regs[2]);

// /* 对于 uint32_t 同理 */
// void uint32_to_regs_be(uint32_t val, uint16_t regs[2]);
void uint32_to_regs_le(uint32_t val, uint16_t regs[2]);

/* 把 float 转为两个寄存器（IEEE‑754），低位在前，高位在后 */
void float_to_regs_le(float f, uint16_t regs[2]);

int32_t regs_to_int32_le(const uint16_t regs[2]);
uint32_t regs_to_uint32_le(const uint16_t regs[2]);
float regs_to_float_le(const uint16_t regs[2]);
void delay_us(uint32_t us);

#ifdef __cplusplus
}
#endif

#endif /* __HELPER_H */
