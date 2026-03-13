#include "helper.h"
#include <string.h>

// /* 将 int32 拆为两个 uint16（高字在前 big_end） */
// void int32_to_regs_be(int32_t val, uint16_t regs[2])
// {
//     regs[0] = (uint16_t)((uint32_t)val >> 16);    // 高字
//     regs[1] = (uint16_t)((uint32_t)val & 0xFFFF); // 低字
// }

/* 将 int32 拆为两个 uint16（低字在前）——也就是 word swap */
void int32_to_regs_le(int32_t val, uint16_t regs[2])
{
    regs[0] = (uint16_t)((uint32_t)val & 0xFFFF); // 低字
    regs[1] = (uint16_t)((uint32_t)val >> 16);    // 高字
}

// /* @brief 把 uint32 转为两个寄存器（IEEE-754）, big_endian， 高位在前，低位在后
//  * @param val uint32_t
//  * @param regs uint16_t[2]
//  */
// void uint32_to_regs_be(uint32_t val, uint16_t regs[2])
// {
//     regs[0] = (uint16_t)(val >> 16);
//     regs[1] = (uint16_t)(val & 0xFFFF);
// }
/* @brief 把 uint32 转为两个寄存器（IEEE-754）, low_endian， 低位在前，高位在后
 * @param val uint32_t
 * @param regs uint16_t[2]
 */
void uint32_to_regs_le(uint32_t v, uint16_t regs[2])
{
    regs[0] = (uint16_t)(v & 0xFFFF);         // low word
    regs[1] = (uint16_t)((v >> 16) & 0xFFFF); // high word
}

// /* 把 float 转为两个寄存器（IEEE‑754），常见也有 word swap 情况 */
// void float_to_regs_be(float f, uint16_t regs[2])
// {
//     uint32_t u;
//     memcpy(&u, &f, sizeof(u));        // 避免未定义行为
//     regs[0] = (uint16_t)(u >> 16);    // 高字
//     regs[1] = (uint16_t)(u & 0xFFFF); // 低字
// }

/* @brief 把 uint32 转为两个寄存器（IEEE-754）, low_endian， 低位在前，高位在后
 * @param val uint32_t
 * @param regs uint16_t[2]
 */
void float_to_regs_le(float f, uint16_t regs[2])
{
    uint32_t u;
    memcpy(&u, &f, sizeof(u));                // 避免未定义行为
    regs[0] = (uint16_t)(u & 0xFFFF);         // 低字
    regs[1] = (uint16_t)((u >> 16) & 0xFFFF); // 高字
}

// 1. 将两个16位寄存器转换为int32_t（小端序）
int32_t regs_to_int32_le(const uint16_t regs[2])
{
    // // 小端序：regs[0]是低16位，regs[1]是高16位
    // uint32_t u32 = ((uint32_t)regs[1] << 16) | regs[0];
    // int32_t i32;
    // memcpy(&i32, &u32, sizeof(i32));  // 避免类型双关
    // return i32;

    // 或者简化为：
    return (int32_t)(((uint32_t)regs[1] << 16) | regs[0]);
}

// 2. 将两个16位寄存器转换为uint32_t（小端序）
uint32_t regs_to_uint32_le(const uint16_t regs[2])
{
    // 小端序：regs[0]是低16位，regs[1]是高16位
    return ((uint32_t)regs[1] << 16) | regs[0];
}

// 3. 将两个16位寄存器转换为float（小端序）
float regs_to_float_le(const uint16_t regs[2])
{
    // 小端序：regs[0]是低16位，regs[1]是高16位
    uint32_t u32 = ((uint32_t)regs[1] << 16) | regs[0];
    float f;
    memcpy(&f, &u32, sizeof(f)); // 避免类型双关
    return f;
}

void delay_us(uint32_t us)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;

    uint32_t cycles = us * (SystemCoreClock / 1000000);
    uint32_t start  = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles);
}

uint16_t _calc_crc16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)buf[pos];
        for (int i = 0; i < 8; i++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}