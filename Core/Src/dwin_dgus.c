#include "bsp_dwin_dgus.h"
#include "dwin_dgus.h"

uint16_t _regs[DWIN_DGUS_MAX_DATA_LEN];

void dwin_dgus_init(UART_HandleTypeDef *huart)
{
    bsp_dwin_dgus_init(huart);
}
uint16_t dwin_dgus_read_label(uint16_t var_addr)
{
    uint8_t len = 100;
    uint16_t regs[len];
    DD_Status_t res = bsp_dwin_dgus_read_var_regs(var_addr, len, regs, 0, 1000);
    if (res != DD_OK) {
        return regs;
    }

    for (uint8_t i = 0; i < len; i++) {
        // uint16_t a = regs[i];
    }

    return regs;
}