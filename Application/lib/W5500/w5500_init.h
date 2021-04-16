
#ifndef __W5500_INIT_H__
#define __W5500_INIT_H__

#include "wizchip_conf.h"
#include <ti/drivers/SPI.h>

#ifdef __cplusplus
extern "C" {
#endif

void wizchip_select(void);
void wizchip_deselect(void);
void delay_cnt(volatile unsigned int nCount);
void Net_Conf(wiz_NetInfo netinfo);
void Display_Net_Conf();
void W5500_Init(SPI_Handle& device);

#ifdef __cplusplus
}
#endif

#endif
