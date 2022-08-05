#ifndef IO_H
#define IO_H

#include <xc.h>

#define IO_HIGH    1
#define IO_LOW     0

void io_init(void);

uint8_t io_checkBtn1Status(void);

uint8_t io_checkBtn2Status(void);

void io_PIC_LED_write(uint8_t state);

void io_VDD_PIC_EN_write(uint8_t state);

void io_VDD_5V_EN_write(uint8_t state);

uint8_t io_checkVDD_3V3_PGOODState(void);

void io_nSS_TX_EN_write(uint8_t state);

void io_VDD_3V3_CE_write(uint8_t state);

#endif