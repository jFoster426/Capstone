#ifndef dac_st_h
#define dac_st_h

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_log.h"
#include <math.h>

#define I2S_NUM         (0)
#define I2S_BCK_IO      (GPIO_NUM_18)
#define I2S_SCK_IO      (GPIO_NUM_48)
#define I2S_WS_IO       (GPIO_NUM_21)
#define I2S_DO_IO       (GPIO_NUM_11)

#define DAC_BUF_LEN     (128)

int32_t dac_buffer[8 * DAC_BUF_LEN];

void dac_init(uint32_t sample_rate);

size_t dac_write(double lVal, double rVal);

#endif