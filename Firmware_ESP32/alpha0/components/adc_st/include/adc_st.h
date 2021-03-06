#ifndef adc_st_h
#define adc_st_h

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "sdkconfig.h"
#include "esp_log.h"

// These pins define the IO pins used for the SPI bus.
// CS pin is uniquely defined for each function.
#define SPI_PIN_NUM_MISO (10)
#define SPI_PIN_NUM_MOSI (11)
#define SPI_PIN_NUM_CLK  (12)

void spi_transfer(spi_device_handle_t *adc, const uint8_t *tx_data, uint8_t *rx_data, const uint8_t data_len);

void adc_init(spi_device_handle_t *adc, uint8_t cs_pin);

uint32_t adc_read_raw(spi_device_handle_t *adc, uint8_t cs_pin);

#endif