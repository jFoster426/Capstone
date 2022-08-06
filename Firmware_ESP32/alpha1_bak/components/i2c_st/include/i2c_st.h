#ifndef i2c_st_h
#define i2c_st_h

#include <stdio.h>
#include <string.h>
#include "driver/i2c.h"

esp_err_t i2c_register_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len);

esp_err_t i2c_register_read_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data);

esp_err_t i2c_register_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len);

esp_err_t i2c_register_write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);

esp_err_t i2c_master_init(int gpio_scl, int gpio_sda, int clk_freq);

#endif