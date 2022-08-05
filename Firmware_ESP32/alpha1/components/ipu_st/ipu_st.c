#include "ipu_st.h"

esp_err_t ipu_init(void)
{
    // uint8_t rcv = 0;
    // i2c_register_read_byte(IPU_I2C_ADDR, IPU_REG_CFG, &rcv);

    return ESP_OK;
}

esp_err_t ipu_read_reg(uint8_t reg, uint8_t *val)
{
    i2c_register_read_byte(IPU_I2C_ADDR, reg, val);
    return ESP_OK;
}

esp_err_t ipu_write_reg(uint8_t reg, uint8_t val)
{
    i2c_register_write_byte(IPU_I2C_ADDR, reg, val);
    return ESP_OK;
}