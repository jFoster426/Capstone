#include "i2c_st.h"

esp_err_t i2c_register_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len)
{
    esp_err_t ret = i2c_master_write_read_device(0, dev_addr, &reg_addr, 1, data, len, 1000 / portTICK_PERIOD_MS);
    return ret;
}

esp_err_t i2c_register_read_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data)
{
    return i2c_register_read(dev_addr, reg_addr, data, 1);
}

esp_err_t i2c_register_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len)
{
    int ret;
    uint8_t *write_buf = malloc(len + 1);

    if (write_buf == NULL)
        return 1; // TODO: Find the appropriate esp_err_t return value for this case.

    write_buf[0] = reg_addr;
    memcpy(&write_buf[1], data, len);

    ret = i2c_master_write_to_device(0, dev_addr, write_buf, len + 1, 1000 / portTICK_PERIOD_MS);

    free(write_buf);
    return ret;
}

esp_err_t i2c_register_write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
    return i2c_register_write(dev_addr, reg_addr, &data, 1);
}

esp_err_t i2c_master_init(int gpio_scl, int gpio_sda, int clk_freq)
{
    i2c_config_t conf =
    {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = gpio_sda,
        .scl_io_num = gpio_scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = clk_freq,
    };
    i2c_param_config(0, &conf);

    esp_err_t ret = i2c_driver_install(0, conf.mode, 0, 0, 0);

    return ret;
}