#include "i2c_st.h"

static esp_err_t i2c_register_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len)
{
#ifdef DEBUG_I2C
    ESP_LOGI("i2c.c", "i2c_register_read(dev_addr = %d, reg_addr = %d)", dev_addr, reg_addr);
    printf("data --> ");
    for (int i = 0; i < len; i++) printf("0x%02X, ", data[i]);
    printf("\n");
#endif
    esp_err_t ret = i2c_master_write_read_device(0, dev_addr, &reg_addr, 1, data, len, 1000 / portTICK_PERIOD_MS);
#ifdef DEBUG_I2C
    if (ret != ESP_OK) ESP_LOGE("i2c.c", "i2c_register_read() failed (reason: %s)", esp_err_to_name(ret));
    else               printf("return --> %s\n", esp_err_to_name(ret));
#endif
    return ret;
}

static esp_err_t i2c_register_read_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data)
{
    return i2c_register_read(dev_addr, reg_addr, data, 1);
}

static esp_err_t i2c_register_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len)
{
    int ret;
    uint8_t *write_buf = malloc(len + 1);

    if (write_buf == NULL)
    {
#ifdef DEBUG_I2C
        ESP_LOGE("i2c.c", "i2c_register_write() malloc failed.");
#endif
        return 1; // TODO: Find the appropriate esp_err_t return value for this case.
    }
#ifdef DEBUG_I2C
    ESP_LOGI("i2c.c", "i2c_register_write(dev_addr = %d, reg_addr = %d)", dev_addr, reg_addr);
    printf("data --> ");
    for (int i = 0; i < len; i++) printf("0x%02X, ", data[i]);
    printf("\n");
#endif
    write_buf[0] = reg_addr;
    memcpy(&write_buf[1], data, len);

    ret = i2c_master_write_to_device(0, dev_addr, write_buf, len + 1, 1000 / portTICK_PERIOD_MS);
#ifdef DEBUG_I2C
    if (ret != ESP_OK) ESP_LOGE("i2c.c", "i2c_register_write() failed (reason: %s)", esp_err_to_name(ret));
    else               printf("return --> %s\n", esp_err_to_name(ret));
#endif
    free(write_buf);
    return ret;
}

static esp_err_t i2c_register_write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
    return i2c_register_write(dev_addr, reg_addr, &data, 1);
}

static esp_err_t i2c_master_init(int gpio_scl, int gpio_sda, int clk_freq)
{
#ifdef DEBUG
    ESP_LOGI("i2c.c", "i2c_master_init(gpio_scl = %d, gpio_sda = %d, clk_freq = %d)", gpio_scl, gpio_sda, clk_freq);
#endif
    i2c_config_t conf =
    {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = gpio_scl, // 36
        .scl_io_num = gpio_sda, // 35
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = clk_freq, // 400000
    };
    i2c_param_config(0, &conf);

    esp_err_t ret = i2c_driver_install(0, conf.mode, 0, 0, 0);
#ifdef DEBUG
    if (ret != ESP_OK) ESP_LOGE("i2c.c", "i2c driver install failed (reason: %s)", esp_err_to_name(ret));
    else               ESP_LOGI("i2c.c", "i2c driver install success (return: %s)", esp_err_to_name(ret));
#endif
    return ret;
}