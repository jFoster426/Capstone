#include "rtc_st.h"

void rtc_st_init(void)
{
    // Perform software reset.
    uint8_t data = 0b00010000;
    i2c_register_write(PCF85063_I2C_ADDR, PCF85063_REG_CONTROL_1, &data, 1);
}

uint8_t rtc_st_get_seconds(void)
{
    uint8_t raw;
    i2c_register_read(PCF85063_I2C_ADDR, PCF85063_REG_SECONDS, &raw, 1);
    uint8_t ones = raw & 0xF;
    uint8_t tens = (raw & 0x70) >> 4;
    uint8_t ret = (tens * 10) + ones;
    return ret;
}

uint8_t rtc_st_get_minutes(void)
{
    uint8_t raw;
    i2c_register_read(PCF85063_I2C_ADDR, PCF85063_REG_MINUTES, &raw, 1);
    uint8_t ones = raw & 0xF;
    uint8_t tens = (raw & 0x70) >> 4;
    uint8_t ret = (tens * 10) + ones;
    return ret;
}

uint8_t rtc_st_get_hours(void)
{
    uint8_t raw;
    i2c_register_read(PCF85063_I2C_ADDR, PCF85063_REG_HOURS, &raw, 1);
    uint8_t ones = raw & 0xF;
    uint8_t tens = (raw & 0x30) >> 4;
    uint8_t ret = (tens * 10) + ones;
    return ret;
}