#ifndef rtc_st_h
#define rtc_st_h

#include <stdio.h>
#include "esp_log.h"

#include "../../i2c_st/include/i2c_st.h"

#define PCF85063_I2C_ADDR                           (0x51)

// PCF85063 register definitions.
#define PCF85063_REG_CONTROL_1                      (0x00)
#define PCF85063_REG_CONTROL_2                      (0x01)
#define PCF85063_REG_OFFSET                         (0x02)
#define PCF85063_REG_RAM_BYTE                       (0x03)
#define PCF85063_REG_SECONDS                        (0x04)
#define PCF85063_REG_MINUTES                        (0x05)
#define PCF85063_REG_HOURS                          (0x06)
#define PCF85063_REG_DAYS                           (0x07)
#define PCF85063_REG_WEEKDAYS                       (0x08)
#define PCF85063_REG_MONTHS                         (0x09)
#define PCF85063_REG_YEARS                          (0x0A)
#define PCF85063_REG_SECOND_ALARM                   (0x0B)
#define PCF85063_REG_MINUTE_ALARM                   (0x0C)
#define PCF85063_REG_HOUR_ALARM                     (0x0D)
#define PCF85063_REG_DAY_ALARM                      (0x0E)
#define PCF85063_REG_WEEKDAY_ALARM                  (0x0F)
#define PCF85063_REG_TIMER_VALUE                    (0x10)
#define PCF85063_REG_TIMER_MODE                     (0x11)



void rtc_st_init(void);
uint8_t rtc_st_get_seconds(void);
uint8_t rtc_st_get_minutes(void);
uint8_t rtc_st_get_hours(void);


#endif