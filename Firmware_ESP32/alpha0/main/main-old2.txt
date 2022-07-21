#include <stdio.h>
#include "esp_log.h"

#include "adc_st.h"
#include "dac_st.h"
#include "i2c_st.h"
#include "ism330_st.h"

void app_main(void)
{
    uint8_t data[2];
    ESP_ERROR_CHECK(i2c_master_init());
    printf("I2C initialized successfully\n");

    ESP_ERROR_CHECK(i2c_register_write_byte(0x6B, 0x18, 0b00000010));

    ESP_ERROR_CHECK(i2c_register_write_byte(0x6B, 0x10, 0b10100000)); // CTRL1_XL
    ESP_ERROR_CHECK(i2c_register_write_byte(0x6B, 0x11, 0b10100100)); // CTRL2_G

    while (1)
    {
        ESP_ERROR_CHECK(i2c_register_read(0x6B, 0x2D, data, 2)); // OUTZ_L_A, OUTZ_H_A
        printf("%d, ", (data[1] << 8) | data[0]);
        vTaskDelay(10 / portTICK_RATE_MS);

        ESP_ERROR_CHECK(i2c_register_read_byte(0x6B, 0x0F, &data[0]));
        printf("%d\n", data[0]);
    }

    ESP_ERROR_CHECK(i2c_driver_delete(0));
    printf("I2C de-initialized successfully\n");
}