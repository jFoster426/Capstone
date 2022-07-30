#include <stdio.h>
#include "esp_log.h"

#include "adc_st.h"
#include "bt_st.h"
#include "dac_st.h"
#include "i2c_st.h"
#include "ism330_st.h"
#include "rtc_st.h"

#include "dataset.h"

#include "esp_system.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_fat.h"
#include "driver/uart.h"

/* IMU CHECK

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init(36, 37, 400000));
    printf("I2C initialized successfully\n");

    ism330_init();

    while (1)
    {
        float accx = ism330_get_acc_x_g();
        float accy = ism330_get_acc_y_g();
        float accz = ism330_get_acc_z_g();
        float gyrx = ism330_get_gyr_x_dps();
        float gyry = ism330_get_gyr_y_dps();
        float gyrz = ism330_get_gyr_z_dps();

        float pitch = accx;
        float roll = accy;

        pitch *= 364;
        roll *= 364;

        int16_t pitch16 = (int16_t)pitch;
        int16_t roll16 = (int16_t)roll;

        // float temp = ism330_get_temp_celcius();
        
        printf("%d, %d, %d, 0,\n", xTaskGetTickCount(), pitch16, roll16);




        vTaskDelay(10 / portTICK_RATE_MS);
    }

    ESP_ERROR_CHECK(i2c_driver_delete(0));
    printf("I2C de-initialized successfully\n");
}

*/

/* BLUETOOTH CHECK

void app_main(void)
{
    // Initialize the bluetooth module.
    struct bt_config_t bt = 
    {
        .device_name = "STRASAnkle Gamma"
    };
    bt_init(&bt);

    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
    ESP_ERROR_CHECK(uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 256, 0, 0, NULL, 0));
    esp_vfs_dev_uart_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
    esp_vfs_dev_uart_port_set_rx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CR);
    esp_vfs_dev_uart_port_set_tx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CRLF);
    char buf[10];

    while (1)
    {
        scanf("%1s", buf);

        printf("received: %s\n", buf);

        if (bt.connected == true && bt.write_available == true && buf[0] == '2')
        {
            uint8_t bigFile[512];

            while (bt.write_available == false);
            strcpy((char *)bigFile, "testFile,w");
            bt_notify(bigFile, 10);

            while (bt.write_available == false);
            strcpy((char *)bigFile, "testFile");
            bt_write(bigFile, 8);

            size_t sz = sizeof(sample_dataset) / sizeof(uint16_t);

            printf("total size = %d\n", sz);

            size_t curPos = 0;

            while (curPos < sz - 256)
            {
                while (bt.write_available == false);
                printf("bt_write returns: %d\n", bt_write((uint8_t *)&sample_dataset[curPos], 512));
                curPos += 256;
            }

            size_t rem = sz - curPos;
            while (bt.write_available == false);
            printf("bt_write returns: %d\n", bt_write((uint8_t *)&sample_dataset[curPos], rem * 2));

            // EOF character.
            strcpy((char *)bigFile, "EOF");
            while (bt.write_available == false);
            bt_write((uint8_t *)bigFile, 3);
        }

        if (bt.connected == true && bt.write_available == true && buf[0] == '1')
        {
            uint8_t case1[11];

            while (bt.write_available == false);
            strcpy((char *)case1, "statFile,w");
            bt_notify(case1, 10);

            while (bt.write_available == false);
            strcpy((char *)case1, "statFile");
            bt_write(case1, 8);

            case1[0] = 63;
            case1[1] = 0;
            case1[2] = 0;
            case1[3] = 1;
            case1[4] = 1;
            case1[5] = 0;
            while (bt.write_available == false);
            bt_write(case1, 6);

            strcpy((char *)case1, "EOF");
            while (bt.write_available == false);
            bt_write((uint8_t *)case1, 3);
        }

        if (bt.connected == true && bt.write_available == true && buf[0] == '0')
        {
            uint8_t case2[11];

            while (bt.write_available == false);
            strcpy((char *)case2, "statFile,w");
            bt_notify(case2, 10);

            while (bt.write_available == false);
            strcpy((char *)case2, "statFile");
            bt_write(case2, 9);

            case2[0] = 63;
            case2[1] = 0;
            case2[2] = 0;
            case2[3] = 1;
            case2[4] = 0;
            case2[5] = 0;
            while (bt.write_available == false);
            bt_write(case2, 6);

            strcpy((char *)case2, "EOF");
            while (bt.write_available == false);
            bt_write((uint8_t *)case2, 3);
        }
        // if (bt.connected == true && bt.read_available == true)
        // {
        //     bt_read(data, len);
        //     printf("read characteristic: %02X\n", data[0]);
        // }
        // if (bt.notify_enabled == true)
        // {
        //     char data[] = "notify test.";
        //     bt_notify((uint8_t *)data, 12);
        // }
        // vTaskDelay(100 / portTICK_RATE_MS);
        // vTaskDelay(1 / portTICK_RATE_MS);
    }

}

*/

/* LOAD CELL CHECK

void app_main(void)
{
    // Initialize ADC.
    spi_device_handle_t adc;
    adc_init(&adc, 7);
    printf("ADC initialized.\n");

    // Initialize DAC. Use a high sampling rate for now to avoid noise on the output (how to avoid this?).
    dac_init(22050);
    printf("DAC initialized.\n");

    // Enable 5VA.
    gpio_config_t nVDD_5VA_EN = 
    {
        .pin_bit_mask = GPIO_NUM_35,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&nVDD_5VA_EN);
    gpio_set_direction(GPIO_NUM_35, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_35, 1);

    // ESP32 LED.
    gpio_config_t ESP32_LED = 
    {
        .pin_bit_mask = GPIO_NUM_9,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&ESP32_LED);
    gpio_set_direction(GPIO_NUM_9, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_9, 0);

    uint32_t val;

    float fl = 1.0;

    while (1)
    {
        val = adc_read_raw(&adc, 7);
        gpio_set_level(GPIO_NUM_9, 1);
        printf("%08X\n", val);
        dac_write(fl, fl);
        vTaskDelay(100 / portTICK_RATE_MS);

        fl -= 0.01;
        if (fl < -1.0) fl = 1.0;
    }
}

*/

// /* CHARGER CHECK

void app_main(void)
{
    // Enable 5VA.
    gpio_config_t nCHG_EN = 
    {
        .pin_bit_mask = GPIO_NUM_17,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&nCHG_EN);
    gpio_set_direction(GPIO_NUM_17, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_17, 0);

    while (1);
}