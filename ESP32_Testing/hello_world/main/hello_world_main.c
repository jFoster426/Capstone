/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "hal/gpio_types.h"

void app_main(void)
{
    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), WiFi%s%s, ",
            CONFIG_IDF_TARGET,
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());

    gpio_config_t led_pin = { GPIO_NUM_9, GPIO_MODE_OUTPUT, 0, 0, 0 };
    gpio_config(&led_pin);
    gpio_set_direction(GPIO_NUM_9, GPIO_MODE_OUTPUT);

    //gpio_config_t nCHG_EN_pin = { GPIO_NUM_17, GPIO_MODE_OUTPUT, 0, 0, 0 };
    //gpio_config(&nCHG_EN_pin);
    //gpio_set_direction(GPIO_NUM_17, GPIO_MODE_OUTPUT);
    //gpio_set_level(GPIO_NUM_17, 0);

    while (1)
    {
        gpio_set_level(GPIO_NUM_9, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        gpio_set_level(GPIO_NUM_9, 0);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
