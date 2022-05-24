/* I2S Example

    This example code will output 100Hz sine wave and triangle wave to 2-channel of I2S driver
    Every 5 seconds, it will change bits_per_sample [16, 24, 32] for i2s data

    This example code is in the Public Domain (or CC0 licensed, at your option.)

    Unless required by applicable law or agreed to in writing, this
    software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
    CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_log.h"
#include <math.h>


#define SAMPLE_RATE     (36000)
#define I2S_NUM         (0)
#define WAVE_FREQ_HZ    (100)
#define PI              (3.14159265)
#define I2S_BCK_IO      (GPIO_NUM_5)
#define I2S_MCK_IO      (GPIO_NUM_3)
#define I2S_WS_IO       (GPIO_NUM_6)
#define I2S_DO_IO       (GPIO_NUM_4)
#define I2S_DI_IO       (-1)

#define SAMPLE_PER_CYCLE (SAMPLE_RATE/WAVE_FREQ_HZ)

static const char* TAG = "i2s_example";

void app_main(void)
{
    //for 36Khz sample rates, we create 100Hz sine wave, every cycle need 36000/100 = 360 samples (4-bytes or 8-bytes each sample)
    //depend on bits_per_sample
    //using 6 buffers, we need 60-samples per buffer
    //if 2-channels, 24/32-bit each channel, total buffer is 360*8 = 2880 bytes
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX,
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_24BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_MSB,
        .dma_buf_count = 6,
        .dma_buf_len = 60,
        .use_apll = false,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1                                //Interrupt level 1
    };
    i2s_pin_config_t pin_config = {
        .mck_io_num = I2S_MCK_IO,
        .bck_io_num = I2S_BCK_IO,
        .ws_io_num = I2S_WS_IO,
        .data_out_num = I2S_DO_IO,
        .data_in_num = I2S_DI_IO                                               //Not used
    };
    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM, &pin_config);

    int *samples_data = malloc(((32+8)/16)*SAMPLE_PER_CYCLE*4);
    float sin_float = 0.0;
    size_t i2s_bytes_write = 0;

    while (1)
    {
        char ch[6];
        int i = 0;
        while (1)
        {
            ch[i] = fgetc(stdin);
            if (ch[i] == '\n')
            {
                ch[i] = '\0';
                break;
            }
            if (ch[i] != 0xFF) i++;
        }

        sin_float = atof(ch);

        int32_t sin_int = 0;

        for(unsigned int i = 0; i < SAMPLE_PER_CYCLE; i++)
        {
            // Set it to be even.
            // sin_float = sin(i * 2 * PI / SAMPLE_PER_CYCLE);
            sin_int = sin_float * (pow(2, 32)/2 - 1);
            samples_data[i*2] = sin_int;
            samples_data[i*2 + 1] = sin_int;
        }
        printf("set clock\n");
        i2s_set_clk(I2S_NUM, SAMPLE_RATE, 32, 2);
        
        printf("write data: %.2f, %d, %s\n", sin_float, sin_int, ch);
        i2s_write(I2S_NUM, samples_data, ((32+8)/16)*SAMPLE_PER_CYCLE*4, &i2s_bytes_write, 100);
    }
}
