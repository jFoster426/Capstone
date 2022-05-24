#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include <math.h>

#include "adc.h"
#include "dac.h"

void app_main(void)
{
    printf("app_main start\n");
    spi_device_handle_t adc1;
    adc_init(&adc1, 7);

    dac_init(25000);

    // double i = -1.0;


    dac_write(-0.2, -0.2);

    while (1)
    {
        /*
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

        double dac_val = atof(ch);

        printf("received %.2f\n", dac_val);

        */

        


        uint32_t x = adc_read_raw(&adc1, 7);
        printf("%d\n", x);
        // dac_write(i, -i);
        // i += 0.01;
        // if (i > 1.0) i = -1.0;
    }
}