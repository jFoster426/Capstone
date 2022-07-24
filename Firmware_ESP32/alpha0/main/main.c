#include <stdio.h>
#include "esp_log.h"

#include "adc_st.h"
#include "bt_st.h"
#include "dac_st.h"
#include "i2c_st.h"
#include "ism330_st.h"
#include "rtc_st.h"

#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "console/console.h"
#include "services/gap/ble_svc_gap.h"
#include "nimble/ble.h"
#include "modlog/modlog.h"

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init(35, 36, 400000));
    printf("I2C initialized successfully\n");

    ism330_init();
    rtc_st_init();
    bt_init();

    while (1)
    {
        printf("%02d:%02d:%02d\n", rtc_st_get_hours(), rtc_st_get_minutes(), rtc_st_get_seconds());
        vTaskDelay(100 / portTICK_RATE_MS);
    }

    ESP_ERROR_CHECK(i2c_driver_delete(0));
    printf("I2C de-initialized successfully\n");
}