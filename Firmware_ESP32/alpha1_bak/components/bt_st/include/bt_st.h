#ifndef bt_st_h
#define bt_st_h

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "nvs_flash.h"
#include "freertos/FreeRTOSConfig.h"

#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"

#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "nimble/ble.h"

#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

// Meant to be initialized by the main program and holds the current
// transfer action that is being completed.
struct bt_config_t
{
    // Access these variables to read from the device.
    bool read_available;
    uint8_t read_data[512];
    uint16_t read_len;

    // Access these variables to write to the device.
    bool write_available;
    uint8_t write_data[512];
    uint16_t write_len;

    bool notify_enabled;        // Does the connected device have notifications enabled?
    char device_name[32];
    bool connected;

    bool initialized;
};

// Initialize the bluetooth module. Call with the address of an instantiated
// bt_config_t struct above to configure the device name.
void bt_init(struct bt_config_t *bt_config_ptr);

// Read data from the remote.
void bt_read(uint8_t *data, uint16_t len);

// Write data without notifying the remote.
uint8_t bt_write(uint8_t *data, uint16_t len);

// Write data and notify the remote.
uint8_t bt_notify(uint8_t *data, uint16_t len);

#endif