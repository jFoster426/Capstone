#include "bt_st.h"

// Global pointer to the defined bluetooth configuration data, needed
// by all the static functions in this file.
static struct bt_config_t *bt_config_handle;
static uint16_t conn_handle;
static uint8_t ble_addr_type;

// Static function prototypes.
static void ble_advertise(void);
static int ble_gap_event(struct ble_gap_event *event, void *arg);
static void ble_on_sync(void);
static void ble_on_reset(int reason);
static int gatt_svr_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);

// Static variables (needed by things like UUID definitions).
static uint16_t notify_handle;

// Advertised services and UUIDs.
static const struct ble_gatt_svc_def gatt_svr_svcs[] =
{
    {
        // Service definition.
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID128_DECLARE(0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F),
        .characteristics = (struct ble_gatt_chr_def[])
        {
            {
                // Characteristic definition.
                .uuid = BLE_UUID128_DECLARE(0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00),
                .access_cb = gatt_svr_chr_access,
                .val_handle = &notify_handle,
                // Do everything in this one characteristic.
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_WRITE,
            },

            {
                0, // No more characteristics in this service
            },
        }
    },

    {
        0, // No more services
    },
};

// Enables advertising with parameters: general discoverable mode, undirected connectable mode.
static void ble_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;
    // Set the advertisement data included in our advertisements:
    // - Flags (indicates advertisement type and other general info)
    // - Advertising tx power
    // - Device name
    memset(&fields, 0, sizeof(fields));
    // Advertise two flags:
    // - Discoverability in forthcoming advertisement (general)
    // - BLE-only (BR/EDR unsupported)
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    // Indicate that the TX power level field should be included; have the
    // stack fill this value automatically.  This is done by assigning the
    // special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    fields.name = (uint8_t *)bt_config_handle->device_name;
    fields.name_len = strlen(bt_config_handle->device_name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0)
    {
        printf("Error setting advertisement data; rc = %d\n", rc);
        return;
    }
    // Begin advertising
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
    if (rc != 0)
    {
        printf("Error enabling advertisement; rc = %d\n", rc);
        return;
    }
}

static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    case BLE_GAP_EVENT_CONNECT:
        // A new connection was established or a connection attempt failed.
        printf("Connection %s.\n", event->connect.status == 0 ? "established" : "failed");
        // If connection failed, continue advertising.
        if (event->connect.status != 0)
            ble_advertise();
        // Set the global variable conn_handle for the other helper functions.
        conn_handle = event->connect.conn_handle;
        // Update the connection status for the main program.
        bt_config_handle->connected = true;
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        printf("Disconnected. Reason = %d.\n", event->disconnect.reason);
        // Connection terminated; resume advertising.
        ble_advertise();
        // Update the connection status for the main program.
        bt_config_handle->connected = false;
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        printf("Advertising complete\n.");
        ble_advertise();
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        printf("Subscribe event: cur_notify = %d, val_handle = %d.\n", event->subscribe.cur_notify, notify_handle);
        if (event->subscribe.attr_handle == notify_handle)
            bt_config_handle->notify_enabled = event->subscribe.cur_notify;
        else if (event->subscribe.attr_handle != notify_handle)
            bt_config_handle->notify_enabled = event->subscribe.cur_notify;
        break;

    case BLE_GAP_EVENT_MTU:
        printf("MTU update event: conn_handle = %d, mtu = %d.\n", event->mtu.conn_handle, event->mtu.value);
        break;
    }
    return 0;
}

static void ble_on_sync(void)
{
    int rc = ble_hs_id_infer_auto(0, &(ble_addr_type));
    assert(rc == 0);

    uint8_t addr_val[6] = { 0 };
    rc = ble_hs_id_copy_addr(ble_addr_type, addr_val, NULL);
    printf("BT MAC: %02X:%02X:%02X:%02X:%02X:%02X.\n", addr_val[5], addr_val[4], addr_val[3], addr_val[2], addr_val[1], addr_val[0]);
    ble_advertise();
}

static void ble_on_reset(int reason)
{
    printf("Resetting state; reason = %d.\n", reason);
}

// Callback function for when remote reads or writes data to the device.
static int gatt_svr_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    // The remote wants to read data from us.
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)
    {
        // Data is available to write to the remote.
        if (bt_config_handle->write_available == false)
        {
            // Don't write the same data twice.
            bt_config_handle->write_available = true;
            printf("write_available = true\n");
            
            // Add the data to the characteristic.
            if (os_mbuf_append(ctxt->om, bt_config_handle->write_data, bt_config_handle->write_len))
                return BLE_ATT_ERR_INSUFFICIENT_RES;
        }
    }
    
    // The remote wants to write data to us.
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR)
    {
        // Copy the received data from the remote.
        memcpy(bt_config_handle->read_data, ctxt->om->om_data, ctxt->om->om_len);
        bt_config_handle->read_len = ctxt->om->om_len;
        // Signify that the data is new and hasn't been read yet.
        bt_config_handle->read_available = true;
    }
    return 0;
}

// Functions accessible to the main program (defined in bt_st.h).
void ble_host_task(void *param)
{
    printf("BLE host tast started.\n");
    // This function will return only when nimble_port_stop() is executed.
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void bt_init(struct bt_config_t *bt_config_ptr)
{
    // Don't initialize without a valid pointer.
    if (bt_config_ptr == NULL) return;

    // Global variable (private to this file) needed by the different helper functions.
    bt_config_handle = bt_config_ptr;

    // Initialize NVS — it is used to store PHY calibration data.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_nimble_hci_and_controller_init());

    nimble_port_init();
    // Initialize the NimBLE host configuration.
    ble_hs_cfg.sync_cb = ble_on_sync;
    ble_hs_cfg.reset_cb = ble_on_reset;

    int rc = 0;
    rc += ble_gatts_count_cfg(gatt_svr_svcs);
    rc += ble_gatts_add_svcs(gatt_svr_svcs);
    // Set the default device name.
    rc += ble_svc_gap_device_name_set(bt_config_handle->device_name);
    assert(rc == 0);

    nimble_port_freertos_init(ble_host_task);

    // Change to the maximum allowed MTU size, so that we can send up
    // to 512 bytes in the attribute.
    ble_att_set_preferred_mtu(527);

    // Initialize the pointer variables to the correct values.
    bt_config_handle->read_len = 0;
    bt_config_handle->write_len = 0;
    bt_config_handle->read_available = true;
    bt_config_handle->write_available = true;
    bt_config_handle->notify_enabled = false;
    bt_config_handle->connected = false;
    for (uint16_t i = 0; i < 512; i++)
    {
        bt_config_handle->read_data[i] = 0;
        bt_config_handle->write_data[i] = 0;
    }
}

void bt_read(uint8_t *data, uint16_t len)
{
    // Copy the latest data read from the remote into the passed memory location.
    memcpy(data, bt_config_handle->read_data, len);
    // Signal that the data has been transferred and we can read again.
    bt_config_handle->read_available = false;
}

uint8_t bt_write(uint8_t *data, uint16_t len)
{
    // Unable to write the data requested.
    if (bt_config_handle->write_available == false)
        return -1;

    // Unable to write because size is too big.
    if (len > 512)
        return -2;

    // Copy the data passed to the function into the write buffer.
    memcpy(bt_config_handle->write_data, data, len);
    bt_config_handle->write_len = len;

    // Set the write available flag, so when the remote reads data from the
    // characteristic, the data will be sent.
    bt_config_handle->write_available = false;

    return 0;
}

uint8_t bt_notify(uint8_t *data, uint16_t len)
{
    // Notify the remote if they are subscribed to notifications from the
    // characteristic.
    if (bt_config_handle->notify_enabled == false)
        return -1;
    
    // Write the data to the remote.
    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    uint8_t rc = ble_gattc_notify_custom(conn_handle, notify_handle, om);
    return rc;
}