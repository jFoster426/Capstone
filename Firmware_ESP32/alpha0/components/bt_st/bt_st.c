#include "bt_st.h"

// Private variables.
const char *device_name = "STRASAnkle Beta";

uint16_t notify_handle;

uint8_t batteryLevel = 0;

uint8_t miscFileWrite = 0;
uint8_t dataFileWrite = 0;

// Advertised services and UUIDs.
static const struct ble_gatt_svc_def gatt_svr_svcs[] =
{
    {
        // Service: Heart-rate
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID128_DECLARE(0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F),
        .characteristics = (struct ble_gatt_chr_def[])
        {
            {
                .uuid = BLE_UUID128_DECLARE(0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00),
                .access_cb = gatt_svr_chr_access,
                .val_handle = &notify_handle,
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

// Utility function to print MAC address.
void print_addr(const void *addr)
{
    const uint8_t *u8p;
    u8p = addr;
    MODLOG_DFLT(INFO, "%02x:%02x:%02x:%02x:%02x:%02x", u8p[5], u8p[4], u8p[3], u8p[2], u8p[1], u8p[0]);
}

static void ble_on_sync(void)
{
    int rc;

    rc = ble_hs_id_infer_auto(0, &blehr_addr_type);
    assert(rc == 0);

    uint8_t addr_val[6] = {0};
    rc = ble_hs_id_copy_addr(blehr_addr_type, addr_val, NULL);

    MODLOG_DFLT(INFO, "Device Address: ");
    print_addr(addr_val);
    MODLOG_DFLT(INFO, "\n");

    ble_advertise();
}

static void ble_on_reset(int reason)
{
    MODLOG_DFLT(ERROR, "Resetting state; reason=%d\n", reason);
}

void ble_host_task(void *param)
{
    ESP_LOGI(tag, "BLE Host Task Started");
    // This function will return only when nimble_port_stop() is executed.
    nimble_port_run();

    nimble_port_freertos_deinit();
}

// Enables advertising with parameters:
// General discoverable mode.
// Undirected connectable mode.
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
    fields.flags = BLE_HS_ADV_F_DISC_GEN |
                   BLE_HS_ADV_F_BREDR_UNSUP;
    // Indicate that the TX power level field should be included; have the
    // stack fill this value automatically.  This is done by assigning the
    // special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0)
    {
        MODLOG_DFLT(ERROR, "error setting advertisement data; rc=%d\n", rc);
        return;
    }
    // Begin advertising
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(blehr_addr_type, NULL, BLE_HS_FOREVER, &adv_params, blehr_gap_event, NULL);
    if (rc != 0)
    {
        MODLOG_DFLT(ERROR, "error enabling advertisement; rc=%d\n", rc);
        return;
    }
}

// Callback function for when computer reads or writes data to the device.
static int gatt_svr_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    printf("gatt_svr_chr_access()\n");

    // The computer wants to read data.
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)
    {
        char *buf = "Reading data";
        if (os_mbuf_append(ctxt->om, buf, strlen(buf)))
        {
            return BLE_ATT_ERR_INSUFFICIENT_RES;
        }
    }
    
    // The computer wants to write data.
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR)
    {
        char buf[1024];
        sprintf(buf, "%.*s", ctxt->om->om_len, ctxt->om->om_data);

        if (miscFileWrite == 1)
        {
            printf("miscfile data: %.*s\n", ctxt->om->om_len, ctxt->om->om_data);
            miscFileWrite = 0;
            return 0;
        }

        if (strcmp(buf, "miscFile,w") == 0)
        {
            // Update the state machine status.
            miscFileWrite = 1;
            printf("initializing miscFile write\n");
            return 0;
        }

        if (strcmp(buf, "dataFile,w") == 0)
        {
            // Update the state machine status.
            dataFileWrite = 1;
            printf("initializing dataFile write\n");
            return 0;
        }

        if (strcmp(buf, "miscFile,r") == 0)
        {
            printf("Sending miscFile\n");

            sprintf(buf, "miscFile\nbatteryStatus,charging,shinConnect,shinMalf,testing,deviceStatus\n%d,0,0,1,0,0", batteryLevel++);
            if (batteryLevel > 100) batteryLevel = 0;
            struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, strlen(buf));
            int rc = ble_gattc_notify_custom(conn_handle, notify_handle, om);
            printf("Data received: %.*s\n", ctxt->om->om_len, ctxt->om->om_data);
            return 0;
        }

        if (strcmp(buf, "dataFile,r") == 0)
        {
            printf("Sending dataFile\n");

            sprintf(buf, "time,faccx,faccy,faccz,fgyrx,fgyry,fgyrz,saccx,saccy,saccz,sgyrx,sgyry,sgyrz,load\n125791,0.02,1.02,-0.10,-1.56,-3.17,-4.12,-0.13,0.61,0.85,-3.39,-1.34,-2.26,16608376\n125801,-0.01,1.01,-0.10,-0.67,-3.02,-1.34,-0.13,0.61,0.85,-3.42,-1.31,-2.75,16607889\n125812,-0.04,1.01,-0.09,3.94,1.62,-0.06,-0.12,0.61,0.84,-3.48,-0.64,-2.62,16607801\n");
            if (batteryLevel > 100) batteryLevel = 0;
            struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, strlen(buf));
            int rc = ble_gattc_notify_custom(conn_handle, notify_handle, om);
            printf("Data received: %.*s\n", ctxt->om->om_len, ctxt->om->om_data);
            return 0;
        }
    }

    return 0;
}

void bt_init(void)
{
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
    rc += ble_svc_gap_device_name_set(device_name);
    assert(rc == 0);

    nimble_port_freertos_init(ble_host_task);
}