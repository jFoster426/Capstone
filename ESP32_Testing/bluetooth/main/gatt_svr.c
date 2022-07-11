#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

uint16_t notify_handle;

static int gatt_svr_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    printf("gatt_svr_chr_access()\n");

    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)
    {
        char *buf = "Reading data";
        if (os_mbuf_append(ctxt->om, buf, strlen(buf))) return BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR)
    {
        printf("Data received: %.*s\n", ctxt->om->om_len, ctxt->om->om_data);
    }

    return 0;
}

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

int gatt_svr_init(void)
{
    int rc;

    // ble_svc_gap_init();
    // ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) return rc;

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) return rc;

    return 0;
}