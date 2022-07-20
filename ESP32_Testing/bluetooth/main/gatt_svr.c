#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

uint16_t notify_handle;

uint8_t batteryLevel = 0;

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
        char buf[1024];
        sprintf(buf, "%.*s", ctxt->om->om_len, ctxt->om->om_data);

        if (strcmp(buf, "miscFile,r") == 0)
        {
            printf("Sending miscFile\n");

            sprintf(buf, "miscfile\nbatteryStatus,charging,shinConnect,shinMalf,testing,deviceStatus\n%d,0,0,1,0,0", batteryLevel++);
            if (batteryLevel > 100) batteryLevel = 0;
            struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, strlen(buf));
            int rc = ble_gattc_notify_custom(conn_handle, notify_handle, om);
        }

        if (strcmp(buf, "dataFile,r") == 0)
        {
            printf("Sending dataFile\n");

            sprintf(buf, "time,faccx,faccy,faccz,fgyrx,fgyry,fgyrz,saccx,saccy,saccz,sgyrx,sgyry,sgyrz,load\n125791,0.02,1.02,-0.10,-1.56,-3.17,-4.12,-0.13,0.61,0.85,-3.39,-1.34,-2.26,16608376\n125801,-0.01,1.01,-0.10,-0.67,-3.02,-1.34,-0.13,0.61,0.85,-3.42,-1.31,-2.75,16607889\n125812,-0.04,1.01,-0.09,3.94,1.62,-0.06,-0.12,0.61,0.84,-3.48,-0.64,-2.62,16607801\n");
            if (batteryLevel > 100) batteryLevel = 0;
            struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, strlen(buf));
            int rc = ble_gattc_notify_custom(conn_handle, notify_handle, om);
        }

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