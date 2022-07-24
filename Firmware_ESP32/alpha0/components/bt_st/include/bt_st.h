#ifndef bt_st_h
#define bt_st_h

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static void ble_on_sync(void);
static void ble_on_reset(int reason);
void ble_host_task(void *param);
static void ble_advertise(void);
static int gatt_svr_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
void bt_init(void);


#endif