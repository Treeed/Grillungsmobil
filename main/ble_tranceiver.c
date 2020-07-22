//
// Created by treed on 11.07.2020.
//

#include "esp_log.h"
#include "nvs_flash.h"
/* BLE */
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "analog_reader.h"


#define GATT_SVR_SVC_ALERT_UUID               0x1811

static int bleprph_gap_event(struct ble_gap_event *event, void *arg);
static int gatt_get_battery_voltage_all(uint16_t conn_handle, uint16_t attr_handle,
                                        struct ble_gatt_access_ctxt *ctxt,
                                        void *arg);

static const ble_uuid128_t battery =
        BLE_UUID128_INIT(0x2d, 0x71, 0xa2, 0x59, 0xb4, 0x54, 0xc8, 0x12,
                         0x99, 0x99, 0x33, 0x97, 0x10, 0x23, 0x26, 0x79);

static const ble_uuid128_t battery_voltage_all =
        BLE_UUID128_INIT(0xf6, 0x7d, 0x29, 0x37, 0x31, 0x00, 0x16, 0xb0,
                         0xe1, 0x45, 0x7e, 0x59, 0x93, 0x75, 0x3a, 0x5c);

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
        {
                /*** Service: Security test. */
                .type = BLE_GATT_SVC_TYPE_PRIMARY,
                .uuid = &battery.u,
                .characteristics = (struct ble_gatt_chr_def[])
                        { {
                                  /*** Characteristic: Random number generator. */
                                  .uuid = &battery_voltage_all.u,
                                  .access_cb = gatt_get_battery_voltage_all,
                                  .flags = BLE_GATT_CHR_F_READ,
                          }, {
                                  0, /* No more characteristics in this service. */
                          }
                        },
        },

        {
                0, /* No more services. */
        },
};

static int gatt_get_battery_voltage_all(uint16_t conn_handle, uint16_t attr_handle,
                                        struct ble_gatt_access_ctxt *ctxt,
                                        void *arg)
{
    int rc;

    uint32_t voltage = 0;
    xQueuePeek(analog_value_queue, &voltage, 0);

    rc = os_mbuf_append(ctxt->om, &voltage, sizeof voltage);
    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

/**
 * Enables advertising with the following parameters:
 *     o General discoverable mode.
 *     o Undirected connectable mode.
 */
static void
bleprph_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    const char *name;
    int rc;

    /**
     *  Set the advertisement data included in our advertisements:
     *     o Flags (indicates advertisement type and other general info).
     *     o Advertising tx power.
     *     o Device name.
     *     o 16-bit service UUIDs (alert notifications).
     */

    memset(&fields, 0, sizeof fields);

    /* Advertise two flags:
     *     o Discoverability in forthcoming advertisement (general)
     *     o BLE-only (BR/EDR unsupported).
     */
    fields.flags = BLE_HS_ADV_F_DISC_GEN |
                   BLE_HS_ADV_F_BREDR_UNSUP;

    /* Indicate that the TX power level field should be included; have the
     * stack fill this value automatically.  This is done by assigning the
     * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
     */
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    fields.uuids16 = (ble_uuid16_t[]) {
            BLE_UUID16_INIT(GATT_SVR_SVC_ALERT_UUID)
    };
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error setting advertisement data; rc=%d\n", rc);
        return;
    }

    /* Begin advertising. */
    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(0, NULL, BLE_HS_FOREVER,
                           &adv_params, bleprph_gap_event, NULL);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error enabling advertisement; rc=%d\n", rc);
        return;
    }
}

/**
 * The nimble host executes this callback when a GAP event occurs.  The
 * application associates a GAP event callback with each connection that forms.
 * bleprph uses the same callback for all connections.
 *
 * @param event                 The type of event being signalled.
 * @param ctxt                  Various information pertaining to the event.
 * @param arg                   Application-specified argument; unuesd by
 *                                  bleprph.
 *
 * @return                      0 if the application successfully handled the
 *                                  event; nonzero on failure.  The semantics
 *                                  of the return code is specific to the
 *                                  particular GAP event being signalled.
 */
static int
bleprph_gap_event(struct ble_gap_event *event, void *arg)
{

    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status != 0) {
                /* Connection failed; resume advertising. */
                bleprph_advertise();
            }
            return 0;

        case BLE_GAP_EVENT_DISCONNECT:
        case BLE_GAP_EVENT_ADV_COMPLETE:
            bleprph_advertise();
            return 0;

    }
    return 0;
}


static void
bleprph_on_startup(void)
{
    int rc;

    rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);

    /* Begin advertising. */
    bleprph_advertise();
}

void bleprph_host_task(void *param)
{
    MODLOG_DFLT(INFO, "BLE Host Task Started");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}

void initialize_ble(){
    int rc;

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_nimble_hci_and_controller_init());

    nimble_port_init();
    /* Initialize the NimBLE host configuration. */
    ble_hs_cfg.sync_cb = bleprph_on_startup;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    assert(rc == 0);

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    assert(rc == 0);

    rc = ble_svc_gap_device_name_set("Grillungsmobil");
    assert(rc == 0);

    nimble_port_freertos_init(bleprph_host_task);
}
