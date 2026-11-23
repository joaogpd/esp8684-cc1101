/*
 * ble_nus.c - BLE peripheral with the Nordic UART Service (NimBLE host)
 *
 * Structure follows the ESP-IDF v5.2 bleprph example: nimble_port_init(),
 * GATT table registered before the host starts, advertising (re)started
 * from the sync callback and after every disconnect.
 */
#include "ble_nus.h"
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "ble_nus";

/* 128-bit UUIDs are stored little-endian: 6E4000xx-B5A3-F393-E0A9-E50E24DCCA9E */
#define NUS_UUID(x) BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, \
                                     0x93, 0xF3, 0xA3, 0xB5, (x), 0x00, 0x40, 0x6E)

static const ble_uuid128_t k_svc_uuid = NUS_UUID(0x01);
static const ble_uuid128_t k_rx_uuid  = NUS_UUID(0x02);
static const ble_uuid128_t k_tx_uuid  = NUS_UUID(0x03);

#define RX_BUF_SIZE     (256u)      /* > the largest write at MTU 247 (244) */
#define SEND_TRIES      (50u)       /* x 10 ms waiting for host buffers */

static ble_nus_rx_cb_t s_on_rx;
static char            s_name[21];
static uint8_t         s_own_addr_type;
static uint16_t        s_tx_handle;

static volatile uint16_t s_conn = BLE_HS_CONN_HANDLE_NONE;
static volatile bool     s_notify;
static volatile uint16_t s_mtu = BLE_ATT_MTU_DFLT;

static int gap_event(struct ble_gap_event *ev, void *arg);

/* ---- GATT ------------------------------------------------------------ */

static int chr_access(uint16_t conn, uint16_t attr,
                      struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn;
    (void)attr;
    (void)arg;
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) { return BLE_ATT_ERR_UNLIKELY; }

    uint8_t buf[RX_BUF_SIZE];
    uint16_t len = 0;
    if (ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof buf, &len) != 0)
    {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    if (s_on_rx && len != 0u) { s_on_rx(buf, len); }
    return 0;
}

static const struct ble_gatt_svc_def s_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &k_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid      = &k_rx_uuid.u,
                .access_cb = chr_access,
                .flags     = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                .uuid       = &k_tx_uuid.u,
                .access_cb  = chr_access,
                .val_handle = &s_tx_handle,
                .flags      = BLE_GATT_CHR_F_NOTIFY,
            },
            { 0 },
        },
    },
    { 0 },
};

/* ---- GAP ------------------------------------------------------------- */

static void advertise(void)
{
    /* Advertising packet (31 bytes): flags + complete name */
    struct ble_hs_adv_fields adv;
    memset(&adv, 0, sizeof adv);
    adv.flags            = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    adv.name             = (const uint8_t *)s_name;
    adv.name_len         = (uint8_t)strlen(s_name);
    adv.name_is_complete = 1;

    /* Scan response: the 128-bit NUS UUID, so apps filtering on it find us */
    struct ble_hs_adv_fields rsp;
    memset(&rsp, 0, sizeof rsp);
    rsp.uuids128             = &k_svc_uuid;
    rsp.num_uuids128         = 1;
    rsp.uuids128_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&adv);
    if (rc == 0) { rc = ble_gap_adv_rsp_set_fields(&rsp); }
    if (rc != 0)
    {
        ESP_LOGE(TAG, "advertising data rc=%d", rc);
        return;
    }

    struct ble_gap_adv_params p;
    memset(&p, 0, sizeof p);
    p.conn_mode = BLE_GAP_CONN_MODE_UND;    /* connectable */
    p.disc_mode = BLE_GAP_DISC_MODE_GEN;    /* discoverable */

    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER, &p, gap_event, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY)
    {
        ESP_LOGE(TAG, "advertising start rc=%d", rc);
        return;
    }
    ESP_LOGI(TAG, "advertising as \"%s\"", s_name);
}

static int gap_event(struct ble_gap_event *ev, void *arg)
{
    (void)arg;
    switch (ev->type)
    {
    case BLE_GAP_EVENT_CONNECT:
        if (ev->connect.status == 0)
        {
            s_conn   = ev->connect.conn_handle;
            s_notify = false;
            s_mtu    = BLE_ATT_MTU_DFLT;
            ESP_LOGI(TAG, "connected");
        }
        else
        {
            advertise();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "disconnected, reason 0x%03X", (unsigned)ev->disconnect.reason);
        s_conn   = BLE_HS_CONN_HANDLE_NONE;
        s_notify = false;
        s_mtu    = BLE_ATT_MTU_DFLT;
        advertise();
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        advertise();
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (ev->subscribe.attr_handle == s_tx_handle)
        {
            s_notify = ev->subscribe.cur_notify != 0;
            ESP_LOGI(TAG, "notifications %s", s_notify ? "on" : "off");
        }
        break;

    case BLE_GAP_EVENT_MTU:
        s_mtu = ev->mtu.value;
        ESP_LOGI(TAG, "MTU %u", (unsigned)s_mtu);
        break;

    default:
        break;
    }
    return 0;
}

static void on_sync(void)
{
    if (ble_hs_util_ensure_addr(0) != 0 ||
        ble_hs_id_infer_auto(0, &s_own_addr_type) != 0)
    {
        ESP_LOGE(TAG, "no usable BLE address");
        return;
    }
    advertise();
}

static void on_reset(int reason)
{
    ESP_LOGW(TAG, "host reset, reason %d", reason);
}

static void host_task(void *param)
{
    (void)param;
    nimble_port_run();              /* returns only after nimble_port_stop() */
    nimble_port_freertos_deinit();
}

/* ---- API ------------------------------------------------------------- */

bool ble_nus_init(const char *name, ble_nus_rx_cb_t on_rx)
{
    s_on_rx = on_rx;
    strlcpy(s_name, name ? name : "esp8684", sizeof s_name);

    if (nimble_port_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "nimble_port_init failed");
        return false;
    }

    ble_hs_cfg.sync_cb  = on_sync;
    ble_hs_cfg.reset_cb = on_reset;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    if (ble_gatts_count_cfg(s_svcs) != 0 || ble_gatts_add_svcs(s_svcs) != 0)
    {
        ESP_LOGE(TAG, "GATT table rejected");
        return false;
    }
    (void)ble_svc_gap_device_name_set(s_name);

    nimble_port_freertos_init(host_task);
    return true;
}

bool ble_nus_connected(void) { return s_conn != BLE_HS_CONN_HANDLE_NONE; }
bool ble_nus_ready(void)     { return s_conn != BLE_HS_CONN_HANDLE_NONE && s_notify; }
uint16_t ble_nus_mtu(void)   { return s_mtu; }

bool ble_nus_send(const uint8_t *data, size_t len)
{
    if (!data || !ble_nus_ready()) { return false; }

    while (len != 0u)
    {
        size_t room = (size_t)s_mtu - 3u;
        size_t n = (len < room) ? len : room;
        bool sent = false;

        for (unsigned tries = 0u; tries < SEND_TRIES && !sent; tries++)
        {
            if (!ble_nus_ready()) { return false; }
            struct os_mbuf *om = ble_hs_mbuf_from_flat(data, (uint16_t)n);
            if (!om)
            {
                vTaskDelay(pdMS_TO_TICKS(10));      /* out of mbufs: wait */
                continue;
            }
            /* om is consumed by the call, also on error */
            int rc = ble_gatts_notify_custom(s_conn, s_tx_handle, om);
            if (rc == 0)                { sent = true; }
            else if (rc == BLE_HS_ENOMEM) { vTaskDelay(pdMS_TO_TICKS(10)); }
            else                        { return false; }
        }
        if (!sent) { return false; }

        data += n;
        len  -= n;
    }
    return true;
}
