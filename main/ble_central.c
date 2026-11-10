/*
 * NimBLE central: scan -> connect -> MTU/PHY/DLE -> discover -> read fw rev (2A26)
 * + codec (19B10002) -> optional time sync -> subscribe audio (19B10001)
 * -> [storage sync only if enabled] subscribe 30295781.
 *
 * Every GATT step is a callback that triggers the next one (step machine).
 */
#include <stdio.h>
#include <string.h>

#include "bridge.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "sdkconfig.h"

static const char *TAG = "ble";

/* 128-bit UUIDs, little-endian byte order */
#define OMI_AUDIO_UUID(b0) \
    BLE_UUID128_INIT(0x14, 0x12, 0x8A, 0x76, 0x04, 0xD1, 0x6C, 0x4F, 0x7E, 0x53, 0xF2, 0xE8, b0, 0x00, 0xB1, 0x19)
#define OMI_STORAGE_UUID(b0) \
    BLE_UUID128_INIT(0x43, 0xAE, 0xFE, 0xAD, 0x49, 0x28, 0x04, 0x29, 0xBD, 0xEA, 0x01, 0x43, b0, 0x57, 0x29, 0x30)

static const ble_uuid128_t UUID_AUDIO_SVC = OMI_AUDIO_UUID(0x00);
static const ble_uuid128_t UUID_AUDIO_DATA = OMI_AUDIO_UUID(0x01);
static const ble_uuid128_t UUID_AUDIO_CODEC = OMI_AUDIO_UUID(0x02);
static const ble_uuid128_t UUID_TIME_SET = OMI_AUDIO_UUID(0x31); /* 19B10031 */
static const ble_uuid128_t UUID_STORAGE_CMD = OMI_STORAGE_UUID(0x81);
static const ble_uuid16_t UUID_FW_REV = BLE_UUID16_INIT(0x2A26);
static const ble_uuid16_t UUID_CCCD = BLE_UUID16_INIT(0x2902);

#define MAX_SVCS 16
#define MAX_CHRS 48

typedef struct { uint16_t start, end; } svc_t;
typedef struct { uint16_t def, val; ble_uuid_any_t uuid; } chr_t;

static uint8_t s_own_addr_type;
static uint16_t s_conn = BLE_HS_CONN_HANDLE_NONE;
static volatile bool s_connected;
static svc_t s_svcs[MAX_SVCS];
static int s_nsvcs;
static chr_t s_chrs[MAX_CHRS];
static int s_nchrs;

static uint16_t h_audio, h_audio_cccd, h_codec, h_time, h_storage, h_storage_cccd, h_fwrev;
static char s_fw[32];
static omi_storage_proto_t s_proto = OMI_PROTO_UNKNOWN;
static omi_frame_asm_t s_asm;
static uint8_t s_frame[OMI_OPUS_MAX_FRAME];
static uint32_t s_notifies;

static int gap_event(struct ble_gap_event *event, void *arg);
static void start_scan(void);
static void step_disc_chrs(void);

static void fail(const char *what, int rc)
{
    ESP_LOGE(TAG, "%s failed rc=%d; disconnecting", what, rc);
    if (s_conn != BLE_HS_CONN_HANDLE_NONE) ble_gap_terminate(s_conn, BLE_ERR_REM_USER_CONN_TERM);
}

static bool mac_matches(const ble_addr_t *a)
{
    if (!g_cfg.pendant_mac[0]) return true;
    char s[18];
    /* NimBLE stores addresses little-endian */
    snprintf(s, sizeof(s), "%02x:%02x:%02x:%02x:%02x:%02x", a->val[5], a->val[4], a->val[3], a->val[2], a->val[1], a->val[0]);
    return strcasecmp(s, g_cfg.pendant_mac) == 0;
}

static bool adv_is_omi(const struct ble_gap_disc_desc *d)
{
    struct ble_hs_adv_fields f;
    if (ble_hs_adv_parse_fields(&f, d->data, d->length_data) != 0) return false;
    size_t pl = strlen(CONFIG_OMI_NAME_PREFIX);
    if (f.name && f.name_len >= pl && memcmp(f.name, CONFIG_OMI_NAME_PREFIX, pl) == 0) return true;
    for (int i = 0; i < f.num_uuids128; i++)
        if (ble_uuid_cmp(&f.uuids128[i].u, &UUID_AUDIO_SVC.u) == 0) return true;
    return false;
}

static void start_scan(void)
{
    struct ble_gap_disc_params dp = {0};
    dp.passive = 0;          /* active: name is often in the scan response */
    dp.filter_duplicates = 1;
    dp.itvl = 0x50;
    dp.window = 0x30;
    int rc = ble_gap_disc(s_own_addr_type, BLE_HS_FOREVER, &dp, gap_event, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) ESP_LOGE(TAG, "scan start rc=%d", rc);
    else ESP_LOGI(TAG, "scanning for pendant%s%s", g_cfg.pendant_mac[0] ? " mac=" : "", g_cfg.pendant_mac);
}

static void connect_to(const ble_addr_t *addr)
{
    if (ble_gap_disc_cancel() != 0) { /* not scanning: fine */ }
    struct ble_gap_conn_params cp = {
        .scan_itvl = 0x10, .scan_window = 0x10,
        .itvl_min = 6, .itvl_max = 12,            /* 7.5 .. 15 ms */
        .latency = 0, .supervision_timeout = 400, /* 4 s */
        .min_ce_len = 0, .max_ce_len = 0,
    };
    int rc = ble_gap_connect(s_own_addr_type, addr, 30000, &cp, gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "connect rc=%d", rc);
        start_scan();
    }
}

/* ---- helpers ---- */
static uint16_t chr_end(int i)
{
    uint16_t end = 0xFFFF;
    if (i + 1 < s_nchrs) end = s_chrs[i + 1].def - 1;
    for (int k = 0; k < s_nsvcs; k++)
        if (s_chrs[i].def >= s_svcs[k].start && s_chrs[i].def <= s_svcs[k].end && s_svcs[k].end < end) end = s_svcs[k].end;
    return end;
}

static int find_chr(const ble_uuid_t *u)
{
    for (int i = 0; i < s_nchrs; i++)
        if (ble_uuid_cmp(&s_chrs[i].uuid.u, u) == 0) return i;
    return -1;
}

static int write_cccd_cb(uint16_t conn, const struct ble_gatt_error *err, struct ble_gatt_attr *attr, void *arg);

static void subscribe(uint16_t cccd, int which)
{
    static const uint8_t notify_on[2] = {0x01, 0x00};
    int rc = ble_gattc_write_flat(s_conn, cccd, notify_on, sizeof(notify_on), write_cccd_cb, (void *)(intptr_t)which);
    if (rc) fail("cccd write", rc);
}

/* ---- step machine ---- */
static int write_cccd_cb(uint16_t conn, const struct ble_gatt_error *err, struct ble_gatt_attr *attr, void *arg)
{
    int which = (int)(intptr_t)arg;
    if (err->status != 0) {
        fail(which == 0 ? "audio subscribe" : "storage subscribe", err->status);
        return 0;
    }
    if (which == 0) {
        ESP_LOGI(TAG, "live audio subscribed (fw %s)", s_fw[0] ? s_fw : "?");
        if (g_cfg.storage_sync && h_storage_cccd && s_proto == OMI_PROTO_RING) {
            subscribe(h_storage_cccd, 1);
        } else if (g_cfg.storage_sync) {
            ESP_LOGW(TAG, "storage sync enabled but %s; live only",
                     s_proto == OMI_PROTO_FILE ? "fw <=3.0.19 file protocol is not implemented" : "storage char/protocol unknown");
        }
    } else {
        ESP_LOGW(TAG, "storage notifications subscribed (ring protocol)");
        ring_sync_on_ready(s_proto);
    }
    return 0;
}

static int time_write_cb(uint16_t conn, const struct ble_gatt_error *err, struct ble_gatt_attr *attr, void *arg)
{
    ESP_LOGI(TAG, "pendant time sync %s", err->status == 0 ? "ok" : "failed (ignored)");
    subscribe(h_audio_cccd, 0);
    return 0;
}

static void step_time_then_subscribe(void)
{
#if CONFIG_OMI_PENDANT_TIME_SYNC
    if (h_time && net_time_valid()) {
        uint8_t p[4];
        omi_time_sync_payload(p, net_epoch());
        if (ble_gattc_write_flat(s_conn, h_time, p, 4, time_write_cb, NULL) == 0) return;
    }
#endif
    subscribe(h_audio_cccd, 0);
}

static int codec_read_cb(uint16_t conn, const struct ble_gatt_error *err, struct ble_gatt_attr *attr, void *arg)
{
    if (err->status != 0 || !attr) {
        fail("codec read", err->status);
        return 0;
    }
    uint8_t codec = 0xFF;
    os_mbuf_copydata(attr->om, 0, 1, &codec);
    ESP_LOGI(TAG, "codec id %u", codec);
    if (codec != OMI_CODEC_OPUS && codec != OMI_CODEC_OPUS_FS320) {
        ESP_LOGE(TAG, "unsupported codec %u (need Opus 20/21)", codec);
        fail("codec check", codec);
        return 0;
    }
    step_time_then_subscribe();
    return 0;
}

static int fw_read_cb(uint16_t conn, const struct ble_gatt_error *err, struct ble_gatt_attr *attr, void *arg)
{
    if (err->status == 0 && attr) {
        uint16_t n = OS_MBUF_PKTLEN(attr->om);
        if (n >= sizeof(s_fw)) n = sizeof(s_fw) - 1;
        os_mbuf_copydata(attr->om, 0, n, s_fw);
        s_fw[n] = 0;
        s_proto = omi_storage_proto_for(s_fw, n);
    } else {
        ESP_LOGW(TAG, "firmware revision read failed (%d)", err->status);
    }
    ESP_LOGI(TAG, "pendant firmware '%s' -> storage protocol %s", s_fw,
             s_proto == OMI_PROTO_RING ? "ring (3.0.21+)" : s_proto == OMI_PROTO_FILE ? "file (<=3.0.19)" : "unknown");
    int rc = ble_gattc_read(s_conn, h_codec, codec_read_cb, NULL);
    if (rc) fail("codec read", rc);
    return 0;
}

static void step_reads(void)
{
    if (h_fwrev) {
        int rc = ble_gattc_read(s_conn, h_fwrev, fw_read_cb, NULL);
        if (rc) fail("fw read", rc);
    } else {
        struct ble_gatt_error e = {.status = BLE_HS_ENOENT};
        fw_read_cb(s_conn, &e, NULL, NULL);
    }
}

static int dsc_cb(uint16_t conn, const struct ble_gatt_error *err, uint16_t chr_val, const struct ble_gatt_dsc *dsc, void *arg)
{
    int which = (int)(intptr_t)arg;
    if (err->status == 0) {
        if (ble_uuid_cmp(&dsc->uuid.u, &UUID_CCCD.u) == 0) {
            if (which == 0) h_audio_cccd = dsc->handle;
            else h_storage_cccd = dsc->handle;
        }
        return 0;
    }
    if (err->status != BLE_HS_EDONE) {
        fail("descriptor discovery", err->status);
        return 0;
    }
    if (which == 0) {
        if (!h_audio_cccd) { fail("audio CCCD missing", 0); return 0; }
        int si = h_storage ? find_chr(&UUID_STORAGE_CMD.u) : -1;
        if (g_cfg.storage_sync && si >= 0) {
            int rc = ble_gattc_disc_all_dscs(s_conn, s_chrs[si].val, chr_end(si), dsc_cb, (void *)1);
            if (rc) fail("storage dsc", rc);
            return 0;
        }
    }
    step_reads();
    return 0;
}

static int chr_cb(uint16_t conn, const struct ble_gatt_error *err, const struct ble_gatt_chr *chr, void *arg)
{
    if (err->status == 0) {
        if (s_nchrs < MAX_CHRS) {
            s_chrs[s_nchrs].def = chr->def_handle;
            s_chrs[s_nchrs].val = chr->val_handle;
            s_chrs[s_nchrs].uuid = chr->uuid;
            s_nchrs++;
        }
        return 0;
    }
    if (err->status != BLE_HS_EDONE) {
        fail("characteristic discovery", err->status);
        return 0;
    }
    int i;
    h_audio = (i = find_chr(&UUID_AUDIO_DATA.u)) >= 0 ? s_chrs[i].val : 0;
    h_codec = (i = find_chr(&UUID_AUDIO_CODEC.u)) >= 0 ? s_chrs[i].val : 0;
    h_time = (i = find_chr(&UUID_TIME_SET.u)) >= 0 ? s_chrs[i].val : 0;
    h_storage = (i = find_chr(&UUID_STORAGE_CMD.u)) >= 0 ? s_chrs[i].val : 0;
    h_fwrev = (i = find_chr(&UUID_FW_REV.u)) >= 0 ? s_chrs[i].val : 0;
    ESP_LOGI(TAG, "%d chrs: audio=%u codec=%u time=%u storage=%u fwrev=%u", s_nchrs, h_audio, h_codec, h_time, h_storage, h_fwrev);
    if (!h_audio || !h_codec) {
        fail("Omi audio service not found", 0);
        return 0;
    }
    int ai = find_chr(&UUID_AUDIO_DATA.u);
    int rc = ble_gattc_disc_all_dscs(s_conn, s_chrs[ai].val, chr_end(ai), dsc_cb, (void *)0);
    if (rc) fail("audio dsc", rc);
    return 0;
}

static void step_disc_chrs(void)
{
    s_nchrs = 0;
    int rc = ble_gattc_disc_all_chrs(s_conn, 1, 0xFFFF, chr_cb, NULL);
    if (rc) fail("chr discovery", rc);
}

static int svc_cb(uint16_t conn, const struct ble_gatt_error *err, const struct ble_gatt_svc *svc, void *arg)
{
    if (err->status == 0) {
        if (s_nsvcs < MAX_SVCS) {
            s_svcs[s_nsvcs].start = svc->start_handle;
            s_svcs[s_nsvcs].end = svc->end_handle;
            s_nsvcs++;
        }
        return 0;
    }
    if (err->status != BLE_HS_EDONE) {
        fail("service discovery", err->status);
        return 0;
    }
    step_disc_chrs();
    return 0;
}

static int mtu_cb(uint16_t conn, const struct ble_gatt_error *err, uint16_t mtu, void *arg)
{
    ESP_LOGI(TAG, "MTU %u (status %d)", mtu, err->status);
    s_nsvcs = 0;
    int rc = ble_gattc_disc_all_svcs(s_conn, svc_cb, NULL);
    if (rc) fail("svc discovery", rc);
    return 0;
}

static void on_connected(uint16_t conn)
{
    s_conn = conn;
    s_connected = true;
    s_fw[0] = 0;
    s_proto = OMI_PROTO_UNKNOWN;
    h_audio = h_audio_cccd = h_codec = h_time = h_storage = h_storage_cccd = h_fwrev = 0;
    omi_frame_asm_init(&s_asm);
    s_notifies = 0;
    led_set(LED_STREAMING);
    /* best effort link upgrades; failures are not fatal */
    int rc = ble_gap_set_data_len(conn, 251, 2120);
    if (rc) ESP_LOGW(TAG, "set_data_len rc=%d", rc);
#if CONFIG_BT_NIMBLE_50_FEATURE_SUPPORT
    rc = ble_gap_set_prefered_le_phy(conn, BLE_GAP_LE_PHY_2M_MASK, BLE_GAP_LE_PHY_2M_MASK, BLE_GAP_LE_PHY_CODED_ANY);
    if (rc) ESP_LOGW(TAG, "set 2M PHY rc=%d", rc);
#endif
    rc = ble_gattc_exchange_mtu(conn, mtu_cb, NULL);
    if (rc) fail("mtu exchange", rc);
}

static void on_notify(uint16_t handle, struct os_mbuf *om)
{
    uint8_t buf[260];
    uint16_t n = OS_MBUF_PKTLEN(om);
    if (n > sizeof(buf)) n = sizeof(buf);
    if (os_mbuf_copydata(om, 0, n, buf) != 0) return;
    if (handle == h_audio) {
        size_t fl = 0;
        if (omi_frame_asm_feed(&s_asm, buf, n, s_frame, sizeof(s_frame), &fl) == 1) uplink_live_frame(s_frame, fl);
        if (++s_notifies % 3000 == 0)
            ESP_LOGI(TAG, "audio: %lu notifies, %lu frames, %lu dropped", (unsigned long)s_notifies,
                     (unsigned long)s_asm.frames_out, (unsigned long)s_asm.dropped);
    } else if (handle == h_storage && g_cfg.storage_sync) {
        ring_sync_on_notify(buf, n);
    }
}

static int gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_DISC: {
        const struct ble_gap_disc_desc *d = &event->disc;
        if (mac_matches(&d->addr) && adv_is_omi(d)) {
            ESP_LOGI(TAG, "found pendant rssi=%d, connecting", d->rssi);
            connect_to(&d->addr);
        }
        return 0;
    }
    case BLE_GAP_EVENT_DISC_COMPLETE:
        if (!s_connected) start_scan();
        return 0;
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            ESP_LOGI(TAG, "connected");
            on_connected(event->connect.conn_handle);
        } else {
            ESP_LOGW(TAG, "connect failed status=%d", event->connect.status);
            start_scan();
        }
        return 0;
    case BLE_GAP_EVENT_DISCONNECT: {
        ESP_LOGW(TAG, "disconnected reason=0x%x", event->disconnect.reason);
        size_t fl = 0;
        if (omi_frame_asm_flush(&s_asm, s_frame, sizeof(s_frame), &fl) == 1) uplink_live_frame(s_frame, fl);
        uplink_live_flush();
        ring_sync_on_disconnect();
        s_conn = BLE_HS_CONN_HANDLE_NONE;
        s_connected = false;
        led_set(net_wifi_up() ? LED_IDLE : LED_NO_WIFI);
        start_scan();
        return 0;
    }
    case BLE_GAP_EVENT_NOTIFY_RX:
        on_notify(event->notify_rx.attr_handle, event->notify_rx.om);
        return 0;
    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "mtu update %u", event->mtu.value);
        return 0;
    default:
        return 0;
    }
}

bool ble_connected(void) { return s_connected; }

bool ble_storage_write(const uint8_t *buf, size_t len)
{
    /* hard guard: this is the ONLY path that writes the storage characteristic */
    if (!omi_storage_write_allowed(s_proto, buf, len)) {
        ESP_LOGE(TAG, "BLOCKED storage write cmd 0x%02x len %u", len ? buf[0] : 0, (unsigned)len);
        return false;
    }
    if (!s_connected || !h_storage) return false;
    return ble_gattc_write_flat(s_conn, h_storage, buf, len, NULL, NULL) == 0;
}

static void on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc == 0) rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "address setup rc=%d", rc);
        return;
    }
    start_scan();
}

static void on_reset(int reason) { ESP_LOGW(TAG, "host reset reason=%d", reason); }

static void host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void ble_central_start(void)
{
    ESP_ERROR_CHECK(nimble_port_init());
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.reset_cb = on_reset;
    ble_att_set_preferred_mtu(247);
    nimble_port_freertos_init(host_task);
}
