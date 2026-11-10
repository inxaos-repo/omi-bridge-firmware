/*
 * Pendant storage sync, firmware 3.0.21 ring protocol. OFF unless g_cfg.storage_sync.
 *
 * WARNING: on 3.0.21 the pendant frees ring data as it is SENT (tx-complete
 * checkpointing), not when the host acknowledges. To bound the exposure this code:
 *  - only issues RING_READ with a bounded count, and only if the flash spool has
 *    room for the whole chunk (plus margin);
 *  - writes every 444-byte packet (ts stripped -> 440-byte omi-sd block) to the
 *    flash spool and fsyncs BEFORE sending RING_ADVANCE for it;
 *  - never sends RING_CLEAR (no builder exists; ble_storage_write() guard rejects it).
 *
 * BLE host task -> (PSRAM ring buffer) -> this task, so flash I/O never blocks the host.
 */
#include <string.h>

#include "bridge.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "ring";

#define SYNC_EVERY_PKTS   32
#define STREAM_MAX_BYTES  (600 * OMI_STORAGE_BLOCK)  /* ~264 KB, ~1 min audio */
#define TS_JUMP_S         30
#define REINFO_US         (60LL * 1000000)
#define CTL_MARK          0xFF

static RingbufHandle_t s_rb;
static StaticRingbuffer_t s_rb_struct;

typedef struct {
    bool active;          /* subscribed & ready */
    bool reading;         /* READ outstanding */
    uint64_t start_seq;   /* first packet of current READ */
    uint64_t persisted;   /* packets fsync'd since start_seq */
    uint64_t advanced;    /* packets ADVANCE'd since start_seq */
    uint32_t pending;     /* packets appended since last fsync */
    uint32_t last_ts;
    spool_stream_t *stream;
    omi_ring_asm_t asm_;
    int64_t last_info_us;
    bool spool_full;
} ring_state_t;

static ring_state_t S;

static void send_cmd(const uint8_t *b, size_t n)
{
    if (!ble_storage_write(b, n)) ESP_LOGW(TAG, "storage cmd 0x%02x not sent", b[0]);
}

static void request_info(void)
{
    uint8_t b[1];
    send_cmd(b, omi_ring_cmd_info(b));
    S.last_info_us = esp_timer_get_time();
}

static bool persist_and_advance(void)
{
    if (S.pending && S.stream) {
        if (!spool_stream_sync(S.stream)) {
            ESP_LOGE(TAG, "fsync failed; NOT advancing");
            return false;
        }
        S.persisted += S.pending;
        S.pending = 0;
    }
    if (S.persisted > S.advanced) {
        uint8_t b[9];
        send_cmd(b, omi_ring_cmd_advance(b, S.start_seq + S.persisted));
        S.advanced = S.persisted;
    }
    return true;
}

static void close_stream(void)
{
    if (!S.stream) return;
    persist_and_advance();
    size_t len = spool_stream_len(S.stream);
    if (!spool_stream_close(S.stream)) ESP_LOGE(TAG, "stream close failed (recovered at next boot)");
    else ESP_LOGI(TAG, "spooled %u B of pendant storage", (unsigned)len);
    S.stream = NULL;
}

static void on_packet(uint32_t ts, const uint8_t block[OMI_STORAGE_BLOCK], void *ctx)
{
    if (S.spool_full) return;
    bool jump = S.stream && (ts < S.last_ts || ts > S.last_ts + TS_JUMP_S);
    if (S.stream && (jump || spool_stream_len(S.stream) >= STREAM_MAX_BYTES)) close_stream();
    if (!S.stream) {
        if (spool_free_bytes() < 4 * OMI_STORAGE_BLOCK * SYNC_EVERY_PKTS) {
            S.spool_full = true;
            ESP_LOGE(TAG, "spool full mid-read: sending STOP (packets already sent by pendant are lost)");
            uint8_t b[1];
            send_cmd(b, omi_ring_cmd_stop(b));
            return;
        }
        S.stream = spool_stream_open(SPOOL_FMT_OMI_SD, ts);
        if (!S.stream) {
            S.spool_full = true;
            uint8_t b[1];
            send_cmd(b, omi_ring_cmd_stop(b));
            return;
        }
    }
    S.last_ts = ts;
    if (!spool_stream_append(S.stream, block, OMI_STORAGE_BLOCK)) {
        S.spool_full = true;
        uint8_t b[1];
        send_cmd(b, omi_ring_cmd_stop(b));
        return;
    }
    if (++S.pending >= SYNC_EVERY_PKTS) persist_and_advance();
}

static void maybe_read(const omi_ring_notify_t *info)
{
    uint64_t unread = omi_ring_unread(info);
    ESP_LOGI(TAG, "INFO read=%llu write=%llu unread=%llu cap=%lu dropped=%llu pkt=%u",
             (unsigned long long)info->read_seq, (unsigned long long)info->write_seq, (unsigned long long)unread,
             (unsigned long)info->capacity_packets, (unsigned long long)info->dropped_packets, info->packet_bytes);
    if (info->packet_bytes != OMI_RING_PACKET) {
        ESP_LOGE(TAG, "unexpected packet size %u; not reading", info->packet_bytes);
        return;
    }
    if (!unread || S.reading) return;
    uint32_t count = unread > CONFIG_OMI_RING_READ_CHUNK ? CONFIG_OMI_RING_READ_CHUNK : (uint32_t)unread;
    size_t need = (size_t)count * OMI_STORAGE_BLOCK * 2 + 64 * 1024;
    if (!spool_ready() || spool_free_bytes() < need) {
        ESP_LOGW(TAG, "spool has %u KB free, need %u KB: not reading", (unsigned)(spool_free_bytes() / 1024), (unsigned)(need / 1024));
        return;
    }
    S.reading = true;
    S.spool_full = false;
    S.start_seq = info->read_seq;
    S.persisted = S.advanced = 0;
    S.pending = 0;
    omi_ring_asm_init(&S.asm_);
    uint8_t b[13];
    ESP_LOGW(TAG, "RING_READ seq=%llu count=%lu (consumes pendant data)", (unsigned long long)S.start_seq, (unsigned long)count);
    send_cmd(b, omi_ring_cmd_read(b, S.start_seq, count));
}

static void handle(const uint8_t *d, size_t n)
{
    if (n >= 2 && d[0] == CTL_MARK) {
        if (d[1] == 'R') {
            memset(&S, 0, sizeof(S));
            S.active = true;
            request_info();
        } else if (d[1] == 'D') {
            if (S.asm_.len) ESP_LOGW(TAG, "disconnect with %u B partial packet", (unsigned)S.asm_.len);
            close_stream();
            S.active = false;
            S.reading = false;
        }
        return;
    }
    if (!S.active) return;
    omi_ring_notify_t m;
    if (omi_parse_ring_notify(d, n, &m) != 0) {
        ESP_LOGW(TAG, "bad notify type 0x%02x len %u", n ? d[0] : 0, (unsigned)n);
        return;
    }
    switch (m.type) {
    case OMI_RING_NOTIFY_INFO: maybe_read(&m); break;
    case OMI_RING_NOTIFY_BEGIN:
        if (m.seq != S.start_seq) ESP_LOGW(TAG, "BEGIN seq %llu != requested %llu", (unsigned long long)m.seq, (unsigned long long)S.start_seq);
        S.start_seq = m.seq;
        break;
    case OMI_RING_NOTIFY_DATA: omi_ring_asm_feed(&S.asm_, m.data, m.data_len, on_packet, NULL); break;
    case OMI_RING_NOTIFY_DONE:
        ESP_LOGI(TAG, "DONE status=%u next=%llu packets=%lu", m.status, (unsigned long long)m.seq, (unsigned long)S.asm_.packets);
        close_stream();
        S.reading = false;
        if (!S.spool_full) request_info();  /* loop until drained */
        break;
    case OMI_RING_NOTIFY_ACK:
        if (m.status) ESP_LOGW(TAG, "ACK status %u", m.status);
        if (m.status && S.reading) { close_stream(); S.reading = false; }
        break;
    }
}

static void ring_task(void *arg)
{
    for (;;) {
        size_t n = 0;
        uint8_t *item = xRingbufferReceive(s_rb, &n, pdMS_TO_TICKS(1000));
        if (item) {
            handle(item, n);
            vRingbufferReturnItem(s_rb, item);
        } else if (S.active && !S.reading && esp_timer_get_time() - S.last_info_us > REINFO_US) {
            request_info();
        }
    }
}

void ring_sync_init(void)
{
    if (!g_cfg.storage_sync) return;
    size_t sz = g_sizing.ring_buf;
    uint8_t *mem = heap_caps_malloc(sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!mem) {
        sz = 32 * 1024;
        mem = heap_caps_malloc(sz, MALLOC_CAP_8BIT);
    }
    configASSERT(mem);
    s_rb = xRingbufferCreateStatic(sz, RINGBUF_TYPE_NOSPLIT, mem, &s_rb_struct);
    configASSERT(s_rb);
    xTaskCreatePinnedToCore(ring_task, "ring_sync", 6144, NULL, 4, NULL, 1);
    ESP_LOGW(TAG, "storage sync ENABLED: reading the pendant ring consumes its data");
}

static void push(const uint8_t *d, size_t n)
{
    if (!s_rb) return;
    if (xRingbufferSend(s_rb, d, n, 0) != pdTRUE) ESP_LOGE(TAG, "ring buffer full: storage notification LOST");
}

void ring_sync_on_notify(const uint8_t *d, size_t len) { push(d, len); }

void ring_sync_on_ready(omi_storage_proto_t proto)
{
    if (proto != OMI_PROTO_RING) return;
    const uint8_t m[2] = {CTL_MARK, 'R'};
    push(m, 2);
}

void ring_sync_on_disconnect(void)
{
    const uint8_t m[2] = {CTL_MARK, 'D'};
    push(m, 2);
}
