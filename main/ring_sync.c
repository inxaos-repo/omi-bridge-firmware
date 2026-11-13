/*
 * Pendant storage sync.
 *
 * fw 3.0.24 (INFO carries a ring id): app custody, always on. Protocol: see
 * omi-monologue-pipeline docs/omi-ble-protocol.md. INFO -> CUSTODY_ENABLE ->
 * INFO (backlog_end = write_seq) -> READ(read_seq, n) -> DONE -> POST omi-sd to
 * ingest -> ADVANCE_ID only on HTTP 2xx -> repeat until backlog_end. Custody data
 * is held in PSRAM only and is NEVER spilled to flash or acked on a flash write:
 * a failed POST simply means the pendant keeps it and we re-read later. Once the
 * backlog is drained, LIVE_MARKs (live-persist) are acked when live audio through
 * the mark's packet id has been ingested (2xx) with no spill/drop this connection.
 *
 * fw 3.0.21 legacy ring protocol below: OFF unless g_cfg.storage_sync.
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

/* ---- 3.0.24 custody ---- */
typedef enum { C_OFF, C_PROBE, C_ENABLE, C_INFO, C_READING, C_ADVANCING, C_IDLE } cstate_t;
#define MAX_MARKS 32
#define CUSTODY_CAPS (OMI_CAP_APP_ACK_RECLAIM | OMI_CAP_LIVE_PERSIST | OMI_CAP_ADVANCE_IDEMPOTENT | OMI_CAP_RING_ID)
typedef struct { uint64_t seq; uint16_t live_index; } mark_t;
static struct {
    cstate_t st;
    bool legacy;            /* pendant is 3.0.21: legacy path */
    uint32_t gen;
    uint64_t ring_id;
    uint8_t caps;
    bool backlog_end_set, backlog_done;
    uint64_t backlog_end, acked, target;
    uint64_t rd_start;
    uint32_t rd_req, rd_pkts, rd_ts;
    bool rd_bad;
    uint8_t *batch;
    size_t batch_cap;
    omi_ring_asm_t asm_;
    mark_t marks[MAX_MARKS];
    int nmarks;
    bool live_gap, uploaded_any;
    uint32_t uploaded_pid;
    int64_t retry_after_us;
} C;
static volatile bool s_lost;  /* a storage notification was dropped */

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

static void c_send_info(cstate_t next)
{
    C.st = next;
    request_info();
}

static void c_advance(uint64_t seq)
{
    uint8_t b[17];
    C.target = seq;
    C.st = C_ADVANCING;
    ESP_LOGI(TAG, "ADVANCE_ID seq=%llu", (unsigned long long)seq);
    send_cmd(b, omi_ring_cmd_advance_id(b, C.ring_id, seq));
}

static void c_try_ack_live(void)
{
    if (C.st != C_IDLE || !(C.caps & OMI_CAP_LIVE_PERSIST) || !C.backlog_done || C.live_gap || !C.uploaded_any) return;
    uint64_t best = 0;
    int k = 0;
    for (int i = 0; i < C.nmarks; i++) {
        uint32_t p = omi_pid_near(C.uploaded_pid, C.marks[i].live_index);
        if ((int32_t)(p - C.uploaded_pid) <= 0) {
            if (C.marks[i].seq > best) best = C.marks[i].seq;
        } else {
            C.marks[k++] = C.marks[i];
        }
    }
    C.nmarks = k;
    if (best > C.acked) c_advance(best);
}

static void c_on_block(uint32_t ts, const uint8_t block[OMI_STORAGE_BLOCK], void *ctx)
{
    (void)ctx;
    if (C.rd_pkts == 0) C.rd_ts = ts;
    if ((size_t)(C.rd_pkts + 1) * OMI_STORAGE_BLOCK > C.batch_cap) { C.rd_bad = true; return; }
    memcpy(C.batch + (size_t)C.rd_pkts * OMI_STORAGE_BLOCK, block, OMI_STORAGE_BLOCK);
    C.rd_pkts++;
}

static void c_on_info(const omi_ring_notify_t *m)
{
    ESP_LOGI(TAG, "INFO(custody) read=%llu write=%llu end=%llu ring=%016llx",
             (unsigned long long)m->read_seq, (unsigned long long)m->write_seq,
             (unsigned long long)C.backlog_end, (unsigned long long)m->ring_id);
    if (m->ring_id != C.ring_id) {
        ESP_LOGW(TAG, "ring id changed: resetting custody session");
        C.ring_id = m->ring_id;
        C.backlog_end_set = C.backlog_done = false;
        C.nmarks = 0;
        C.acked = 0;
    }
    if (!C.backlog_end_set) { C.backlog_end = m->write_seq; C.backlog_end_set = true; }
    if (m->read_seq > C.acked) C.acked = m->read_seq;
    if (m->read_seq >= C.backlog_end) {
        if (!C.backlog_done) ESP_LOGI(TAG, "backlog drained");
        C.backlog_done = true;
        C.st = C_IDLE;
        c_try_ack_live();
        return;
    }
    C.st = C_IDLE;
    if (!C.batch || m->packet_bytes != OMI_RING_PACKET || !net_wifi_up() ||
        esp_timer_get_time() < C.retry_after_us) return;  /* REINFO retries */
    uint64_t left = C.backlog_end - m->read_seq;
    uint32_t n = (uint32_t)(C.batch_cap / OMI_STORAGE_BLOCK);
    if (n > CONFIG_OMI_RING_READ_CHUNK) n = CONFIG_OMI_RING_READ_CHUNK;
    if (left < n) n = (uint32_t)left;
    C.rd_start = m->read_seq;
    C.rd_req = n;
    C.rd_pkts = 0;
    C.rd_bad = false;
    s_lost = false;
    omi_ring_asm_init(&C.asm_);
    C.st = C_READING;
    uint8_t b[13];
    ESP_LOGI(TAG, "RING_READ seq=%llu count=%lu (custody: nothing freed until ingest 2xx)",
             (unsigned long long)C.rd_start, (unsigned long)n);
    send_cmd(b, omi_ring_cmd_read(b, C.rd_start, n));
}

static void c_on_done(void)
{
    uint64_t tgt;
    C.st = C_IDLE;
    if (C.rd_bad || s_lost || C.asm_.len || !omi_custody_target(C.rd_start, C.rd_pkts, C.rd_req, &tgt)) {
        ESP_LOGW(TAG, "custody read incomplete (pkts=%lu bad=%d lost=%d): not acking, retry later",
                 (unsigned long)C.rd_pkts, C.rd_bad, (int)s_lost);
        C.retry_after_us = esp_timer_get_time() + REINFO_US / 2;
        return;
    }
    uint32_t ts = C.rd_ts;
    if ((ts & 0x80000000u) || ts < 1600000000u) ts = net_time_valid() ? net_epoch() : 0;
    int status = uplink_post(SPOOL_FMT_OMI_SD, ts, C.batch, (size_t)C.rd_pkts * OMI_STORAGE_BLOCK);
    if (status < 200 || status > 299) {
        ESP_LOGW(TAG, "backlog POST status %d: pendant keeps the data, retry later", status);
        C.retry_after_us = esp_timer_get_time() + REINFO_US / 2;
        return;
    }
    c_advance(tgt);
}

static void c_handle(const omi_ring_notify_t *m)
{
    switch (m->type) {
    case OMI_RING_NOTIFY_INFO:
        if (C.st == C_PROBE) {
            if (m->has_ring_id && m->custody_version >= OMI_CUSTODY_VERSION) {
                uint8_t b[3];
                C.ring_id = m->ring_id;
                C.st = C_ENABLE;
                ESP_LOGI(TAG, "pendant offers custody v%u caps=0x%02x", m->custody_version, m->caps);
                send_cmd(b, omi_ring_cmd_custody(b, CUSTODY_CAPS));
            } else {
                C.st = C_OFF;
                C.legacy = true;
                if (g_cfg.storage_sync) maybe_read(m);
                else ESP_LOGI(TAG, "pendant is pre-3.0.24 and storage_sync is off: live only");
            }
        } else if (C.st == C_INFO || C.st == C_IDLE) {
            c_on_info(m);
        }
        break;
    case OMI_RING_NOTIFY_ACK:
        if (C.st == C_ENABLE) {
            if (m->status != OMI_RING_ST_OK) {
                ESP_LOGW(TAG, "CUSTODY_ENABLE refused (%u): live only", m->status);
                C.st = C_OFF;
                break;
            }
            C.caps = m->caps;
            ESP_LOGI(TAG, "custody granted caps=0x%02x", C.caps);
            c_send_info(C_INFO);
        } else if (C.st == C_ADVANCING) {
            if (m->status == OMI_RING_ST_OK) {
                if (C.target > C.acked) C.acked = C.target;
            } else if (m->status == OMI_RING_ST_ID_MISMATCH) {
                ESP_LOGW(TAG, "ADVANCE_ID ring id mismatch: re-sync");
                C.backlog_done = C.backlog_end_set = false;
                C.nmarks = 0;
            } else {
                ESP_LOGW(TAG, "ADVANCE_ID status %u", m->status);
            }
            c_send_info(C_INFO);
        } else if (C.st == C_READING && m->status) {
            ESP_LOGW(TAG, "READ refused (%u)", m->status);
            C.st = C_IDLE;
            C.retry_after_us = esp_timer_get_time() + REINFO_US / 2;
        }
        break;
    case OMI_RING_NOTIFY_BEGIN:
        if (C.st == C_READING && m->seq != C.rd_start) C.rd_bad = true;
        break;
    case OMI_RING_NOTIFY_DATA:
        if (C.st == C_READING) omi_ring_asm_feed(&C.asm_, m->data, m->data_len, c_on_block, NULL);
        break;
    case OMI_RING_NOTIFY_DONE:
        if (C.st == C_READING) c_on_done();
        break;
    case OMI_RING_NOTIFY_LIVE_MARK:
        if (C.caps && m->ring_id == C.ring_id) {
            if (C.nmarks == MAX_MARKS) { memmove(C.marks, C.marks + 1, sizeof(mark_t) * (MAX_MARKS - 1)); C.nmarks--; }
            C.marks[C.nmarks++] = (mark_t){m->seq, m->live_index};
            c_try_ack_live();
        }
        break;
    }
}

static void handle(const uint8_t *d, size_t n)
{
    if (n == 10 && d[0] == CTL_MARK && (d[1] == 'U' || d[1] == 'G')) {
        uint32_t gen, pid;
        memcpy(&gen, d + 2, 4);
        memcpy(&pid, d + 6, 4);
        if (gen != C.gen) return;
        if (d[1] == 'G') {
            if (!C.live_gap) ESP_LOGW(TAG, "live audio gap: live marks will not be acked this connection");
            C.live_gap = true;
        } else if (!C.uploaded_any || (int32_t)(pid - C.uploaded_pid) > 0) {
            C.uploaded_pid = pid;
            C.uploaded_any = true;
            c_try_ack_live();
        }
        return;
    }
    if (n >= 2 && d[0] == CTL_MARK) {
        if (d[1] == 'R') {
            memset(&S, 0, sizeof(S));
            S.active = true;
            uint8_t *batch = C.batch;
            size_t cap = C.batch_cap;
            memset(&C, 0, sizeof(C));
            C.batch = batch;
            C.batch_cap = cap;
            C.gen = ble_conn_gen();
            c_send_info(C_PROBE);
        } else if (d[1] == 'D') {
            if (S.asm_.len) ESP_LOGW(TAG, "disconnect with %u B partial packet", (unsigned)S.asm_.len);
            close_stream();
            S.active = false;
            S.reading = false;
            C.st = C_OFF;
        }
        return;
    }
    if (!S.active) return;
    omi_ring_notify_t m;
    if (omi_parse_ring_notify(d, n, &m) != 0) {
        ESP_LOGW(TAG, "bad notify type 0x%02x len %u", n ? d[0] : 0, (unsigned)n);
        return;
    }
    if (!C.legacy) { c_handle(&m); return; }
    if (!g_cfg.storage_sync) return;
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
        } else if (S.active && !C.legacy && C.st == C_IDLE && !C.backlog_done &&
                   esp_timer_get_time() - S.last_info_us > REINFO_US / 2) {
            c_send_info(C_INFO);
        } else if (S.active && C.legacy && g_cfg.storage_sync && !S.reading &&
                   esp_timer_get_time() - S.last_info_us > REINFO_US) {
            request_info();
        }
    }
}

void ring_sync_init(void)
{
    size_t sz = g_sizing.ring_buf;
    uint8_t *mem = heap_caps_malloc(sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!mem) {
        sz = 32 * 1024;
        mem = heap_caps_malloc(sz, MALLOC_CAP_8BIT);
    }
    configASSERT(mem);
    s_rb = xRingbufferCreateStatic(sz, RINGBUF_TYPE_NOSPLIT, mem, &s_rb_struct);
    configASSERT(s_rb);
    C.batch_cap = g_sizing.custody_batch;
    C.batch = heap_caps_malloc(C.batch_cap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!C.batch) {
        C.batch_cap = 16 * 1024;
        C.batch = heap_caps_malloc(C.batch_cap, MALLOC_CAP_8BIT);
    }
    if (!C.batch) ESP_LOGE(TAG, "no custody batch buffer: backlog sync disabled");
    xTaskCreatePinnedToCore(ring_task, "ring_sync", 6144, NULL, 4, NULL, 1);
    ESP_LOGI(TAG, "3.0.24 custody sync on (batch %u KB)", (unsigned)(C.batch_cap / 1024));
    if (g_cfg.storage_sync) ESP_LOGW(TAG, "3.0.21 legacy storage sync ENABLED: reading a 3.0.21 ring consumes its data");
}

static void push(const uint8_t *d, size_t n)
{
    if (!s_rb) return;
    if (xRingbufferSend(s_rb, d, n, 0) != pdTRUE) {
        s_lost = true;
        ESP_LOGE(TAG, "ring buffer full: storage notification LOST");
    }
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

static void push_live(char kind, uint32_t gen, uint32_t pid)
{
    uint8_t m[10] = {CTL_MARK, (uint8_t)kind};
    memcpy(m + 2, &gen, 4);
    memcpy(m + 6, &pid, 4);
    if (s_rb) xRingbufferSend(s_rb, m, sizeof(m), pdMS_TO_TICKS(50));
}

void ring_sync_live_uploaded(uint32_t gen, uint32_t pid) { push_live('U', gen, pid); }
void ring_sync_live_gap(uint32_t gen) { push_live('G', gen, 0); }
