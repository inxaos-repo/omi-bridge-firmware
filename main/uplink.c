/*
 * Uplink: live frames -> lp16 batches (PSRAM) -> RAM queue -> POST /v1/frames.
 * Anything that can't be uploaded promptly is spilled to the flash spool, and the
 * flash spool (older data) is always drained before the RAM queue.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bridge.h"
#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "uplink";

#define LIVE_CAP        (96 * 1024)  /* 10 s of CV1 Opus is ~41 KB incl. prefixes */
#define LIVE_IDLE_MS    2000         /* close a batch if no frame for this long */
#define MAX_RAM_ITEMS   256

typedef struct item {
    struct item *next;
    uint8_t fmt;
    uint32_t start;      /* epoch, 0 if time was not valid yet */
    int64_t start_us;    /* esp_timer time of first frame (to backfill start) */
    int64_t queued_us;
    uint32_t live_pid;   /* unwrapped pid of last live packet in batch */
    uint32_t gen;        /* BLE connection generation (custody live marks) */
    size_t len;
    uint8_t data[];      /* PSRAM */
} item_t;

static SemaphoreHandle_t s_mu;
static TaskHandle_t s_task;
static item_t *s_head, *s_tail;
static size_t s_ram_bytes;
static int s_ram_items;

/* live batch under construction (protected by s_mu) */
static uint8_t *s_live;
static size_t s_live_len;
static int s_live_frames;
static uint32_t s_live_start;
static int64_t s_live_start_us, s_live_last_us;
static uint32_t s_live_pid, s_live_gen;

static volatile bool s_backlog;
static uint32_t s_ok_posts, s_failed_posts, s_lost_batches;

static void *ps_alloc(size_t n)
{
    void *p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    return p ? p : malloc(n);
}

static uint32_t resolve_start(uint32_t start, int64_t start_us)
{
    if (start) return start;
    if (!net_time_valid()) return 0;
    int64_t age_s = (esp_timer_get_time() - start_us) / 1000000;
    return net_epoch() - (uint32_t)age_s;
}

/* caller holds s_mu */
static void live_finalize_locked(void)
{
    if (!s_live || s_live_frames == 0) return;
    item_t *it = ps_alloc(sizeof(item_t) + s_live_len);
    if (!it) {
        s_lost_batches++;
        ring_sync_live_gap(s_live_gen);
        ESP_LOGE(TAG, "OOM: dropped live batch (%d frames)", s_live_frames);
    } else {
        it->next = NULL;
        it->fmt = SPOOL_FMT_LP16;
        it->start = s_live_start;
        it->start_us = s_live_start_us;
        it->queued_us = esp_timer_get_time();
        it->live_pid = s_live_pid;
        it->gen = s_live_gen;
        it->len = s_live_len;
        memcpy(it->data, s_live, s_live_len);
        if (s_tail) s_tail->next = it; else s_head = it;
        s_tail = it;
        s_ram_bytes += it->len;
        s_ram_items++;
    }
    s_live_len = 0;
    s_live_frames = 0;
    if (s_task) xTaskNotifyGive(s_task);
}

void uplink_live_frame(const uint8_t *frame, size_t n, uint32_t pid, uint32_t gen)
{
    int64_t now = esp_timer_get_time();
    xSemaphoreTake(s_mu, portMAX_DELAY);
    if (s_live_frames && (gen != s_live_gen ||
        (now - s_live_start_us) / 1000 >= CONFIG_OMI_BATCH_MS || s_live_len + 2 + n > LIVE_CAP))) {
        live_finalize_locked();
    }
    if (s_live_frames == 0) {
        s_live_start_us = now;
        s_live_start = net_time_valid() ? net_epoch() : 0;
    }
    size_t u = omi_lp16_append(s_live, LIVE_CAP, s_live_len, frame, n);
    if (u) {
        s_live_len = u;
        s_live_frames++;
        s_live_last_us = now;
        s_live_pid = pid;
        s_live_gen = gen;
    } else {
        ring_sync_live_gap(gen);
    }
    xSemaphoreGive(s_mu);
}

void uplink_live_flush(void)
{
    xSemaphoreTake(s_mu, portMAX_DELAY);
    live_finalize_locked();
    xSemaphoreGive(s_mu);
}

void uplink_kick(void)
{
    if (s_task) xTaskNotifyGive(s_task);
}

bool uplink_backlogged(void) { return s_backlog; }

/* POST one batch. Returns HTTP status, or -1 on transport error. */
int uplink_post(uint8_t fmt, uint32_t start, const uint8_t *data, size_t len)
{
    char url[320];
    snprintf(url, sizeof(url), "%s/v1/frames?device=%s&start=%lu&format=%s&codec=21",
             g_cfg.ingest_url, g_cfg.device_id, (unsigned long)start,
             fmt == SPOOL_FMT_OMI_SD ? "omi-sd" : "lp16");
    esp_http_client_config_t hc = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 20000,
        .buffer_size = 1024,
    };
    esp_http_client_handle_t c = esp_http_client_init(&hc);
    if (!c) return -1;
    esp_http_client_set_header(c, "Content-Type", "application/octet-stream");
    if (g_cfg.token[0]) {
        char auth[150];
        snprintf(auth, sizeof(auth), "Bearer %s", g_cfg.token);
        esp_http_client_set_header(c, "Authorization", auth);
    }
    esp_http_client_set_post_field(c, (const char *)data, (int)len);
    esp_err_t err = esp_http_client_perform(c);
    int status = err == ESP_OK ? esp_http_client_get_status_code(c) : -1;
    esp_http_client_cleanup(c);
    if (status == 201) {
        s_ok_posts++;
        ESP_LOGI(TAG, "POST %s %u B start=%lu -> 201", fmt ? "omi-sd" : "lp16", (unsigned)len, (unsigned long)start);
    } else {
        s_failed_posts++;
        ESP_LOGW(TAG, "POST %u B -> %d (%s)", (unsigned)len, status, err == ESP_OK ? "http" : esp_err_to_name(err));
    }
    return status;
}

static bool permanent_reject(int status) { return status == 400 || status == 413 || status == 415; }

/* pop RAM head (caller frees) */
static item_t *pop_head(void)
{
    xSemaphoreTake(s_mu, portMAX_DELAY);
    item_t *it = s_head;
    if (it) {
        s_head = it->next;
        if (!s_head) s_tail = NULL;
        s_ram_bytes -= it->len;
        s_ram_items--;
    }
    xSemaphoreGive(s_mu);
    return it;
}

static void push_front(item_t *it)
{
    xSemaphoreTake(s_mu, portMAX_DELAY);
    it->next = s_head;
    s_head = it;
    if (!s_tail) s_tail = it;
    s_ram_bytes += it->len;
    s_ram_items++;
    xSemaphoreGive(s_mu);
}

/* move old / excess RAM items to flash so a reboot or long outage loses nothing */
static void maybe_spill(void)
{
    if (!spool_ready()) return;
    for (;;) {
        xSemaphoreTake(s_mu, portMAX_DELAY);
        item_t *h = s_head;
        bool spill = h && (s_ram_bytes > g_sizing.ram_queue_soft || s_ram_items > MAX_RAM_ITEMS ||
                           (esp_timer_get_time() - h->queued_us) / 1000000 >= CONFIG_OMI_SPILL_AFTER_S);
        xSemaphoreGive(s_mu);
        if (!spill) return;
        item_t *it = pop_head();
        if (!it) return;
        uint32_t start = resolve_start(it->start, it->start_us);
        if (spool_write(it->fmt, start, it->data, it->len)) {
            ESP_LOGW(TAG, "spilled %u B batch to flash", (unsigned)it->len);
            if (it->fmt == SPOOL_FMT_LP16) ring_sync_live_gap(it->gen); /* not ingested yet: stop live acks */
            free(it);
        } else {
            push_front(it);   /* flash full: keep in RAM; drop only if RAM is over hard cap */
            xSemaphoreTake(s_mu, portMAX_DELAY);
            bool over = s_ram_bytes > g_sizing.ram_queue_hard;
            xSemaphoreGive(s_mu);
            if (over) {
                item_t *drop = pop_head();
                s_lost_batches++;
                if (drop) ring_sync_live_gap(drop->gen);
                ESP_LOGE(TAG, "RAM+flash full: DROPPED oldest batch (%u B)", drop ? (unsigned)drop->len : 0);
                free(drop);
            }
            return;
        }
    }
}

static void uplink_task(void *arg)
{
    uint32_t backoff_ms = 1000;
    int64_t last_stats = 0;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500));

        /* close an idle live batch so start times stay accurate */
        xSemaphoreTake(s_mu, portMAX_DELAY);
        if (s_live_frames && (esp_timer_get_time() - s_live_last_us) / 1000 >= LIVE_IDLE_MS) live_finalize_locked();
        xSemaphoreGive(s_mu);

        maybe_spill();

        int spooled = spool_count();
        s_backlog = spooled > 0 || s_ram_items > 2;
        led_set(!net_wifi_up() ? LED_NO_WIFI : spooled > 0 ? LED_SPOOLING : s_ram_items > 2 ? LED_BACKLOG
                : ble_connected() ? LED_STREAMING : LED_IDLE);

        int64_t now = esp_timer_get_time();
        if (now - last_stats > 60LL * 1000000) {
            last_stats = now;
            ESP_LOGI(TAG, "health: wifi=%d ble=%d ram_q=%d (%u KB) spool=%d free=%u KB ok=%lu fail=%lu lost=%lu psram_free=%u KB",
                     net_wifi_up(), ble_connected(), s_ram_items, (unsigned)(s_ram_bytes / 1024), spooled,
                     (unsigned)(spool_free_bytes() / 1024), (unsigned long)s_ok_posts, (unsigned long)s_failed_posts,
                     (unsigned long)s_lost_batches, (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));
        }

        if (!net_wifi_up()) continue;

        int status = 0;
        char path[64];
        if (spool_oldest(path, sizeof(path))) {
            spool_hdr_t h;
            uint8_t *data;
            if (!spool_read(path, &h, &data)) {
                ESP_LOGE(TAG, "unreadable spool record %s, quarantined", path);
                spool_quarantine(path);
                continue;
            }
            uint32_t start = h.start ? h.start : (net_time_valid() ? net_epoch() : 0);
            status = uplink_post(h.fmt, start, data, h.len);
            free(data);
            if (status == 201) spool_remove(path);
            else if (permanent_reject(status)) spool_quarantine(path);
        } else {
            item_t *it = pop_head();
            if (!it) continue;
            status = uplink_post(it->fmt, resolve_start(it->start, it->start_us), it->data, it->len);
            if (status == 201) {
                if (it->fmt == SPOOL_FMT_LP16) ring_sync_live_uploaded(it->gen, it->live_pid);
                free(it);
            } else if (permanent_reject(status)) {
                /* server says the batch itself is bad: retrying can't help */
                s_lost_batches++;
                ring_sync_live_gap(it->gen);
                ESP_LOGE(TAG, "server rejected live batch (%d), dropped %u B", status, (unsigned)it->len);
                free(it);
            } else {
                push_front(it);
            }
        }

        if (status == 201 || permanent_reject(status)) {
            backoff_ms = 1000;
            xTaskNotifyGive(s_task);   /* keep draining */
        } else {
            vTaskDelay(pdMS_TO_TICKS(backoff_ms));
            backoff_ms = backoff_ms * 2 > 30000 ? 30000 : backoff_ms * 2;
        }
    }
}

void uplink_init(void)
{
    s_mu = xSemaphoreCreateMutex();
    s_live = ps_alloc(LIVE_CAP);
    configASSERT(s_live);
}

void uplink_start(void)
{
    xTaskCreatePinnedToCore(uplink_task, "uplink", 8192, NULL, 5, &s_task, 1);
}
