/*
 * Flash spool: one file per batch on LittleFS (partition "spool").
 *   /spool/q/<seq:010u>.bin   finished record: 16-byte header + body
 *   /spool/q/<seq>.prt        ring-sync stream in progress (recovered at boot)
 *   /spool/bad/               records the server rejected with 400/413/415
 * Header: "OMIS" ver(1) fmt(1) rsv(2) start(u32 LE) len(u32 LE)
 */
#include <dirent.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "bridge.h"
#include "esp_heap_caps.h"
#include "esp_littlefs.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "spool";
#define BASE "/spool"
#define QDIR BASE "/q"
#define BADDIR BASE "/bad"
#define HDR_LEN 16
#define RESERVE_BYTES (64 * 1024) /* keep LittleFS out of the full-disk corner */

static bool s_ready;
static uint32_t s_seq;
static SemaphoreHandle_t s_mu;

struct spool_stream {
    FILE *f;
    char path[64];
    uint8_t fmt;
    uint32_t start;
    size_t len;
};

static void hdr_pack(uint8_t *b, uint8_t fmt, uint32_t start, uint32_t len)
{
    memcpy(b, "OMIS", 4);
    b[4] = 1; b[5] = fmt; b[6] = 0; b[7] = 0;
    for (int i = 0; i < 4; i++) { b[8 + i] = start >> (8 * i); b[12 + i] = len >> (8 * i); }
}

static bool hdr_unpack(const uint8_t *b, spool_hdr_t *h)
{
    if (memcmp(b, "OMIS", 4) || b[4] != 1) return false;
    h->fmt = b[5];
    h->start = b[8] | (b[9] << 8) | (b[10] << 16) | ((uint32_t)b[11] << 24);
    h->len = b[12] | (b[13] << 8) | (b[14] << 16) | ((uint32_t)b[15] << 24);
    return true;
}

static uint32_t next_seq(void)
{
    xSemaphoreTake(s_mu, portMAX_DELAY);
    uint32_t v = ++s_seq;
    xSemaphoreGive(s_mu);
    return v;
}

static bool fsync_file(FILE *f)
{
    if (fflush(f) != 0) return false;
    return fsync(fileno(f)) == 0;
}

/* finalize an interrupted ring stream: keep whole 440-byte blocks that hit flash */
static void recover_prt(const char *prt)
{
    FILE *f = fopen(prt, "r+b");
    if (!f) return;
    uint8_t b[HDR_LEN];
    spool_hdr_t h;
    struct stat st;
    if (fread(b, 1, HDR_LEN, f) != HDR_LEN || !hdr_unpack(b, &h) || stat(prt, &st) != 0) {
        fclose(f);
        ESP_LOGW(TAG, "dropping unreadable %s", prt);
        unlink(prt);
        return;
    }
    size_t body = st.st_size > HDR_LEN ? (size_t)st.st_size - HDR_LEN : 0;
    if (h.fmt == SPOOL_FMT_OMI_SD) body -= body % OMI_STORAGE_BLOCK;
    hdr_pack(b, h.fmt, h.start, (uint32_t)body);
    fseek(f, 0, SEEK_SET);
    fwrite(b, 1, HDR_LEN, f);
    fsync_file(f);
    fclose(f);
    if (body == 0) {
        unlink(prt);
        return;
    }
    char fin[64];
    strlcpy(fin, prt, sizeof(fin));
    strcpy(strrchr(fin, '.'), ".bin");
    rename(prt, fin);
    ESP_LOGW(TAG, "recovered %s (%u bytes)", fin, (unsigned)body);
}

bool spool_init(void)
{
    s_mu = xSemaphoreCreateMutex();
    esp_vfs_littlefs_conf_t conf = {
        .base_path = BASE,
        .partition_label = "spool",
        .format_if_mount_failed = true,
        .dont_mount = false,
    };
    esp_err_t err = esp_vfs_littlefs_register(&conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LittleFS mount failed (%s); running RAM-only", esp_err_to_name(err));
        return false;
    }
    mkdir(QDIR, 0775);
    mkdir(BADDIR, 0775);
    DIR *d = opendir(QDIR);
    int n = 0;
    if (d) {
        struct dirent *e;
        while ((e = readdir(d))) {
            uint32_t v = strtoul(e->d_name, NULL, 10);
            if (v > s_seq) s_seq = v;
            char p[300];
            snprintf(p, sizeof(p), QDIR "/%s", e->d_name);
            if (strstr(e->d_name, ".prt")) recover_prt(p);
            else if (strstr(e->d_name, ".tmp")) unlink(p);  /* never renamed => never acked */
            else n++;
        }
        closedir(d);
    }
    s_ready = true;
    ESP_LOGI(TAG, "spool mounted: %d queued record(s), %u KB free", n, (unsigned)(spool_free_bytes() / 1024));
    return true;
}

bool spool_ready(void) { return s_ready; }

size_t spool_free_bytes(void)
{
    size_t total = 0, used = 0;
    if (!s_ready || esp_littlefs_info("spool", &total, &used) != ESP_OK) return 0;
    size_t f = total > used ? total - used : 0;
    return f > RESERVE_BYTES ? f - RESERVE_BYTES : 0;
}

int spool_count(void)
{
    if (!s_ready) return 0;
    DIR *d = opendir(QDIR);
    if (!d) return 0;
    int n = 0;
    struct dirent *e;
    while ((e = readdir(d))) if (strstr(e->d_name, ".bin")) n++;
    closedir(d);
    return n;
}

bool spool_write(uint8_t fmt, uint32_t start, const uint8_t *data, size_t len)
{
    if (!s_ready || len + HDR_LEN > spool_free_bytes()) return false;
    uint32_t seq = next_seq();
    char tmp[64], fin[64];
    snprintf(tmp, sizeof(tmp), QDIR "/%010lu.tmp", (unsigned long)seq);
    snprintf(fin, sizeof(fin), QDIR "/%010lu.bin", (unsigned long)seq);
    FILE *f = fopen(tmp, "wb");
    if (!f) return false;
    uint8_t h[HDR_LEN];
    hdr_pack(h, fmt, start, (uint32_t)len);
    bool ok = fwrite(h, 1, HDR_LEN, f) == HDR_LEN && fwrite(data, 1, len, f) == len && fsync_file(f);
    fclose(f);
    if (!ok || rename(tmp, fin) != 0) {
        unlink(tmp);
        ESP_LOGE(TAG, "spool write failed (errno %d)", errno);
        return false;
    }
    return true;
}

bool spool_oldest(char *path, size_t cap)
{
    if (!s_ready) return false;
    DIR *d = opendir(QDIR);
    if (!d) return false;
    uint32_t best = UINT32_MAX;
    struct dirent *e;
    while ((e = readdir(d))) {
        if (!strstr(e->d_name, ".bin")) continue;
        uint32_t v = strtoul(e->d_name, NULL, 10);
        if (v < best) best = v;
    }
    closedir(d);
    if (best == UINT32_MAX) return false;
    snprintf(path, cap, QDIR "/%010lu.bin", (unsigned long)best);
    return true;
}

bool spool_read(const char *path, spool_hdr_t *hdr, uint8_t **data)
{
    *data = NULL;
    FILE *f = fopen(path, "rb");
    if (!f) return false;
    uint8_t b[HDR_LEN];
    bool ok = fread(b, 1, HDR_LEN, f) == HDR_LEN && hdr_unpack(b, hdr) && hdr->len > 0 && hdr->len < 8 * 1024 * 1024;
    if (ok) {
        *data = heap_caps_malloc(hdr->len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!*data) *data = malloc(hdr->len);
        ok = *data && fread(*data, 1, hdr->len, f) == hdr->len;
    }
    fclose(f);
    if (!ok) {
        free(*data);
        *data = NULL;
    }
    return ok;
}

void spool_remove(const char *path) { unlink(path); }

void spool_quarantine(const char *path)
{
    char dst[96];
    snprintf(dst, sizeof(dst), BADDIR "/%s", strrchr(path, '/') + 1);
    if (rename(path, dst) != 0) unlink(path);
}

spool_stream_t *spool_stream_open(uint8_t fmt, uint32_t start)
{
    if (!s_ready) return NULL;
    spool_stream_t *s = calloc(1, sizeof(*s));
    if (!s) return NULL;
    snprintf(s->path, sizeof(s->path), QDIR "/%010lu.prt", (unsigned long)next_seq());
    s->f = fopen(s->path, "wb");
    if (!s->f) {
        free(s);
        return NULL;
    }
    s->fmt = fmt;
    s->start = start;
    uint8_t h[HDR_LEN];
    hdr_pack(h, fmt, start, 0);
    fwrite(h, 1, HDR_LEN, s->f);
    return s;
}

bool spool_stream_append(spool_stream_t *s, const uint8_t *d, size_t n)
{
    if (!s || fwrite(d, 1, n, s->f) != n) return false;
    s->len += n;
    return true;
}

bool spool_stream_sync(spool_stream_t *s) { return s && fsync_file(s->f); }

size_t spool_stream_len(spool_stream_t *s) { return s ? s->len : 0; }

bool spool_stream_close(spool_stream_t *s)
{
    if (!s) return false;
    uint8_t h[HDR_LEN];
    hdr_pack(h, s->fmt, s->start, (uint32_t)s->len);
    fseek(s->f, 0, SEEK_SET);
    fwrite(h, 1, HDR_LEN, s->f);
    bool ok = fsync_file(s->f);
    fclose(s->f);
    if (s->len == 0) {
        unlink(s->path);
    } else {
        char fin[64];
        strlcpy(fin, s->path, sizeof(fin));
        strcpy(strrchr(fin, '.'), ".bin");
        ok = ok && rename(s->path, fin) == 0;
    }
    free(s);
    if (ok) uplink_kick();
    return ok;
}
