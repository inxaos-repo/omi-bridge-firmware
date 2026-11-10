/* Shared interfaces between bridge modules. */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "omi_proto.h"
#include "omi_sizing.h"

/* buffer sizes from detected PSRAM, set in app_main before any init */
extern omi_sizing_t g_sizing;

/* ---- config (Kconfig defaults, NVS namespace "omi" overrides) ---- */
typedef struct {
    char wifi_ssid[33];
    char wifi_pass[65];
    char ingest_url[160];
    char token[128];
    char device_id[48];
    char pendant_mac[18];
    char ntp[64];
    bool storage_sync;
} bridge_config_t;

extern bridge_config_t g_cfg;
void config_load(void);

/* ---- LED / health ---- */
typedef enum {
    LED_NO_WIFI = 0,   /* error. RGB: red. plain: blip every 2 s */
    LED_IDLE,          /* scanning/connecting to pendant. RGB: blue. plain: slow blink */
    LED_STREAMING,     /* streaming, uploads OK. RGB: green. plain: solid */
    LED_BACKLOG,       /* buffering in PSRAM (ingest unreachable). RGB: amber. plain: fast blink */
    LED_SPOOLING,      /* data in the flash spool. RGB: red. plain: fast blink */
} led_state_t;
void led_init(void);
void led_set(led_state_t s);

/* ---- network ---- */
void net_start(void);
bool net_wifi_up(void);
bool net_time_valid(void);
uint32_t net_epoch(void);

/* ---- flash spool (LittleFS on partition "spool") ---- */
#define SPOOL_FMT_LP16   0
#define SPOOL_FMT_OMI_SD 1

typedef struct {
    uint8_t fmt;
    uint32_t start;   /* epoch; 0 = unknown */
    uint32_t len;
} spool_hdr_t;

bool spool_init(void);
bool spool_ready(void);
size_t spool_free_bytes(void);
int spool_count(void);
/* atomic write: .tmp -> fsync -> rename. returns true on success */
bool spool_write(uint8_t fmt, uint32_t start, const uint8_t *data, size_t len);
/* oldest finished record path; false if none */
bool spool_oldest(char *path, size_t cap);
/* read record into a malloc'd (PSRAM-preferred) buffer; caller frees */
bool spool_read(const char *path, spool_hdr_t *hdr, uint8_t **data);
void spool_remove(const char *path);
void spool_quarantine(const char *path);

/* streaming writer for ring sync (.prt, recovered on boot) */
typedef struct spool_stream spool_stream_t;
spool_stream_t *spool_stream_open(uint8_t fmt, uint32_t start);
bool spool_stream_append(spool_stream_t *s, const uint8_t *d, size_t n);
bool spool_stream_sync(spool_stream_t *s);     /* fflush + fsync */
size_t spool_stream_len(spool_stream_t *s);
bool spool_stream_close(spool_stream_t *s);    /* sync, finalize header, rename */

/* ---- uplink (live batching + RAM queue + upload task) ---- */
void uplink_init(void);
void uplink_start(void);
/* called from the NimBLE host task for every reassembled Opus frame */
void uplink_live_frame(const uint8_t *frame, size_t n);
/* finalize the current live batch now (e.g. on disconnect) */
void uplink_live_flush(void);
/* "a spool file was produced" hint */
void uplink_kick(void);
bool uplink_backlogged(void);

/* ---- BLE central ---- */
void ble_central_start(void);
bool ble_connected(void);
/* write to storage characteristic through the safety guard. false if rejected/failed */
bool ble_storage_write(const uint8_t *buf, size_t len);

/* ---- ring storage sync (fw 3.0.21; only if g_cfg.storage_sync) ---- */
void ring_sync_init(void);
/* host task: raw storage notification (incl. type byte) */
void ring_sync_on_notify(const uint8_t *d, size_t len);
/* host task: storage notifications subscribed and protocol known */
void ring_sync_on_ready(omi_storage_proto_t proto);
void ring_sync_on_disconnect(void);
