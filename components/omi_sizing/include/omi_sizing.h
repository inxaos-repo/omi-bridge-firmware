/* Pure-C buffer sizing from detected PSRAM (no ESP-IDF includes; host-testable). */
#pragma once
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    size_t ram_queue_soft; /* spill RAM queue to flash above this */
    size_t ram_queue_hard; /* drop oldest above this when flash is also full */
    size_t ring_buf;       /* BLE -> ring_sync buffer */
    size_t custody_batch;  /* 3.0.24 backlog READ batch held until ingest 2xx */
} omi_sizing_t;

#define OMI_SIZING_NO_PSRAM_QUEUE (64u * 1024u)
#define OMI_SIZING_NO_PSRAM_RING  (32u * 1024u)
#define OMI_SIZING_RING_MAX       (256u * 1024u)
#define OMI_SIZING_NO_PSRAM_BATCH (16u * 1024u)
#define OMI_SIZING_BATCH_MAX      (128u * 1024u)

/*
 * psram_bytes: detected PSRAM size (0 = none). cfg_queue_kb: CONFIG_OMI_RAM_QUEUE_KB.
 * With PSRAM, soft queue = min(cfg, 35% PSRAM). hard = 2*soft (<= 70%) when
 * PSRAM >= 4 MB, else 1.5*soft (<= 53%, SuperMini 2 MB). ring = min(256 KB, PSRAM/8),
 * custody_batch = min(128 KB, PSRAM/16). 2 MB: ~1.07 MB+256K+128K, leaving ~0.5 MB
 * for live batches, spool reads and NimBLE/WiFi PSRAM use.
 */
omi_sizing_t omi_sizing_compute(size_t psram_bytes, unsigned cfg_queue_kb);

#ifdef __cplusplus
}
#endif
