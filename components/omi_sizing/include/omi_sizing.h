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
} omi_sizing_t;

#define OMI_SIZING_NO_PSRAM_QUEUE (64u * 1024u)
#define OMI_SIZING_NO_PSRAM_RING  (32u * 1024u)
#define OMI_SIZING_RING_MAX       (256u * 1024u)

/*
 * psram_bytes: detected PSRAM size (0 = none). cfg_queue_kb: CONFIG_OMI_RAM_QUEUE_KB.
 * With PSRAM, soft queue = min(cfg, 35% PSRAM), hard = 2*soft (<= 70% PSRAM),
 * ring = min(256 KB, PSRAM/8). Leaves >= ~17% of PSRAM for live batches and spool reads.
 */
omi_sizing_t omi_sizing_compute(size_t psram_bytes, unsigned cfg_queue_kb);

#ifdef __cplusplus
}
#endif
