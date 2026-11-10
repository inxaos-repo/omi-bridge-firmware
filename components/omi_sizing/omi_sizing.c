#include "omi_sizing.h"

omi_sizing_t omi_sizing_compute(size_t psram_bytes, unsigned cfg_queue_kb)
{
    omi_sizing_t s;
    size_t cfg = (size_t)cfg_queue_kb * 1024u;
    if (psram_bytes == 0) {
        s.ram_queue_soft = cfg < OMI_SIZING_NO_PSRAM_QUEUE ? cfg : OMI_SIZING_NO_PSRAM_QUEUE;
        s.ram_queue_hard = s.ram_queue_soft * 2u;
        s.ring_buf = OMI_SIZING_NO_PSRAM_RING;
        return s;
    }
    size_t cap = psram_bytes / 100u * 35u;
    s.ram_queue_soft = cfg < cap ? cfg : cap;
    s.ram_queue_hard = s.ram_queue_soft * 2u;
    size_t rb = psram_bytes / 8u;
    s.ring_buf = rb < OMI_SIZING_RING_MAX ? rb : OMI_SIZING_RING_MAX;
    return s;
}
