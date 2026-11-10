#include "omi_proto.h"

#include <string.h>

static uint64_t rd_be64(const uint8_t *p)
{
    uint64_t v = 0;
    for (int i = 0; i < 8; i++) v = (v << 8) | p[i];
    return v;
}
static uint32_t rd_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}
static void wr_be64(uint8_t *p, uint64_t v)
{
    for (int i = 7; i >= 0; i--) { p[i] = (uint8_t)v; v >>= 8; }
}
static void wr_be32(uint8_t *p, uint32_t v)
{
    p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = (uint8_t)v;
}

/* ---- live audio ---- */
int omi_parse_audio_packet(const uint8_t *d, size_t len, omi_audio_packet_t *o)
{
    if (!d || len < OMI_AUDIO_HDR) return -1;
    o->packet_id = (uint16_t)(d[0] | (d[1] << 8));
    o->index = d[2];
    o->payload = d + OMI_AUDIO_HDR;
    o->payload_len = len - OMI_AUDIO_HDR;
    return 0;
}

void omi_frame_asm_init(omi_frame_asm_t *a)
{
    memset(a, 0, sizeof(*a));
    a->last_idx = -1;
}

static int emit(omi_frame_asm_t *a, uint8_t *out, size_t cap, size_t *out_len)
{
    if (!a->in_frame || a->len == 0 || a->len > cap) {
        if (a->in_frame && a->len > cap) a->dropped++;
        return 0;
    }
    memcpy(out, a->buf, a->len);
    *out_len = a->len;
    a->frames_out++;
    return 1;
}

int omi_frame_asm_feed(omi_frame_asm_t *a, const uint8_t *data, size_t len,
                       uint8_t *out, size_t out_cap, size_t *out_len)
{
    omi_audio_packet_t p;
    if (omi_parse_audio_packet(data, len, &p) != 0) {
        a->bad_packets++;
        return -1;
    }
    int r = 0;
    bool contiguous = a->have_last && p.packet_id == (uint16_t)(a->last_id + 1);
    if (p.index == 0) {
        r = emit(a, out, out_cap, out_len);
        a->len = 0;
        a->in_frame = true;
        if (p.payload_len > sizeof(a->buf)) {  /* oversize: drop */
            a->in_frame = false;
            a->dropped++;
        } else {
            memcpy(a->buf, p.payload, p.payload_len);
            a->len = p.payload_len;
        }
    } else if (contiguous && (int)p.index == a->last_idx + 1 && a->in_frame) {
        if (a->len + p.payload_len > sizeof(a->buf)) {
            a->in_frame = false;
            a->len = 0;
            a->dropped++;
        } else {
            memcpy(a->buf + a->len, p.payload, p.payload_len);
            a->len += p.payload_len;
        }
    } else {
        if (a->in_frame && a->len) a->dropped++;
        a->in_frame = false;
        a->len = 0;
    }
    a->have_last = true;
    a->last_id = p.packet_id;
    a->last_idx = p.index;
    return r;
}

int omi_frame_asm_flush(omi_frame_asm_t *a, uint8_t *out, size_t out_cap, size_t *out_len)
{
    int r = emit(a, out, out_cap, out_len);
    a->in_frame = false;
    a->len = 0;
    a->have_last = false;
    a->last_idx = -1;
    return r;
}

/* ---- lp16 ---- */
size_t omi_lp16_append(uint8_t *dst, size_t cap, size_t used, const uint8_t *f, size_t n)
{
    if (n == 0 || n > OMI_OPUS_MAX_FRAME || used + 2 + n > cap) return 0;
    dst[used] = (uint8_t)(n & 0xff);
    dst[used + 1] = (uint8_t)(n >> 8);
    memcpy(dst + used + 2, f, n);
    return used + 2 + n;
}

int omi_lp16_count(const uint8_t *b, size_t len)
{
    size_t o = 0;
    int c = 0;
    while (o < len) {
        if (o + 2 > len) return -1;
        size_t n = b[o] | (b[o + 1] << 8);
        if (n == 0 || n > OMI_OPUS_MAX_FRAME || o + 2 + n > len) return -1;
        o += 2 + n;
        c++;
    }
    return c;
}

/* ---- stored records ---- */
int omi_iter_stored_frames(const uint8_t *raw, size_t len, size_t block, omi_frame_cb cb, void *ctx)
{
    int c = 0;
    for (size_t base = 0; base < len; base += block) {
        size_t blen = len - base < block ? len - base : block;
        const uint8_t *blk = raw + base;
        size_t o = 0;
        while (o < blen) {
            size_t n = blk[o];
            if (n == 0 || n > OMI_OPUS_MAX_FRAME || o + 1 + n > blen || o + 1 + n > block - 1) break;
            if (cb) cb(blk + o + 1, n, ctx);
            c++;
            o += 1 + n;
        }
    }
    return c;
}

/* ---- ring ---- */
size_t omi_ring_cmd_info(uint8_t out[1]) { out[0] = OMI_RING_CMD_INFO; return 1; }
size_t omi_ring_cmd_stop(uint8_t out[1]) { out[0] = OMI_RING_CMD_STOP; return 1; }

size_t omi_ring_cmd_read(uint8_t out[13], uint64_t start_seq, uint32_t count)
{
    out[0] = OMI_RING_CMD_READ;
    wr_be64(out + 1, start_seq);
    if (!count) return 9;
    wr_be32(out + 9, count);
    return 13;
}

size_t omi_ring_cmd_advance(uint8_t out[9], uint64_t seq)
{
    out[0] = OMI_RING_CMD_ADVANCE;
    wr_be64(out + 1, seq);
    return 9;
}

int omi_parse_ring_notify(const uint8_t *d, size_t len, omi_ring_notify_t *o)
{
    if (!d || len < 1) return -1;
    memset(o, 0, sizeof(*o));
    o->type = (omi_ring_notify_type_t)d[0];
    switch (d[0]) {
    case OMI_RING_NOTIFY_ACK:
        if (len < 2) return -1;
        o->status = d[1];
        return 0;
    case OMI_RING_NOTIFY_INFO:
        if (len < 31) return -1;
        o->read_seq = rd_be64(d + 1);
        o->write_seq = rd_be64(d + 9);
        o->capacity_packets = rd_be32(d + 17);
        o->dropped_packets = rd_be64(d + 21);
        o->packet_bytes = (uint16_t)((d[29] << 8) | d[30]);
        return 0;
    case OMI_RING_NOTIFY_DATA:
        o->data = d + 1;
        o->data_len = len - 1;
        return 0;
    case OMI_RING_NOTIFY_DONE:
        if (len < 10) return -1;
        o->status = d[1];
        o->seq = rd_be64(d + 2);
        return 0;
    case OMI_RING_NOTIFY_BEGIN:
        if (len < 13) return -1;
        o->seq = rd_be64(d + 1);
        o->count = rd_be32(d + 9);
        return 0;
    default:
        return -1;
    }
}

void omi_ring_asm_init(omi_ring_asm_t *a) { memset(a, 0, sizeof(*a)); }

void omi_ring_asm_feed(omi_ring_asm_t *a, const uint8_t *chunk, size_t len, omi_ring_packet_cb cb, void *ctx)
{
    while (len) {
        size_t take = OMI_RING_PACKET - a->len;
        if (take > len) take = len;
        memcpy(a->buf + a->len, chunk, take);
        a->len += take;
        chunk += take;
        len -= take;
        if (a->len == OMI_RING_PACKET) {
            a->packets++;
            if (cb) cb(rd_be32(a->buf), a->buf + OMI_RING_TS_BYTES, ctx);
            a->len = 0;
        }
    }
}

/* ---- guard ---- */
bool omi_storage_write_allowed(omi_storage_proto_t proto, const uint8_t *b, size_t len)
{
    if (!b || len == 0) return false;
    if (b[0] == OMI_RING_CMD_STOP && len == 1) return true;
    if (proto == OMI_PROTO_RING) {
        switch (b[0]) {
        case OMI_RING_CMD_INFO: return len == 1;
        case OMI_RING_CMD_READ: return len == 9 || len == 13;
        case OMI_RING_CMD_ADVANCE: return len == 9;
        default: return false; /* incl. RING_CLEAR 0x13 */
        }
    }
    if (proto == OMI_PROTO_FILE) {
        /* LIST and READ only; DELETE_FILE (0x12) is never allowed */
        if (b[0] == 0x10) return len == 1;
        if (b[0] == 0x11) return len == 6;
        return false;
    }
    return false;
}

/* ---- misc ---- */
int omi_parse_fw_version(const char *s, size_t len, int v[3])
{
    if (!s) return -1;
    for (size_t i = 0; i < len; i++) {
        size_t j = i;
        int got = 0;
        int tmp[3] = {0, 0, 0};
        while (got < 3) {
            if (j >= len || s[j] < '0' || s[j] > '9') break;
            int x = 0;
            while (j < len && s[j] >= '0' && s[j] <= '9') x = x * 10 + (s[j++] - '0');
            tmp[got++] = x;
            if (got < 3) {
                if (j < len && s[j] == '.') j++;
                else break;
            }
        }
        if (got == 3) {
            memcpy(v, tmp, sizeof(tmp));
            return 0;
        }
    }
    return -1;
}

omi_storage_proto_t omi_storage_proto_for(const char *s, size_t len)
{
    int v[3];
    if (omi_parse_fw_version(s, len, v) != 0) return OMI_PROTO_UNKNOWN;
    long n = (long)v[0] * 1000000L + (long)v[1] * 1000L + v[2];
    return n >= 3000021L ? OMI_PROTO_RING : OMI_PROTO_FILE;
}

void omi_time_sync_payload(uint8_t out[4], uint32_t e)
{
    out[0] = (uint8_t)e; out[1] = e >> 8; out[2] = e >> 16; out[3] = e >> 24;
}
