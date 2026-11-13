/*
 * omi_proto: pure-C Omi CV1 BLE protocol helpers (no ESP-IDF dependencies).
 *
 * Mirrors omi-monologue-pipeline src/omi_pipeline/omi_protocol.py and
 * docs/omi-ble-protocol.md. Host-tested in test/host (same vectors as
 * tests/test_omi_protocol.py).
 */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OMI_OPUS_MAX_FRAME   1275  /* RFC 6716 max; CV1 actually sends <=160 */
#define OMI_STORAGE_BLOCK    440   /* MAX_WRITE_SIZE record block */
#define OMI_RING_TS_BYTES    4
#define OMI_RING_PACKET      (OMI_RING_TS_BYTES + OMI_STORAGE_BLOCK) /* 444 */
#define OMI_AUDIO_HDR        3     /* [packet_id u16 LE][fragment_index u8] */

/* codec ids (19B10002) */
#define OMI_CODEC_OPUS       20
#define OMI_CODEC_OPUS_FS320 21

/* ---- live audio ------------------------------------------------------- */
typedef struct {
    uint16_t packet_id;
    uint8_t index;
    const uint8_t *payload;
    size_t payload_len;
} omi_audio_packet_t;

/* returns 0 ok, -1 if shorter than the 3-byte header */
int omi_parse_audio_packet(const uint8_t *data, size_t len, omi_audio_packet_t *out);

typedef struct {
    uint8_t buf[OMI_OPUS_MAX_FRAME];
    size_t len;
    bool in_frame;      /* a frame is being assembled (got its index-0 fragment) */
    bool have_last;
    uint16_t last_id;
    int last_idx;
    uint32_t frames_out;
    uint32_t dropped;
    uint32_t bad_packets;
} omi_frame_asm_t;

void omi_frame_asm_init(omi_frame_asm_t *a);
/*
 * Feed one notification. Returns 1 and copies a completed frame to `out`
 * (out_cap >= OMI_OPUS_MAX_FRAME recommended), 0 if nothing completed,
 * -1 on a malformed packet. Semantics = FrameAssembler: a frame completes
 * when the next index-0 fragment arrives; a packet-id gap or index jump
 * drops the partial frame. Stricter than the Python reference in one case:
 * a continuation fragment with no frame in progress is ignored (never
 * emitted as a headless frame).
 */
int omi_frame_asm_feed(omi_frame_asm_t *a, const uint8_t *data, size_t len,
                       uint8_t *out, size_t out_cap, size_t *out_len);
/* emit the in-progress frame (e.g. on disconnect). Returns 1 if emitted. */
int omi_frame_asm_flush(omi_frame_asm_t *a, uint8_t *out, size_t out_cap, size_t *out_len);

/* ---- lp16 batching (ingest format=lp16: repeated [len u16 LE][frame]) -- */
/* Append a frame. Returns new used length, or 0 if it does not fit / is invalid. */
size_t omi_lp16_append(uint8_t *dst, size_t cap, size_t used, const uint8_t *frame, size_t n);
/* Validate like ingest.parse_lp16. Returns frame count (>=0) or -1. */
int omi_lp16_count(const uint8_t *body, size_t len);

/* ---- stored records ([len u8][frame] in 440-byte blocks) -------------- */
typedef void (*omi_frame_cb)(const uint8_t *frame, size_t n, void *ctx);
/* iterate frames exactly like iter_stored_frames(); returns frame count */
int omi_iter_stored_frames(const uint8_t *raw, size_t len, size_t block, omi_frame_cb cb, void *ctx);

/* ---- 3.0.21 ring protocol --------------------------------------------- */
#define OMI_RING_CMD_STOP    0x03
#define OMI_RING_CMD_INFO    0x10
#define OMI_RING_CMD_READ    0x11
#define OMI_RING_CMD_ADVANCE 0x12
#define OMI_RING_CMD_CLEAR   0x13  /* NEVER sent by this firmware (no builder exists) */
/* fw 3.0.24 app custody (see omi-monologue-pipeline docs/omi-ble-protocol.md) */
#define OMI_RING_CMD_CUSTODY    0x14  /* [0x14][version u8][caps u8] */
#define OMI_RING_CMD_ADVANCE_ID 0x15  /* [0x15][ring_id be64][seq be64] */
#define OMI_CAP_APP_ACK_RECLAIM    0x01
#define OMI_CAP_LIVE_PERSIST       0x02
#define OMI_CAP_ADVANCE_IDEMPOTENT 0x04
#define OMI_CAP_RING_ID            0x08
#define OMI_CUSTODY_VERSION        1
#define OMI_RING_ST_OK             0
#define OMI_RING_ST_ID_MISMATCH    11

size_t omi_ring_cmd_info(uint8_t out[1]);
size_t omi_ring_cmd_stop(uint8_t out[1]);
/* count==0 -> 9-byte form (all unread); else 13-byte form */
size_t omi_ring_cmd_read(uint8_t out[13], uint64_t start_seq, uint32_t count);
size_t omi_ring_cmd_advance(uint8_t out[9], uint64_t seq);
size_t omi_ring_cmd_custody(uint8_t out[3], uint8_t caps);
size_t omi_ring_cmd_advance_id(uint8_t out[17], uint64_t ring_id, uint64_t seq);
/* ADVANCE_ID target for a backlog READ of `requested` packets at `start` that
 * delivered `packets`: never past what was actually received. false if none. */
bool omi_custody_target(uint64_t start, uint32_t packets, uint32_t requested, uint64_t *out);
/* extend a 16-bit BLE audio packet id into a monotonically unwrapped counter */
void omi_pid_unwrap(uint32_t *state, bool *valid, uint16_t id);
/* unwrap a 16-bit id to the 32-bit value nearest `ref` */
uint32_t omi_pid_near(uint32_t ref, uint16_t id);

typedef enum {
    OMI_RING_NOTIFY_ACK = 1,
    OMI_RING_NOTIFY_INFO = 2,
    OMI_RING_NOTIFY_DATA = 3,
    OMI_RING_NOTIFY_DONE = 4,
    OMI_RING_NOTIFY_BEGIN = 5,
    OMI_RING_NOTIFY_LIVE_MARK = 6,  /* 3.0.24: [6][ring_id be64][seq be64][live_index be16] */
} omi_ring_notify_type_t;

typedef struct {
    omi_ring_notify_type_t type;
    uint8_t status;                 /* ACK, DONE */
    uint64_t read_seq, write_seq;   /* INFO */
    uint32_t capacity_packets;      /* INFO */
    uint64_t dropped_packets;       /* INFO */
    uint16_t packet_bytes;          /* INFO */
    uint8_t caps;                   /* ACK to CUSTODY (len>=3), INFO (3.0.24) */
    uint8_t custody_version;        /* INFO (3.0.24) */
    bool has_ring_id;               /* INFO carries a ring id (3.0.24) */
    uint64_t ring_id;               /* INFO, LIVE_MARK */
    uint16_t live_index;            /* LIVE_MARK */
    uint64_t seq;                   /* BEGIN start_seq, DONE next_seq */
    uint32_t count;                 /* BEGIN */
    const uint8_t *data;            /* DATA payload (after marker) */
    size_t data_len;
} omi_ring_notify_t;

/* returns 0 ok, -1 malformed/unknown */
int omi_parse_ring_notify(const uint8_t *d, size_t len, omi_ring_notify_t *out);
static inline uint64_t omi_ring_unread(const omi_ring_notify_t *info)
{
    return info->write_seq > info->read_seq ? info->write_seq - info->read_seq : 0;
}

/* reassemble the DATA byte stream into 444-byte packets */
typedef void (*omi_ring_packet_cb)(uint32_t ts, const uint8_t block[OMI_STORAGE_BLOCK], void *ctx);
typedef struct {
    uint8_t buf[OMI_RING_PACKET];
    size_t len;
    uint32_t packets;
} omi_ring_asm_t;
void omi_ring_asm_init(omi_ring_asm_t *a);
void omi_ring_asm_feed(omi_ring_asm_t *a, const uint8_t *chunk, size_t len, omi_ring_packet_cb cb, void *ctx);

/* ---- safety guard ----------------------------------------------------- */
typedef enum { OMI_PROTO_UNKNOWN = 0, OMI_PROTO_FILE, OMI_PROTO_RING } omi_storage_proto_t;
/*
 * Final gate before ANY write to the storage characteristic 30295781.
 * Rejects RING_CLEAR (0x13) and the 3.0.19 DELETE_FILE (0x12 on file proto),
 * and anything on an unknown protocol except STOP.
 */
bool omi_storage_write_allowed(omi_storage_proto_t proto, const uint8_t *buf, size_t len);

/* ---- misc -------------------------------------------------------------- */
/* parse "x.y.z" anywhere in s. Returns 0 ok, -1 not found */
int omi_parse_fw_version(const char *s, size_t len, int v[3]);
omi_storage_proto_t omi_storage_proto_for(const char *s, size_t len);
/* uint32 LE epoch for 19B10031 */
void omi_time_sync_payload(uint8_t out[4], uint32_t epoch);

#ifdef __cplusplus
}
#endif
