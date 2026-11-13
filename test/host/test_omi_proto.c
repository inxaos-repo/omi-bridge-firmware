/* Host unit tests for components/omi_proto. Vectors mirror
 * omi-monologue-pipeline tests/test_omi_protocol.py. Build: make -C test/host */
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "omi_proto.h"

static int fails;
#define CHECK(c) do { if (!(c)) { fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); fails++; } } while (0)

static size_t pkt(uint8_t *b, uint16_t id, uint8_t idx, const char *payload)
{
    b[0] = id & 0xff; b[1] = id >> 8; b[2] = idx;
    size_t n = strlen(payload);
    memcpy(b + 3, payload, n);
    return 3 + n;
}

static int feed(omi_frame_asm_t *a, uint16_t id, uint8_t idx, const char *p, char *got)
{
    uint8_t b[64], out[OMI_OPUS_MAX_FRAME];
    size_t n = pkt(b, id, idx, p), ol = 0;
    int r = omi_frame_asm_feed(a, b, n, out, sizeof(out), &ol);
    if (r == 1) { memcpy(got, out, ol); got[ol] = 0; } else got[0] = 0;
    return r;
}

static void test_parse_audio_packet(void)
{
    uint8_t b[8];
    omi_audio_packet_t p;
    size_t n = pkt(b, 0x1234, 2, "abc");
    CHECK(omi_parse_audio_packet(b, n, &p) == 0);
    CHECK(p.packet_id == 0x1234 && p.index == 2 && p.payload_len == 3 && !memcmp(p.payload, "abc", 3));
    CHECK(omi_parse_audio_packet((const uint8_t *)"\x00\x01", 2, &p) == -1);
}

static void test_assembler_single_and_fragmented(void)
{
    omi_frame_asm_t a; char g[64]; uint8_t out[OMI_OPUS_MAX_FRAME]; size_t ol;
    omi_frame_asm_init(&a);
    CHECK(feed(&a, 1, 0, "F1", g) == 0);
    CHECK(feed(&a, 2, 0, "F2a", g) == 1 && !strcmp(g, "F1"));
    CHECK(feed(&a, 3, 1, "F2b", g) == 0);
    CHECK(feed(&a, 4, 0, "F3", g) == 1 && !strcmp(g, "F2aF2b"));
    CHECK(omi_frame_asm_flush(&a, out, sizeof(out), &ol) == 1 && ol == 2 && !memcmp(out, "F3", 2));
    CHECK(a.frames_out == 3 && a.dropped == 0);
}

static void test_assembler_drops_on_gap_and_wraps(void)
{
    omi_frame_asm_t a; char g[64]; uint8_t out[OMI_OPUS_MAX_FRAME]; size_t ol;
    omi_frame_asm_init(&a);
    feed(&a, 0xFFFE, 0, "A1", g);
    feed(&a, 0xFFFF, 1, "A2", g);
    CHECK(feed(&a, 0x0000, 0, "B", g) == 1 && !strcmp(g, "A1A2"));
    feed(&a, 0x0001, 0, "C1", g);
    CHECK(feed(&a, 0x0003, 1, "C2", g) == 0);
    CHECK(a.dropped == 1);
    CHECK(omi_frame_asm_flush(&a, out, sizeof(out), &ol) == 0);
}

static void test_assembler_no_headless_frame(void)
{
    /* after a gap, a contiguous continuation must not start a bogus frame */
    omi_frame_asm_t a; char g[64]; uint8_t out[OMI_OPUS_MAX_FRAME]; size_t ol;
    omi_frame_asm_init(&a);
    feed(&a, 1, 0, "X1", g);
    feed(&a, 3, 1, "X2", g);  /* gap -> drop */
    feed(&a, 4, 2, "X3", g);  /* contiguous continuation, no frame in progress */
    CHECK(feed(&a, 5, 0, "Y", g) == 0);
    CHECK(omi_frame_asm_flush(&a, out, sizeof(out), &ol) == 1 && ol == 1 && out[0] == 'Y');
}

static void test_lp16(void)
{
    uint8_t buf[16];
    size_t u = omi_lp16_append(buf, sizeof(buf), 0, (const uint8_t *)"abc", 3);
    CHECK(u == 5 && buf[0] == 3 && buf[1] == 0);
    u = omi_lp16_append(buf, sizeof(buf), u, (const uint8_t *)"defgh", 5);
    CHECK(u == 12);
    CHECK(omi_lp16_append(buf, sizeof(buf), u, (const uint8_t *)"toolong", 7) == 0);
    CHECK(omi_lp16_count(buf, u) == 2);
    CHECK(omi_lp16_count(buf, u - 1) == -1);
    CHECK(omi_lp16_count(buf, 0) == 0);
    uint8_t zero[2] = {0, 0};
    CHECK(omi_lp16_count(zero, 2) == -1);
}

/* mimic firmware write_to_storage() */
static size_t build_blocks(uint8_t *out, uint8_t frames[][200], size_t *lens, int nf)
{
    size_t total = 0, bl = 0;
    uint8_t blk[OMI_STORAGE_BLOCK];
    for (int i = 0; i < nf; i++) {
        size_t n = lens[i] + 1;
        if (bl + n > OMI_STORAGE_BLOCK - 1) {
            blk[bl++] = (uint8_t)lens[i];
            memset(blk + bl, 0xEE, OMI_STORAGE_BLOCK - bl);
            memcpy(out + total, blk, OMI_STORAGE_BLOCK);
            total += OMI_STORAGE_BLOCK;
            bl = 0;
        }
        blk[bl++] = (uint8_t)lens[i];
        memcpy(blk + bl, frames[i], lens[i]);
        bl += lens[i];
    }
    if (bl) {
        memset(blk + bl, 0, OMI_STORAGE_BLOCK - bl);
        memcpy(out + total, blk, OMI_STORAGE_BLOCK);
        total += OMI_STORAGE_BLOCK;
    }
    return total;
}

struct collect { uint8_t (*frames)[200]; size_t *lens; int n; int bad; };
static void collect_cb(const uint8_t *f, size_t n, void *ctx)
{
    struct collect *c = ctx;
    if (n != c->lens[c->n] || memcmp(f, c->frames[c->n], n)) c->bad++;
    c->n++;
}

static uint8_t g_frames[40][200];
static size_t g_lens[40];
static uint8_t g_raw[40 * 2 * OMI_STORAGE_BLOCK];

static void test_iter_stored_frames_roundtrip(void)
{
    for (int i = 0; i < 40; i++) {
        g_lens[i] = 60 + i % 90;
        memset(g_frames[i], i % 251, g_lens[i]);
    }
    size_t n = build_blocks(g_raw, g_frames, g_lens, 40);
    CHECK(n % OMI_STORAGE_BLOCK == 0);
    struct collect c = {g_frames, g_lens, 0, 0};
    CHECK(omi_iter_stored_frames(g_raw, n, OMI_STORAGE_BLOCK, collect_cb, &c) == 40);
    CHECK(c.n == 40 && c.bad == 0);
}

static void test_ring_commands(void)
{
    uint8_t b[13];
    CHECK(omi_ring_cmd_info(b) == 1 && b[0] == 0x10);
    CHECK(omi_ring_cmd_stop(b) == 1 && b[0] == 0x03);
    uint8_t r9[9] = {0x11, 0, 0, 0, 0, 0, 0, 0, 5};
    CHECK(omi_ring_cmd_read(b, 5, 0) == 9 && !memcmp(b, r9, 9));
    uint8_t r13[13] = {0x11, 0, 0, 0, 0, 0, 0, 0, 5, 0, 0, 0, 7};
    CHECK(omi_ring_cmd_read(b, 5, 7) == 13 && !memcmp(b, r13, 13));
    uint8_t a9[9] = {0x12, 0, 0, 0, 0, 0, 0, 0, 9};
    CHECK(omi_ring_cmd_advance(b, 9) == 9 && !memcmp(b, a9, 9));
}

static void put64(uint8_t *p, uint64_t v) { for (int i = 7; i >= 0; i--) { p[i] = v & 0xff; v >>= 8; } }
static void put32(uint8_t *p, uint32_t v) { p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v; }

static void test_parse_ring_notify_all_types(void)
{
    uint8_t info[31] = {2};
    put64(info + 1, 100); put64(info + 9, 160); put32(info + 17, 20000); put64(info + 21, 3);
    info[29] = 444 >> 8; info[30] = 444 & 0xff;
    omi_ring_notify_t n;
    CHECK(omi_parse_ring_notify(info, 31, &n) == 0 && n.type == OMI_RING_NOTIFY_INFO);
    CHECK(n.read_seq == 100 && n.write_seq == 160 && n.capacity_packets == 20000 && n.dropped_packets == 3 && n.packet_bytes == 444);
    CHECK(omi_ring_unread(&n) == 60);
    CHECK(omi_parse_ring_notify((const uint8_t *)"\x01\x0a", 2, &n) == 0 && n.type == OMI_RING_NOTIFY_ACK && n.status == 10);
    CHECK(omi_parse_ring_notify((const uint8_t *)"\x03" "abc", 4, &n) == 0 && n.type == OMI_RING_NOTIFY_DATA && n.data_len == 3 && !memcmp(n.data, "abc", 3));
    uint8_t done[10] = {4, 0}; put64(done + 2, 161);
    CHECK(omi_parse_ring_notify(done, 10, &n) == 0 && n.type == OMI_RING_NOTIFY_DONE && n.status == 0 && n.seq == 161);
    uint8_t begin[13] = {5}; put64(begin + 1, 100); put32(begin + 9, 60);
    CHECK(omi_parse_ring_notify(begin, 13, &n) == 0 && n.type == OMI_RING_NOTIFY_BEGIN && n.seq == 100 && n.count == 60);
    CHECK(omi_parse_ring_notify((const uint8_t *)"\x02short", 6, &n) == -1);
}

struct ringc { struct collect c; uint32_t ts[64]; int nts; };
static void ring_cb(uint32_t ts, const uint8_t *block, void *ctx)
{
    struct ringc *r = ctx;
    r->ts[r->nts++] = ts;
    omi_iter_stored_frames(block, OMI_STORAGE_BLOCK, OMI_STORAGE_BLOCK, collect_cb, &r->c);
}

static void test_ring_assembler_across_notification_boundaries(void)
{
    static uint8_t frames[30][200], blocks[30 * 2 * OMI_STORAGE_BLOCK], stream[30 * 2 * OMI_RING_PACKET];
    static size_t lens[30];
    for (int i = 0; i < 30; i++) { lens[i] = 50 + i; memset(frames[i], i, lens[i]); }
    size_t bn = build_blocks(blocks, frames, lens, 30);
    int n = (int)(bn / OMI_STORAGE_BLOCK);
    size_t sl = 0;
    for (int i = 0; i < n; i++) {
        put32(stream + sl, 1762700000u + i);
        memcpy(stream + sl + 4, blocks + i * OMI_STORAGE_BLOCK, OMI_STORAGE_BLOCK);
        sl += OMI_RING_PACKET;
    }
    omi_ring_asm_t a; omi_ring_asm_init(&a);
    struct ringc r = {{frames, lens, 0, 0}, {0}, 0};
    for (size_t i = 0; i < sl; i += 243) {
        size_t take = sl - i < 243 ? sl - i : 243;
        omi_ring_asm_feed(&a, stream + i, take, ring_cb, &r);
    }
    CHECK(r.c.n == 30 && r.c.bad == 0 && a.len == 0 && r.nts == n);
    for (int i = 0; i < n; i++) CHECK(r.ts[i] == 1762700000u + i);
}

static void test_fw_version_protocol_selection(void)
{
    int v[3];
    CHECK(omi_parse_fw_version("3.0.21", 6, v) == 0 && v[0] == 3 && v[1] == 0 && v[2] == 21);
    CHECK(omi_parse_fw_version("v3.0.19-omi", 11, v) == 0 && v[2] == 19);
    CHECK(omi_storage_proto_for("3.0.19", 6) == OMI_PROTO_FILE);
    CHECK(omi_storage_proto_for("3.0.21", 6) == OMI_PROTO_RING);
    CHECK(omi_storage_proto_for("3.1.0", 5) == OMI_PROTO_RING);
    CHECK(omi_storage_proto_for("", 0) == OMI_PROTO_UNKNOWN);
    CHECK(omi_storage_proto_for("1.2", 3) == OMI_PROTO_UNKNOWN);
}

static void test_guard_never_wipes(void)
{
    uint8_t b[13];
    uint8_t clear = OMI_RING_CMD_CLEAR;
    CHECK(!omi_storage_write_allowed(OMI_PROTO_RING, &clear, 1));
    CHECK(!omi_storage_write_allowed(OMI_PROTO_FILE, &clear, 1));
    uint8_t del[2] = {0x12, 0x01};
    CHECK(!omi_storage_write_allowed(OMI_PROTO_FILE, del, 2));
    CHECK(omi_storage_write_allowed(OMI_PROTO_RING, b, omi_ring_cmd_info(b)));
    CHECK(omi_storage_write_allowed(OMI_PROTO_RING, b, omi_ring_cmd_read(b, 1, 2)));
    CHECK(omi_storage_write_allowed(OMI_PROTO_RING, b, omi_ring_cmd_advance(b, 1)));
    CHECK(omi_storage_write_allowed(OMI_PROTO_UNKNOWN, b, omi_ring_cmd_stop(b)));
    CHECK(!omi_storage_write_allowed(OMI_PROTO_UNKNOWN, b, omi_ring_cmd_info(b)));
}

static void test_time_sync(void)
{
    uint8_t b[4];
    omi_time_sync_payload(b, 1);
    CHECK(b[0] == 1 && b[1] == 0 && b[2] == 0 && b[3] == 0);
}


static void test_custody_3024(void)
{
    uint8_t b[17];
    CHECK(omi_ring_cmd_custody(b, 0x0f) == 3 && b[0] == 0x14 && b[1] == 1 && b[2] == 0x0f);
    CHECK(omi_storage_write_allowed(OMI_PROTO_RING, b, 3));
    CHECK(omi_ring_cmd_advance_id(b, 0x0102030405060708ull, 0x1122ull) == 17);
    CHECK(b[0] == 0x15 && b[1] == 1 && b[8] == 8 && b[15] == 0x11 && b[16] == 0x22);
    CHECK(omi_storage_write_allowed(OMI_PROTO_RING, b, 17));
    CHECK(!omi_storage_write_allowed(OMI_PROTO_RING, b, 16));
    uint8_t clr[1] = {0x13};
    CHECK(!omi_storage_write_allowed(OMI_PROTO_RING, clr, 1));

    omi_ring_notify_t m;
    uint8_t ack[3] = {1, 0, 0x0f};
    CHECK(omi_parse_ring_notify(ack, 3, &m) == 0 && m.type == OMI_RING_NOTIFY_ACK && m.caps == 0x0f);
    uint8_t nak[3] = {1, 11, 0x0f};
    CHECK(omi_parse_ring_notify(nak, 3, &m) == 0 && m.status == 11 && m.caps == 0);

    uint8_t info[41] = {2};
    info[8] = 5;          /* read_seq = 5 */
    info[16] = 9;         /* write_seq = 9 */
    info[29] = 0x01; info[30] = 0xbc; /* 444 */
    info[31] = 0x0f; info[32] = 1;
    info[33] = 0xaa; info[40] = 0x55;
    CHECK(omi_parse_ring_notify(info, 41, &m) == 0);
    CHECK(m.read_seq == 5 && m.write_seq == 9 && m.packet_bytes == 444);
    CHECK(m.has_ring_id && m.custody_version == 1 && m.ring_id == 0xaa00000000000055ull);
    CHECK(omi_parse_ring_notify(info, 31, &m) == 0 && !m.has_ring_id); /* 3.0.21 INFO */

    uint8_t mk[19] = {6};
    mk[8] = 7; mk[16] = 0x30; mk[17] = 0x12; mk[18] = 0x34;
    CHECK(omi_parse_ring_notify(mk, 19, &m) == 0 && m.type == OMI_RING_NOTIFY_LIVE_MARK);
    CHECK(m.ring_id == 7 && m.seq == 0x30 && m.live_index == 0x1234);
    CHECK(omi_parse_ring_notify(mk, 18, &m) != 0);

    uint64_t t = 0;
    CHECK(omi_custody_target(100, 10, 10, &t) && t == 110);
    CHECK(omi_custody_target(100, 4, 10, &t) && t == 104);
    CHECK(omi_custody_target(100, 12, 10, &t) && t == 110); /* never past request */
    CHECK(!omi_custody_target(100, 0, 10, &t));

    uint32_t u = 0; bool v = false;
    omi_pid_unwrap(&u, &v, 65530);
    CHECK(v && u == 65530);
    omi_pid_unwrap(&u, &v, 3);
    CHECK(u == 65539);
    CHECK(omi_pid_near(65539, 65535) == 65535);
    CHECK(omi_pid_near(65530, 2) == 65538);
    CHECK(omi_pid_near(100, 90) == 90);
}

int main(void)
{
    test_custody_3024();
    test_parse_audio_packet();
    test_assembler_single_and_fragmented();
    test_assembler_drops_on_gap_and_wraps();
    test_assembler_no_headless_frame();
    test_lp16();
    test_iter_stored_frames_roundtrip();
    test_ring_commands();
    test_parse_ring_notify_all_types();
    test_ring_assembler_across_notification_boundaries();
    test_fw_version_protocol_selection();
    test_guard_never_wipes();
    test_time_sync();
    if (fails) { fprintf(stderr, "%d check(s) failed\n", fails); return 1; }
    printf("all omi_proto host tests passed\n");
    return 0;
}
