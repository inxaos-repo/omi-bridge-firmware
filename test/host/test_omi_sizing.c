#include <stdio.h>

#include "omi_sizing.h"

static int fails;
#define CHECK(c) do { if (!(c)) { fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); fails++; } } while (0)
#define MB (1024u * 1024u)

int main(void)
{
    /* XIAO: 8 MB PSRAM, default 2048 KB -> config wins */
    omi_sizing_t x = omi_sizing_compute(8 * MB, 2048);
    CHECK(x.ram_queue_soft == 2048u * 1024u);
    CHECK(x.ram_queue_hard == 4096u * 1024u);
    CHECK(x.ring_buf == 256u * 1024u);
    CHECK(x.ram_queue_hard + x.ring_buf < 8 * MB);

    /* SuperMini: 2 MB quad PSRAM -> capped well below PSRAM */
    omi_sizing_t m = omi_sizing_compute(2 * MB, 2048);
    CHECK(m.ram_queue_soft < 2048u * 1024u);
    CHECK(m.ram_queue_soft <= 2 * MB * 35 / 100);
    CHECK(m.ram_queue_hard + m.ring_buf <= 2 * MB * 83 / 100);
    CHECK(m.ring_buf == 256u * 1024u);
    CHECK(m.ram_queue_soft >= 512u * 1024u); /* still > 2 min of audio */

    /* small config is respected */
    omi_sizing_t s = omi_sizing_compute(2 * MB, 128);
    CHECK(s.ram_queue_soft == 128u * 1024u);

    /* no PSRAM */
    omi_sizing_t n = omi_sizing_compute(0, 2048);
    CHECK(n.ram_queue_soft == 64u * 1024u);
    CHECK(n.ring_buf == 32u * 1024u);

    if (fails) { fprintf(stderr, "%d sizing check(s) failed\n", fails); return 1; }
    printf("all omi_sizing host tests passed\n");
    return 0;
}
