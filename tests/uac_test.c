/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 henri (hdavid) */
/* USB audio input and output (chip/usb.c, FELUCCA_UAC): the driver against a model of the SIE and of a host that
 * takes one isochronous packet a millisecond, with the I2S at its own clock (44,117.6 Hz). Built once per
 * configuration (tests/run_usb_host_tests.sh): FELUCCA_UAC_CH (TEST_CH), FELUCCA_UAC_24 (TEST_24), FELUCCA_UAC_OUT
 * (TEST_OUT), FELUCCA_CDC (TEST_CDC).
 *   - the configuration descriptor walks as UAC1 (lengths, interfaces, the AC header's total, the input terminal, each
 *     streaming alt's format and packet size); in the default configuration, byte for byte the descriptors before
 *     FELUCCA_UAC_CH / FELUCCA_UAC_24 (TEST_GOLDEN)
 *   - each alt streams: the frames the render gave, in order and whole once primed (none lost, none repeated), packed
 *     16 or 24-bit; uac_tap(0, n) is silence; uac_capturing follows the host
 *   - uac_play_take hands over exactly what uac_play would have added to silence at unity, and silence when there is
 *     nothing to play
 * It does not test the controller (the SIE's register sequences are the device's own) nor a real host's driver. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wint-to-pointer-cast"
#include "../chip/fm1_usb.h"
#pragma GCC diagnostic pop

#ifndef TEST_CH
#define TEST_CH 2
#endif
#ifndef TEST_24
#define TEST_24 0
#endif
#ifndef TEST_OUT
#define TEST_OUT 1
#endif
#ifndef TEST_CDC
#define TEST_CDC 0
#endif
#ifndef TEST_GOLDEN
#define TEST_GOLDEN (TEST_CH == 2 && !TEST_24 && !TEST_CDC)   /* the configuration felucca-bsp had before them */
#endif

/* ------------------------------------------------------- the SIE model --- */
static uint8_t regs[5][32], common[16], index_reg, last_read;
static uint8_t *ep4rx_buf;
static uint8_t in_pkt[1024];                            /* the packet queued on EP4 IN */
static uint32_t in_len;

static void mock_write(uint32_t r, uint32_t v)
{
    if (r == 14)
        index_reg = (uint8_t)(v & 7u);
    else if (r < 16)
        common[r] = (uint8_t)v;
    else if (index_reg == 0 && r == 17)
        regs[0][17] = 0;                                /* EP0: each request handled at once */
    else if (index_reg == 4 && r == 17)
        regs[4][17] = (uint8_t)(v & 1u);                /* TxPktRdy until the host takes the packet */
    else if (index_reg == 4 && r == 20)
        regs[4][20] = (uint8_t)(v & 0x10u ? 0u : v);    /* the OUT packet taken */
    else
        regs[index_reg][r] = (uint8_t)v;
}
static void mock_rd(uint32_t r) { last_read = (uint8_t)r; }
static uint32_t mock_dt(void) { return last_read < 16 ? common[last_read] : regs[index_reg][last_read]; }
static void mock_buf(uint32_t ep, void *p) { (void)ep, (void)p; }
static void mock_ep4_txbuf(void *p) { (void)p; }
static void mock_ep4_rxbuf(void *p) { ep4rx_buf = p; }
static void mock_send(uint32_t ep, void *p, uint32_t n) { (void)ep, (void)p, (void)n; }
static void mock_ep4_send(void *p, uint32_t n)
{
    assert(n <= sizeof in_pkt);
    memcpy(in_pkt, p, n);
    in_len = n;
}
static void mock_ep0_send(void *p, uint32_t n) { (void)p, (void)n; }
static void mock_ep0_buf(void *p) { (void)p; }
static void mock_attach(void *p) { (void)p; }
static void mock_nop(void) {}
static void mock_enable(uint32_t m) { (void)m; }
static uint32_t mock_one(void) { return 1; }
static uint32_t mock_zero(void) { return 0; }
static uint32_t mock_line(void) { return 0; }
static void fm1_delay_ms(uint32_t ms) { (void)ms; }

#define fm1_usb_ep_enable mock_enable
#define fm1_usb_ep_txbuf mock_buf
#define fm1_usb_ep_rxbuf mock_buf
#define fm1_usb_ep_send mock_send
#define fm1_usb_ep0_send mock_ep0_send
#define fm1_usb_ep0_buf mock_ep0_buf
#define fm1_usb_ep4_txbuf mock_ep4_txbuf
#define fm1_usb_ep4_rxbuf mock_ep4_rxbuf
#define fm1_usb_ep4_send mock_ep4_send
#define fm1_usb_rx_sync mock_nop
#define fm1_usb_sie_on mock_one
#define fm1_usb_sie_done mock_one
#define fm1_usb_sie_wr_start mock_write
#define fm1_usb_sie_rd_start mock_rd
#define fm1_usb_sie_data mock_dt
#define fm1_usb_reset mock_nop
#define fm1_usb_off mock_nop
#define fm1_usb_attach mock_attach
#define fm1_usb_host_attach mock_attach
#define fm1_usb_line mock_line
#define fm1_usb_sof_take mock_zero
#define RING_PUBLISH() ((void)0)
#define FELUCCA_UAC 1
#define FELUCCA_UAC_48K 0
#define FELUCCA_UAC_OUT TEST_OUT
#define FELUCCA_UAC_CH TEST_CH
#define FELUCCA_UAC_24 TEST_24
#define FELUCCA_CDC TEST_CDC
#define FELUCCA_OTA 0
#define FELUCCA_USB_HOST 0
#define HALF_FRAMES 256u
#include "../chip/usb.c"

/* ------------------------------------------------------------ helpers --- */
static void setup(uint8_t rt, uint8_t rq, uint16_t val, uint16_t idx)   /* one EP0 request with no data stage */
{
    uint8_t s[8] = {rt, rq, (uint8_t)val, (uint8_t)(val >> 8), (uint8_t)idx, (uint8_t)(idx >> 8), 0, 0};
    memcpy(ep0buf, s, 8);
    regs[0][17] = 1;                                    /* RxPktRdy */
    common[S_INTRTX1] = 1;
    usb_poll();
    common[S_INTRTX1] = 0;
    assert(!usb.e0_tx);
}

static void render_start_idle(void);
static uint32_t frame_bytes(uint32_t alt) { return UA_CH * (alt == 2u ? 3u : 2u); }

/* --------------------------------------------------- the descriptors --- */
#if TEST_GOLDEN
/* felucca-bsp 2cd696a, FELUCCA_UAC 1, FELUCCA_UAC_OUT TEST_OUT, FELUCCA_UAC_48K 0, FELUCCA_CDC 0 (fm1-base's) */
#if TEST_OUT
static const uint8_t GOLD_CFG[] = {
    9, 2, 249, 0, 4, 1, 0, 128, 50, 9, 4, 0, 0, 0, 1, 1,
    0, 0, 11, 36, 1, 0, 1, 53, 0, 3, 1, 2, 3, 12, 36, 2,
    1, 3, 6, 0, 2, 3, 0, 0, 0, 9, 36, 3, 2, 1, 1, 0,
    1, 0, 12, 36, 2, 3, 1, 1, 0, 2, 3, 0, 0, 0, 9, 36,
    3, 4, 1, 3, 0, 3, 0, 9, 4, 1, 0, 2, 1, 3, 0, 0,
    7, 36, 1, 0, 1, 65, 0, 6, 36, 2, 1, 1, 0, 6, 36, 2,
    2, 2, 0, 9, 36, 3, 1, 3, 1, 2, 1, 0, 9, 36, 3, 2,
    4, 1, 1, 1, 0, 9, 5, 1, 2, 64, 0, 0, 0, 0, 5, 37,
    1, 1, 1, 9, 5, 129, 2, 64, 0, 0, 0, 0, 5, 37, 1, 1,
    3, 9, 4, 2, 0, 0, 1, 2, 0, 0, 9, 4, 2, 1, 1, 1,
    2, 0, 0, 7, 36, 1, 2, 1, 1, 0, 11, 36, 2, 1, 2, 2,
    16, 1, 68, 172, 0, 9, 5, 132, 5, 184, 0, 1, 0, 0, 7, 37,
    1, 1, 0, 0, 0, 9, 4, 3, 0, 0, 1, 2, 0, 0, 9, 4,
    3, 1, 1, 1, 2, 0, 0, 7, 36, 1, 3, 1, 1, 0, 11, 36,
    2, 1, 2, 2, 16, 1, 68, 172, 0, 9, 5, 4, 9, 196, 0, 1,
    0, 0, 7, 37, 1, 0, 0, 0, 0
};
#else
static const uint8_t GOLD_CFG[] = {
    9, 2, 175, 0, 3, 1, 0, 128, 50, 9, 4, 0, 0, 0, 1, 1,
    0, 0, 10, 36, 1, 0, 1, 31, 0, 2, 1, 2, 12, 36, 2, 1,
    3, 6, 0, 2, 3, 0, 0, 0, 9, 36, 3, 2, 1, 1, 0, 1,
    0, 9, 4, 1, 0, 2, 1, 3, 0, 0, 7, 36, 1, 0, 1, 65,
    0, 6, 36, 2, 1, 1, 0, 6, 36, 2, 2, 2, 0, 9, 36, 3,
    1, 3, 1, 2, 1, 0, 9, 36, 3, 2, 4, 1, 1, 1, 0, 9,
    5, 1, 2, 64, 0, 0, 0, 0, 5, 37, 1, 1, 1, 9, 5, 129,
    2, 64, 0, 0, 0, 0, 5, 37, 1, 1, 3, 9, 4, 2, 0, 0,
    1, 2, 0, 0, 9, 4, 2, 1, 1, 1, 2, 0, 0, 7, 36, 1,
    2, 1, 1, 0, 11, 36, 2, 1, 2, 2, 16, 1, 68, 172, 0, 9,
    5, 132, 5, 184, 0, 1, 0, 0, 7, 37, 1, 1, 0, 0, 0
};
#endif
static const uint8_t GOLD_DEV[18] = {18, 1, 0x10, 0x01, 0, 0, 0, 64, 0x09, 0x12, 0x01, 0x00,
                                     0x10 + 0x40 * TEST_OUT, 0x03, 1, 2, 0, 1};
#endif

static void test_descriptors(void)
{
    const uint8_t *d, *dv;
    uint16_t l, dl, i;
    uint32_t nif = 0, ac_total = 0, ac_sum = 0, alts = 0, it_ok = 0, cur_if = 0xFF, cur_alt = 0, sub = 0;
    uint8_t seen_if[16] = {0};
    assert(get_desc(0x0100, &dv, &dl) && dl == 18);
    assert(get_desc(0x0200, &d, &l));
    assert(d[1] == 2 && (d[2] | d[3] << 8) == l);
    assert(dv[12] == (uint8_t)BCD_LO(USB_CDC_ON) && dv[13] == BCD_HI);
    if (TEST_CH == 2 && !TEST_24)
        assert(dv[13] == 0x03);                         /* the version before these options */
    for (i = 0; i < l; i += d[i]) {
        const uint8_t *p = d + i;
        assert(p[0] >= 2 && i + p[0] <= l);
        if (p[1] == 4) {                                /* interface */
            if (cur_if == AUD_IF && ac_total)
                assert(ac_sum == ac_total);             /* the AC header's total: its class-specific descriptors */
            cur_if = p[2];
            cur_alt = p[3];
            if (!seen_if[cur_if & 15u]++)
                nif++;
            if (cur_if == UAC_AS_IF && cur_alt)
                alts++;
        } else if (p[1] == 0x24 && cur_if == AUD_IF) {  /* audio control */
            if (p[2] == 1) {
                ac_total = p[5] | p[6] << 8;
                ac_sum = 0;
            }
            ac_sum += p[0];
            if (p[2] == 2 && p[3] == 1) {               /* input terminal 1, the line: the input's channels */
                uint32_t cfg = p[8] | p[9] << 8, bits = 0, m;
                for (m = cfg; m; m >>= 1)
                    bits += m & 1u;
                assert(p[7] == TEST_CH && (cfg == 0 || bits == TEST_CH));
                it_ok = 1;
            }
        } else if (p[1] == 0x24 && cur_if == UAC_AS_IF && p[2] == 2) {   /* format type I */
            assert(p[3] == 1 && p[4] == TEST_CH);
            sub = p[5];
            assert(cur_alt == 1 ? (p[5] == 2 && p[6] == 16) : (p[5] == 3 && p[6] == 24));
        } else if (p[1] == 5 && p[2] == 0x84) {         /* EP4 IN: a packet of 46 frames at most (44.1 + 1 + 1) */
            uint32_t mp = p[4] | p[5] << 8;
            assert(cur_if == UAC_AS_IF && p[3] == 0x05 && mp == 46u * TEST_CH * sub && mp <= 1023u);
        }
    }
    assert(i == l);
    assert(nif == d[4]);
    assert(it_ok && alts == 1u + TEST_24);
#if TEST_GOLDEN
    assert(l == sizeof GOLD_CFG && !memcmp(d, GOLD_CFG, l));
    assert(!memcmp(dv, GOLD_DEV, 18));
#endif
#ifdef TEST_DUMP
    for (i = 0; i < l; i++)
        printf("%u,%s", d[i], i % 16u == 15u ? "\n" : " ");
    printf("\n");
#endif
    printf("USB audio (%u ch%s%s): descriptors, %u bytes: OK\n", TEST_CH, TEST_24 ? ", 24-bit" : "",
           TEST_OUT ? ", output" : "", l);
}

/* ------------------------------------------------------ the stream --- */
/* the render's sample for frame f, channel c: a count the packet keeps exactly in 16 bits (<< 8) or 24 */
static int32_t smp(uint32_t f, uint32_t c, uint32_t wide)
{
    uint32_t k = f * UA_CH + c;
    return wide ? (int32_t)(k & 0x7FFFFFu) : (int32_t)((k & 0x7FFFu) << 8);
}

static void test_stream(uint32_t alt, int silent)
{
    static int32_t blk[HALF_FRAMES * 8u];
    uint32_t ms, f = 0, acc = 0, got = 0, next = 0, started = 0, pkts = 0, ok = 1, wide = alt == 2u;
    uint32_t under = uac.underruns, over = uac.overruns, up = uac.adj_up, down = uac.adj_down;
    usb_start();
    setup(0x00, 9, 1, 0);                               /* SET_CONFIGURATION 1 */
    setup(0x01, 11, (uint16_t)alt, UAC_AS_IF);          /* SET_INTERFACE: the alt */
    assert(uac.alt == alt);
    for (ms = 0; ms < 3000u; ms++) {
        uint32_t i, c;
        if (regs[4][17] & 1u) {                         /* the host's IN token: takes the queued packet */
            uint32_t fb = frame_bytes(alt), n;
            assert(in_len % fb == 0);
            n = in_len / fb;
            assert(n >= 43u && n <= 46u);
            for (i = 0; i < n; i++)
                for (c = 0; c < UA_CH; c++) {
                    const uint8_t *b = in_pkt + i * fb + c * (wide ? 3u : 2u);
                    int32_t v = wide ? (int32_t)((uint32_t)b[0] | (uint32_t)b[1] << 8 | (uint32_t)b[2] << 16)
                                     : (int32_t)(int16_t)(b[0] | b[1] << 8);
                    if (silent) {
                        ok &= v == 0;
                        continue;
                    }
                    if (!started && v == 0)
                        continue;                       /* the priming's silence (and the count's own 0) */
                    if (!started) {
                        started = 1;                    /* the first rendered sample: frame 0 channel 1 or later */
                        next = (uint32_t)v;
                    }
                    ok &= (uint32_t)v == (wide ? next & 0x7FFFFFu : next & 0x7FFFu);
                    next++;
                    got++;
                }
            regs[4][17] = 0;
            pkts++;
        }
        usb_poll();
        usb_poll();
        for (acc += 441176u; acc >= HALF_FRAMES * 10000u; acc -= HALF_FRAMES * 10000u) {   /* the I2S: 44,117.6 Hz */
            uac_render_start();
            if (ms > 50u)
                assert(uac_capturing());
            if (uac_capturing()) {
                for (i = 0; i < HALF_FRAMES; i++)
                    for (c = 0; c < UA_CH; c++)
                        blk[i * UA_CH + c] = smp(f + i, c, wide);
                uac_tap(silent ? 0 : blk, HALF_FRAMES);
                f += HALF_FRAMES;
            }
        }
    }
    assert(ok);
    assert(pkts > 2900u && uac.underruns == under && uac.overruns == over);
    if (!silent)
        assert(got > 2900u * 44u * UA_CH - 2u * HALF_FRAMES * UA_CH);
    printf("USB audio (%u ch): alt %u%s, %u packets, every frame once: OK (fill %u..%u, +%u -%u)\n", TEST_CH, alt,
           silent ? " silence" : "", pkts, uac.fill_lo, uac.fill_hi, uac.adj_up - up, uac.adj_down - down);
    setup(0x01, 11, 0, UAC_AS_IF);
    render_start_idle();
}

static void render_start_idle(void)
{
    uac_render_start();
    assert(!uac_capturing());                           /* alt 0: nobody records, the stems may rest */
}

#if TEST_OUT
/* ---------------------------------------------------------- playback --- */
static void host_out(uint32_t n, uint32_t *t)           /* an OUT packet of n stereo frames, a ramp */
{
    uint32_t i;
    for (i = 0; i < n; i++, (*t)++) {
        uint16_t l = (uint16_t)(*t * 7u), r = (uint16_t)(0x8000u - *t * 3u);
        memcpy(ep4rx_buf + 4u * i, &l, 2);
        memcpy(ep4rx_buf + 4u * i + 2u, &r, 2);
    }
    regs[4][22] = (uint8_t)(4u * n);
    regs[4][23] = (uint8_t)((4u * n) >> 8);
    regs[4][20] = 1;                                    /* RxPktRdy */
}

static void test_play_take(void)
{
    static int32_t a[2u * HALF_FRAMES], b[2u * HALF_FRAMES];
    uint32_t ms, t = 0, acc = 0, cnt = 0, blocks = 0, with = 0, i, under = uo.underruns;
    usb_start();
    setup(0x00, 9, 1, 0);
    for (i = 0; i < 2u * HALF_FRAMES; i++)
        b[i] = 12345;
    assert(uac_play_take(b, HALF_FRAMES) == 0);         /* nothing streams: silence */
    for (i = 0; i < 2u * HALF_FRAMES; i++)
        assert(b[i] == 0);
    setup(0x01, 11, 1, UAC_ASO_IF);
    for (ms = 0; ms < 2000u; ms++) {
        cnt += 441u;                                    /* the host's 44.1 kHz: 44, 44 ... 45 */
        host_out(cnt / 10u, &t);
        cnt %= 10u;
        usb_poll();
        usb_poll();
        for (acc += 441176u; acc >= HALF_FRAMES * 10000u; acc -= HALF_FRAMES * 10000u) {
            /* the same render twice from the same state: uac_play into silence at 4096, then uac_play_take */
            __typeof__(uo) s = uo;
            uint32_t r = uo_r, w = uo_w;
            int got;
            memset(a, 0, sizeof a);
            uac_play(a, HALF_FRAMES, 4096u);
            uo = s;
            uo_r = r;
            uo_w = w;
            for (i = 0; i < 2u * HALF_FRAMES; i++)
                b[i] = -1;
            got = uac_play_take(b, HALF_FRAMES);
            assert(!memcmp(a, b, sizeof a));
            for (i = 0; i < 2u * HALF_FRAMES && !got; i++)
                assert(b[i] == 0);
            with += (uint32_t)got;
            blocks++;
        }
    }
    assert(with > blocks - 10u && uo.underruns == under);
    printf("USB audio output: uac_play_take = uac_play into silence at unity, %u of %u blocks: OK\n", with, blocks);
    setup(0x01, 11, 0, UAC_ASO_IF);
    assert(uac_play_take(b, HALF_FRAMES) == 0);
}
#endif

int main(void)
{
    test_descriptors();
    test_stream(1, 0);
    test_stream(1, 1);
#if TEST_24
    test_stream(2, 0);
#endif
#if TEST_OUT
    test_play_take();
#endif
    printf("USB audio: all OK\n");
    return 0;
}
