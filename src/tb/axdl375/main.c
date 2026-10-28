// ============================================================================
// ADXL375 test rig - Raspberry Pi Pico 2 W
//
// The FPGA is the SPI master and owns the ADXL375. This Pico is an SPI SLAVE
// of the FPGA: it receives a 20-byte packet for every sample and for every
// status change (plus a heartbeat at least every 100 ms), checks it, and
// reports the sensor's state and data to the PC over USB serial, in the same
// format as the old Pico-as-master bring-up code.
//
// Keys in the terminal:  p = pause/resume sample lines   s = link statistics
// ============================================================================

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/spi.h"
#include "hardware/dma.h"

// ---------------------------------------------------------------------------
// Wiring (FPGA is the SPI MASTER, this Pico is the slave, one-way link)
// Same SPI0 pins as the old code, but now they face the FPGA.
// ---------------------------------------------------------------------------
#define SPI_PORT  spi0
#define PIN_RX    16   // SPI0 RX  <- FPGA pico_mosi
#define PIN_CS    17   // SPI0 CSn <- FPGA pico_cs_n
#define PIN_SCK   18   // SPI0 SCK <- FPGA pico_sclk
                       // GP19 (SPI0 TX) is not used. Connect GND to the FPGA board's GND.

// Baud is ignored in slave mode; the FPGA sets SCLK (~1 MHz). The PL022
// needs clk_peri >= 12 x SCLK as a slave, so the hard limit is 12.5 MHz.
#define LINK_SPI_HZ  (1000 * 1000)

// ---------------------------------------------------------------------------
// Packet format (must match adxl375_rig_top.sv)
// ---------------------------------------------------------------------------
#define PKT_LEN        20
#define PKT_SYNC0      0xA5
#define PKT_SYNC1      0x5A
#define PKT_VERSION    0x1
#define PKT_T_SAMPLE   0x1
#define PKT_T_STATUS   0x2

#define FLAG_INIT_DONE    0x01
#define FLAG_FAULT        0x02
#define FLAG_CAUSE_WDOG   0x04   // fault_cause: 1 = INT1 watchdog, 0 = init step failed
#define FLAG_INT1         0x08

#define MG_PER_LSB       49.0f   // typical scale factor
#define LINK_TIMEOUT_US  1000000 // FPGA sends at least every 100 ms

typedef struct {
    uint8_t  type, seq, flags;
    uint8_t  fault_pc, fault_rdata, reinit_count, link_drops;
    uint32_t sample_count;
    int16_t  x, y, z;
} rig_packet_t;

// ---------------------------------------------------------------------------
// The FPGA's init sequence (must match the ROM in adxl375_ctrl.sv), so a
// fault can be reported as a register name instead of a step number.
// ---------------------------------------------------------------------------
typedef struct {
    const char *name;
    uint8_t     reg;
    uint8_t     val;
    bool        is_read;   // true: read until equal; false: write + read back
} init_step_t;

static const init_step_t init_steps[] = {
    { "DEVID",               0x00, 0xE5, true  },
    { "POWER_CTL (standby)", 0x2D, 0x00, false },
    { "BW_RATE",             0x2C, 0x0A, false },
    { "DATA_FORMAT",         0x31, 0x0B, false },
    { "FIFO_CTL",            0x38, 0x00, false },
    { "INT_MAP",             0x2F, 0x00, false },
    { "INT_ENABLE",          0x2E, 0x80, false },
    { "POWER_CTL (measure)", 0x2D, 0x08, false },
};
#define N_INIT_STEPS (sizeof init_steps / sizeof init_steps[0])

// Explain a bad DEVID so the wiring problem is obvious from the console.
// (Same table as the old code. Now it describes the FPGA <-> sensor wiring.)
static const char *devid_hint(uint8_t devid)
{
    switch (devid) {
        case 0x00: return "MISO stuck low: SDO not wired, or sensor unpowered (needs VS AND VDD I/O)";
        case 0xFF: return "MISO stuck high/floating, or CS never goes low";
        case 0x72: case 0xCA: return "bit-shifted: SPI mode is probably not mode 3";
        default:   return "unexpected value: check SDI/SDO not swapped, SCK, and mode 3";
    }
}

static const char *readback_hint(uint8_t rb)
{
    if (rb == 0x00 || rb == 0xFF) return devid_hint(rb);
    return "write did not stick: check SDI (FPGA acc_mosi) wiring and the sensor supply";
}

// ---------------------------------------------------------------------------
// CRC-8, poly 0x07, init 0x00 (same as the FPGA)
// ---------------------------------------------------------------------------
static uint8_t crc8(const uint8_t *p, size_t n)
{
    uint8_t c = 0;
    while (n--) {
        c ^= *p++;
        for (int i = 0; i < 8; i++)
            c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
    }
    return c;
}

// ---------------------------------------------------------------------------
// Reporting
// ---------------------------------------------------------------------------
static struct {
    uint32_t bytes;          // raw bytes received
    uint32_t packets;        // good packets
    uint32_t crc_errors;     // sync found but CRC/version wrong
    uint32_t lost;           // gaps in the packet sequence number
    uint32_t missed_samples; // gaps in sample_count
} st;

static bool         link_up      = false;
static bool         have_prev    = false;
static bool         print_samples = true;
static rig_packet_t prev;               // last good packet
static uint32_t     last_pkt_us;
static uint32_t     last_check_us;      // link_health() period

static void print_status(const rig_packet_t *p)
{
    bool init_done = p->flags & FLAG_INIT_DONE;
    bool fault     = p->flags & FLAG_FAULT;
    bool wdog      = p->flags & FLAG_CAUSE_WDOG;
    bool int1      = p->flags & FLAG_INT1;

    printf("[FPGA] ");
    if (init_done && !fault)
        printf("STREAMING");
    else if (init_done && fault)
        printf("RE-INITIALIZED, waiting for the first sample");
    else if (!fault)
        printf("INITIALIZING (power-up wait / configuring)");
    else if (wdog)
        printf("FAULT: no INT1 for 30 ms while streaming -> re-initializing sensor");
    else
        printf("FAULT: init gave up, retrying every 500 ms");
    printf("  INT1=%d  re-inits=%u\n", int1, p->reinit_count);

    // Detail of the last fault, while it is active or after any recovery.
    if (fault || p->reinit_count) {
        if (p->flags & FLAG_CAUSE_WDOG) {
            printf("       last fault: INT1 watchdog (sensor browned out / reset, or INT1 wiring)\n");
        } else if (p->fault_pc < N_INIT_STEPS) {
            const init_step_t *s = &init_steps[p->fault_pc];
            if (s->is_read)
                printf("       last fault: step %u %s (0x%02X) read 0x%02X, want 0x%02X -> %s\n",
                       p->fault_pc, s->name, s->reg, p->fault_rdata, s->val, devid_hint(p->fault_rdata));
            else
                printf("       last fault: step %u %s (0x%02X) <- 0x%02X, readback 0x%02X -> %s\n",
                       p->fault_pc, s->name, s->reg, s->val, p->fault_rdata,
                       readback_hint(p->fault_rdata));
        } else {
            printf("       last fault: step %u, readback 0x%02X\n", p->fault_pc, p->fault_rdata);
        }
    }
}

static void print_link_stats(void)
{
    printf("[link] %lu bytes, %lu good packets, %lu corrupt, %lu lost, "
           "%lu samples missed (FPGA-side drops %u)\n",
           (unsigned long)st.bytes, (unsigned long)st.packets, (unsigned long)st.crc_errors,
           (unsigned long)st.lost, (unsigned long)st.missed_samples,
           have_prev ? prev.link_drops : 0);
}

static void handle_packet(const rig_packet_t *p)
{
    st.packets++;
    last_pkt_us = time_us_32();

    if (!link_up) {
        printf("[link] receiving packets from the FPGA\n");
        link_up = true;
    }

    // FPGA reset: its counters went backwards.
    bool fpga_reset = have_prev && (p->sample_count < prev.sample_count);
    if (fpga_reset) {
        printf("[link] FPGA was reset (sample count went back to %lu)\n",
               (unsigned long)p->sample_count);
        have_prev = false;
    }

    if (have_prev && p->seq != (uint8_t)(prev.seq + 1))
        st.lost += (uint8_t)(p->seq - prev.seq - 1);

    // Status line whenever anything except INT1 / the data changes.
    uint8_t mask = FLAG_INIT_DONE | FLAG_FAULT | FLAG_CAUSE_WDOG;
    if (!have_prev ||
        (p->flags & mask) != (prev.flags & mask) ||
        p->fault_pc     != prev.fault_pc   ||
        p->fault_rdata  != prev.fault_rdata ||
        p->reinit_count != prev.reinit_count)
        print_status(p);

    if (p->type == PKT_T_SAMPLE) {
        if (have_prev && p->sample_count != prev.sample_count + 1) {
            uint32_t gap = p->sample_count - prev.sample_count - 1;
            st.missed_samples += gap;
            if (print_samples) printf("       (%lu samples missed)\n", (unsigned long)gap);
        }
        if (print_samples)
            printf("[%lu] X=%6d Y=%6d Z=%6d  |  %7.2f %7.2f %7.2f g\n",
                   (unsigned long)p->sample_count,
                   p->x, p->y, p->z,
                   p->x * MG_PER_LSB / 1000.0f,
                   p->y * MG_PER_LSB / 1000.0f,
                   p->z * MG_PER_LSB / 1000.0f);
    }

    prev      = *p;
    have_prev = true;
}

// ---------------------------------------------------------------------------
// Packet parser: find A5 5A, collect 20 bytes, check version + CRC.
// On a bad packet, drop one byte and search again, so it resynchronizes after
// noise or when the Pico starts listening in the middle of a packet.
// ---------------------------------------------------------------------------
static uint8_t pkt[PKT_LEN];
static int     pkt_n = 0;

static void parse_byte(uint8_t b)
{
    st.bytes++;
    pkt[pkt_n++] = b;

    for (;;) {
        if ((pkt_n >= 1 && pkt[0] != PKT_SYNC0) || (pkt_n >= 2 && pkt[1] != PKT_SYNC1)) {
            memmove(pkt, pkt + 1, --pkt_n);
            continue;
        }
        if (pkt_n < PKT_LEN)
            return;

        if ((pkt[2] >> 4) == PKT_VERSION && crc8(pkt, PKT_LEN - 1) == pkt[PKT_LEN - 1]) {
            rig_packet_t p = {
                .type         = pkt[2] & 0x0F,
                .seq          = pkt[3],
                .flags        = pkt[4],
                .fault_pc     = pkt[5],
                .fault_rdata  = pkt[6],
                .reinit_count = pkt[7],
                .sample_count = (uint32_t)pkt[8] | (uint32_t)pkt[9] << 8 |
                                (uint32_t)pkt[10] << 16 | (uint32_t)pkt[11] << 24,
                .x            = (int16_t)(pkt[12] | pkt[13] << 8),
                .y            = (int16_t)(pkt[14] | pkt[15] << 8),
                .z            = (int16_t)(pkt[16] | pkt[17] << 8),
                .link_drops   = pkt[18],
            };
            pkt_n = 0;
            handle_packet(&p);
            return;
        }

        st.crc_errors++;
        memmove(pkt, pkt + 1, --pkt_n);   // resync from the next byte
    }
}

// ---------------------------------------------------------------------------
// SPI slave + DMA
//
// RX: a DMA channel copies every received byte into a 16 KB ring buffer, so
// nothing is lost while printf waits on USB (~10 s of slack at the link's
// data rate). The main loop reads the ring behind the DMA's write pointer.
//
// TX: a second channel keeps the TX FIFO topped up with 0x00. The link is
// one-way and the TX pin is not connected, but this way the PL022 never runs
// as a slave with an empty TX FIFO.
// ---------------------------------------------------------------------------
#define RING_BITS  14
#define RING_SIZE  (1u << RING_BITS)
#define DMA_COUNT  0x0FFFFFFFu        // ~37 h at the link's data rate; restarted when it runs out

static uint8_t rx_ring[RING_SIZE] __attribute__((aligned(RING_SIZE)));
static uint8_t tx_zero = 0x00;
static int     rx_dma, tx_dma;
static uint32_t rx_rd = 0;            // next ring index to parse

static uint32_t rx_wr_index(void)
{
    return (dma_channel_hw_addr(rx_dma)->write_addr - (uintptr_t)rx_ring) & (RING_SIZE - 1);
}

static void start_rx_dma(void)
{
    dma_channel_config c = dma_channel_get_default_config(rx_dma);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_ring(&c, true, RING_BITS);              // wrap the write address
    channel_config_set_dreq(&c, spi_get_dreq(SPI_PORT, false));
    // Continue from where the ring is, so the parser's index stays valid.
    dma_channel_configure(rx_dma, &c, &rx_ring[rx_wr_index()],
                          &spi_get_hw(SPI_PORT)->dr, DMA_COUNT, true);
}

static void start_tx_dma(void)
{
    dma_channel_config c = dma_channel_get_default_config(tx_dma);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, spi_get_dreq(SPI_PORT, true));
    dma_channel_configure(tx_dma, &c, &spi_get_hw(SPI_PORT)->dr, &tx_zero, DMA_COUNT, true);
}

static void link_init(void)
{
    // Pulls first: with the FPGA unpowered or unprogrammed, CS reads high
    // (not selected) and SCK idles high (mode 3) instead of floating.
    gpio_pull_up(PIN_CS);
    gpio_pull_up(PIN_SCK);
    gpio_pull_down(PIN_RX);

    // Enable as a slave before routing the pins, so the PL022 never drives
    // SCK as a master while the FPGA is driving it.
    spi_init(SPI_PORT, LINK_SPI_HZ);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    spi_set_slave(SPI_PORT, true);

    gpio_set_function(PIN_RX,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);

    while (spi_is_readable(SPI_PORT))                         // drop anything stale
        (void)spi_get_hw(SPI_PORT)->dr;

    rx_dma = dma_claim_unused_channel(true);
    tx_dma = dma_claim_unused_channel(true);
    dma_channel_hw_addr(rx_dma)->write_addr = (uintptr_t)rx_ring;   // so rx_wr_index() starts at 0
    start_rx_dma();
    start_tx_dma();
}

static void link_keep_running(void)
{
    if (!dma_channel_is_busy(rx_dma)) start_rx_dma();
    if (!dma_channel_is_busy(tx_dma)) start_tx_dma();
}

// Once a second: report new link errors, or that the FPGA has gone quiet.
static void link_health(void)
{
    static uint32_t last_bytes, last_errs, last_lost;
    uint32_t now = time_us_32();
    if (now - last_check_us < 1000000) return;
    last_check_us = now;

    if (st.crc_errors != last_errs || st.lost != last_lost) {
        printf("[link] %lu corrupt and %lu lost packets in the last second\n",
               (unsigned long)(st.crc_errors - last_errs), (unsigned long)(st.lost - last_lost));
        last_errs = st.crc_errors;
        last_lost = st.lost;
    }

    if (link_up && now - last_pkt_us > LINK_TIMEOUT_US) {
        link_up   = false;
        have_prev = false;      // don't count the outage as lost packets; reprint status when back
        printf("[link] DOWN: no valid packet from the FPGA for 1 s\n");
    }
    if (!link_up) {
        if (st.bytes == last_bytes)
            printf("[link] nothing received: is the FPGA programmed? check GND, "
                   "GP17<-pico_cs_n, GP18<-pico_sclk, GP16<-pico_mosi\n");
        else
            printf("[link] bytes arriving but no valid packets: check GP16 (data) wiring, "
                   "that the link runs SPI mode 3, and the SCLK speed\n");
    }
    last_bytes = st.bytes;
}

// ---------------------------------------------------------------------------
int main(void)
{
    stdio_init_all();

    // USB CDC discards anything printed before the host opens the port,
    // so hold here until PuTTY (or similar) actually connects.
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    sleep_ms(200);

    printf("\n=== ADXL375 test rig: Pico 2 W as SPI slave of the FPGA ===\n");
    link_init();
    last_check_us = time_us_32();       // first link check one second from now
    printf("SPI0 slave ready: mode 3, 8-bit MSB first, CSn=GP%d SCK=GP%d RX=GP%d\n",
           PIN_CS, PIN_SCK, PIN_RX);
    printf("Keys: p = pause/resume samples, s = link statistics\n");
    printf("Waiting for packets from the FPGA...\n\n");

    while (true) {
        uint32_t wr = rx_wr_index();
        while (rx_rd != wr) {
            parse_byte(rx_ring[rx_rd]);
            rx_rd = (rx_rd + 1) & (RING_SIZE - 1);
        }

        int c = getchar_timeout_us(0);
        if (c == 'p' || c == 'P') {
            print_samples = !print_samples;
            printf("-- sample lines %s --\n", print_samples ? "resumed" : "paused");
        } else if (c == 's' || c == 'S') {
            print_link_stats();
            if (have_prev) print_status(&prev);
        }

        link_keep_running();
        link_health();
        tight_loop_contents();
    }
}
