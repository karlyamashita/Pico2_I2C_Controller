#include <stdio.h>

#include <stdlib.h>

#include <string.h>

#include <ctype.h>

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"


#include "hardware/i2c.h"
#include "hardware/spi.h"

#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/pio.h"
#include "hardware/pio_instructions.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "i2c_sniff.pio.h"
#include "spi_sniff.pio.h"

#include "hardware/regs/i2c.h"



// EXPERIMENTAL RP2040 OVERCLOCK: 200 MHz is above the rated 133 MHz.
// Set to 0 to revert to the SDK's normal startup clock.
// Must run before CYW43, USB, SPI, I2C and PIO initialization.
#define SOH_EXPERIMENTAL_200MHZ 0

#include "pico/cyw43_arch.h"
#define SOH_LED_PIN CYW43_WL_GPIO_LED_PIN  // Pico 2 W onboard LED via CYW43
#define FW_VERSION "PICO2W-R1-PORT"

#define SDA_PIN 4

#define SCL_PIN 5

#define SLAVE_SDA_PIN 6

#define SLAVE_SCL_PIN 7
#define SNIFF_SDA_PIN 8
#define SNIFF_SCL_PIN 9

#define DEFAULT_SLAVE_ADDR 0x42
static uint8_t test_slave_addr = DEFAULT_SLAVE_ADDR;

#define MAX_DATA 128

#define LINE_SIZE 768

static uint32_t bus_hz = 100000;



// SPI0 master: GP16 RX, GP17 manual CS, GP18 SCK, GP19 TX.
// SPI1 slave: GP12 RX, GP13 CS, GP14 SCK, GP15 TX.
// GP20 MOSI, GP21 MISO, GP22 SCK, GP26 CS: passive PIO/DMA sniffer.
#define SPI_MASTER_CS 17
#define SPI_FIXED_MAX 128
static uint8_t spi_fixed[SPI_FIXED_MAX] = {0x11,0x22,0x33,0x44};
static uint32_t spi_fixed_len = 4;
static volatile uint32_t spi_slave_rx_count = 0;
static uint8_t spi_slave_rx[SPI_FIXED_MAX];
static uint8_t spi_dma_tx[SPI_FIXED_MAX];
static uint8_t spi_dma_rx[SPI_FIXED_MAX];
static uint32_t spi_baud = 100000;
static uint32_t spi_mode = 1;
static int spi_dma_tx_ch = -1, spi_dma_rx_ch = -1;
static uint32_t spi_dma_transactions = 0, spi_dma_failures = 0;
static uint32_t spi_dma_rx_done = 0, spi_dma_tx_done = 0;
static uint32_t spi_slave_overruns = 0;

// R6: one-shot GPIO edge trace. GPIO input sampling works even when a pin
// is muxed to the SPI peripheral. Intended for low speed (10 kHz) debugging.
#define SPI_TRACE_MAX 128u
static volatile bool spi_trace_armed, spi_trace_running, spi_trace_done;
static volatile uint32_t spi_trace_cs17_fall, spi_trace_cs17_rise;
static volatile uint32_t spi_trace_cs13_fall, spi_trace_cs13_rise;
static volatile uint32_t spi_trace_cs26_fall, spi_trace_cs26_rise;
static volatile uint32_t spi_trace_sck_rise, spi_trace_sck_fall;
static volatile uint32_t spi_trace_sck_while_cs13, spi_trace_sck_while_cs17;
static volatile uint32_t spi_trace_cs13_high_on_clock;
static volatile uint32_t spi_trace_first_levels, spi_trace_last_levels;
static volatile uint32_t spi_trace_bits, spi_trace_bytes;
static volatile uint8_t spi_trace_mosi[SPI_TRACE_MAX], spi_trace_miso[SPI_TRACE_MAX];
static volatile uint8_t spi_trace_shift_mosi, spi_trace_shift_miso;
static volatile uint32_t spi_trace_start_us, spi_trace_end_us;

static uint32_t spi_trace_levels(void) {
    // bit0=GP17 CS out, bit1=GP13 CS in, bit2=GP26 sniffer CS,
    // bit3=GP18 SCK, bit4=GP19 MOSI, bit5=GP15 MISO.
    return (gpio_get(17) ? 1u : 0u) | (gpio_get(13) ? 2u : 0u) |
           (gpio_get(26) ? 4u : 0u) | (gpio_get(18) ? 8u : 0u) |
           (gpio_get(19) ? 16u : 0u) | (gpio_get(15) ? 32u : 0u);
}
static void spi_trace_gpio_irq(uint gpio, uint32_t events) {
    if (!spi_trace_armed) return;
    if (gpio == 13) {
        if (events & GPIO_IRQ_EDGE_FALL) ++spi_trace_cs13_fall;
        if (events & GPIO_IRQ_EDGE_RISE) ++spi_trace_cs13_rise;
    } else if (gpio == 26) {
        if (events & GPIO_IRQ_EDGE_FALL) ++spi_trace_cs26_fall;
        if (events & GPIO_IRQ_EDGE_RISE) ++spi_trace_cs26_rise;
    } else if (gpio == 17) {
        if (events & GPIO_IRQ_EDGE_FALL) {
            ++spi_trace_cs17_fall;
            if (!spi_trace_done && !spi_trace_running) {
                spi_trace_running = true;
                spi_trace_start_us = time_us_32();
                spi_trace_first_levels = spi_trace_levels();
            }
        }
        if (events & GPIO_IRQ_EDGE_RISE) {
            ++spi_trace_cs17_rise;
            if (spi_trace_running) {
                spi_trace_last_levels = spi_trace_levels();
                spi_trace_end_us = time_us_32();
                spi_trace_running = false;
                spi_trace_done = true;
            }
        }
    } else if (gpio == 18 && spi_trace_running) {
        if (events & GPIO_IRQ_EDGE_RISE) ++spi_trace_sck_rise;
        if (events & GPIO_IRQ_EDGE_FALL) ++spi_trace_sck_fall;
        uint32_t pins = spi_trace_levels();
        if (!(pins & 1u)) ++spi_trace_sck_while_cs17;
        if (!(pins & 2u)) ++spi_trace_sck_while_cs13;
        else ++spi_trace_cs13_high_on_clock;
        bool leading = (events & ((spi_mode & 2u) ? GPIO_IRQ_EDGE_FALL : GPIO_IRQ_EDGE_RISE)) != 0;
        bool sample = (spi_mode & 1u) ? !leading : leading;
        if (sample && spi_trace_bytes < SPI_TRACE_MAX) {
            spi_trace_shift_mosi = (uint8_t)((spi_trace_shift_mosi << 1) | !!(pins & 16u));
            spi_trace_shift_miso = (uint8_t)((spi_trace_shift_miso << 1) | !!(pins & 32u));
            if (++spi_trace_bits == 8) {
                spi_trace_mosi[spi_trace_bytes] = spi_trace_shift_mosi;
                spi_trace_miso[spi_trace_bytes] = spi_trace_shift_miso;
                ++spi_trace_bytes;
                spi_trace_bits = 0;
                spi_trace_shift_mosi = spi_trace_shift_miso = 0;
            }
        }
    }
}
static void spi_trace_off(void) {
    spi_trace_armed = false;
    const uint32_t mask = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
    gpio_set_irq_enabled(17, mask, false);
    gpio_set_irq_enabled(13, mask, false);
    gpio_set_irq_enabled(26, mask, false);
    gpio_set_irq_enabled(18, mask, false);
}
static void spi_sniff_stop(void);
static volatile bool spi_sniff_active;
// SLA stream is opt-in; legacy SPI SNIFF diagnostics remain unchanged.
static bool spi_sla_stream = false;
static volatile uint64_t spi_sla_frame_start_us = 0;
static volatile uint64_t spi_sla_ready_start_us = 0;
static volatile uint64_t spi_sla_ready_end_us = 0;
static volatile uint32_t spi_sla_capture_id = 0;
static volatile uint32_t spi_sla_packet_id = 0;

static void spi_trace_arm(void) {
    if (spi_sniff_active) spi_sniff_stop();
    spi_trace_off();
    spi_trace_running = spi_trace_done = false;
    spi_trace_cs17_fall = spi_trace_cs17_rise = 0;
    spi_trace_cs13_fall = spi_trace_cs13_rise = 0;
    spi_trace_cs26_fall = spi_trace_cs26_rise = 0;
    spi_trace_sck_rise = spi_trace_sck_fall = 0;
    spi_trace_sck_while_cs13 = spi_trace_sck_while_cs17 = 0;
    spi_trace_cs13_high_on_clock = 0;
    spi_trace_bits = spi_trace_bytes = 0;
    spi_trace_shift_mosi = spi_trace_shift_miso = 0;
    spi_trace_start_us = spi_trace_end_us = 0;
    spi_trace_first_levels = spi_trace_last_levels = 0;
    const uint32_t mask = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
    // R10: the PIO sniffer may leave GP26 muxed to PIO1 after STOP.
    // Return it to SIO input for GPIO IRQ-based CS tracing.
    // SPI NEW CAPTURE restores the PIO function when needed.
    gpio_set_function(26, GPIO_FUNC_SIO);
    gpio_set_dir(26, GPIO_IN);
    gpio_disable_pulls(26);
    spi_trace_armed = true;
    gpio_set_irq_enabled_with_callback(17, mask, true, spi_trace_gpio_irq);
    gpio_set_irq_enabled(13, mask, true);
    gpio_set_irq_enabled(26, mask, true);
    gpio_set_irq_enabled(18, mask, true);
}
static void spi_trace_status(void) {
    printf("SPI_TRACE: ARMED=%u DONE=%u CS17_FALL=%lu RISE=%lu CS13_FALL=%lu RISE=%lu CS26_FALL=%lu RISE=%lu\n",
           spi_trace_armed, spi_trace_done, (unsigned long)spi_trace_cs17_fall,
           (unsigned long)spi_trace_cs17_rise, (unsigned long)spi_trace_cs13_fall,
           (unsigned long)spi_trace_cs13_rise, (unsigned long)spi_trace_cs26_fall,
           (unsigned long)spi_trace_cs26_rise);
    printf("SPI_TRACE: SCK_RISE=%lu FALL=%lu CS17_LOW_EDGES=%lu CS13_LOW_EDGES=%lu CS13_HIGH_EDGES=%lu BYTES=%lu PARTIAL_BITS=%lu DURATION_US=%lu LEVELS_FIRST=0x%02lX LAST=0x%02lX NOW=0x%02lX\n",
           (unsigned long)spi_trace_sck_rise, (unsigned long)spi_trace_sck_fall,
           (unsigned long)spi_trace_sck_while_cs17, (unsigned long)spi_trace_sck_while_cs13,
           (unsigned long)spi_trace_cs13_high_on_clock, (unsigned long)spi_trace_bytes,
           (unsigned long)spi_trace_bits, (unsigned long)(spi_trace_end_us - spi_trace_start_us),
           (unsigned long)spi_trace_first_levels, (unsigned long)spi_trace_last_levels,
           (unsigned long)spi_trace_levels());
}
static void spi_trace_data(void) {
    printf("SPI_TRACE MOSI:");
    for (uint32_t i = 0; i < spi_trace_bytes; ++i) printf(" %02X", spi_trace_mosi[i]);
    printf("\nSPI_TRACE MISO:");
    for (uint32_t i = 0; i < spi_trace_bytes; ++i) printf(" %02X", spi_trace_miso[i]);
    puts("");
}

// R5: SPI1 slave uses DMA, not CPU interrupts, for every local SPI XFER.
// The RP2040 has a single core and a blocking SPI0 master call; DMA lets
// SPI1 service its FIFOs even when the core is busy driving SPI0.
// R10: register snapshots from before and after SPI1 initialization.
static uint32_t spi_init_calls;
static uint32_t spi_init_cr0_before, spi_init_cr1_before, spi_init_sr_before;
static uint32_t spi_init_cr0_after, spi_init_cr1_after, spi_init_sr_after;
static uint32_t spi_init_cpsr_after, spi_init_dmacr_after;

static void spi_slave_prime(bool reset_history) {
    spi_hw_t *hw = spi_get_hw(spi1);
    ++spi_init_calls;
    spi_init_cr0_before = hw->cr0;
    spi_init_cr1_before = hw->cr1;
    spi_init_sr_before = hw->sr;
    hw->imsc = 0;
    hw->dmacr = 0;
    spi_deinit(spi1);
    spi_init(spi1, spi_baud);
    spi_cpol_t cpol = (spi_mode & 2u) ? SPI_CPOL_1 : SPI_CPOL_0;
    spi_cpha_t cpha = (spi_mode & 1u) ? SPI_CPHA_1 : SPI_CPHA_0;
    spi_set_format(spi1, 8, cpol, cpha, SPI_MSB_FIRST);
    spi_set_slave(spi1, true);
    spi_init_cr0_after = hw->cr0;
    spi_init_cr1_after = hw->cr1;
    spi_init_sr_after = hw->sr;
    spi_init_cpsr_after = hw->cpsr;
    spi_init_dmacr_after = hw->dmacr;
    if (reset_history) spi_slave_rx_count = 0;
}

static void spi_setup(void) {
    spi_init(spi0, spi_baud);
    gpio_set_function(16, GPIO_FUNC_SPI);
    gpio_set_function(18, GPIO_FUNC_SPI);
    gpio_set_function(19, GPIO_FUNC_SPI);
    gpio_init(SPI_MASTER_CS);
    gpio_set_dir(SPI_MASTER_CS, GPIO_OUT);
    gpio_put(SPI_MASTER_CS, true);
    for (int pin = 12; pin <= 15; ++pin) gpio_set_function(pin, GPIO_FUNC_SPI);
    spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);
    spi_dma_tx_ch = dma_claim_unused_channel(true);
    spi_dma_rx_ch = dma_claim_unused_channel(true);
    spi_slave_prime(true);
}

static void spi_set_mode(uint32_t mode) {
    spi_cpol_t cpol = (mode & 2u) ? SPI_CPOL_1 : SPI_CPOL_0;
    spi_cpha_t cpha = (mode & 1u) ? SPI_CPHA_1 : SPI_CPHA_0;
    spi_set_format(spi0, 8, cpol, cpha, SPI_MSB_FIRST);
    spi_mode = mode;
    spi_slave_prime(true);
}

static void spi_dma_transfer(const uint8_t *tx, uint8_t *rx, uint32_t n) {
    spi_hw_t *hw = spi_get_hw(spi1);
    // Start with a clean SPI1 FIFO and exactly n response bytes.
    spi_slave_prime(false);
    for (uint32_t i = 0; i < n; ++i)
        spi_dma_tx[i] = spi_fixed[i % spi_fixed_len];
    memset(spi_dma_rx, 0, n);

    dma_channel_config rc = dma_channel_get_default_config(spi_dma_rx_ch);
    channel_config_set_transfer_data_size(&rc, DMA_SIZE_8);
    channel_config_set_read_increment(&rc, false);
    channel_config_set_write_increment(&rc, true);
    channel_config_set_dreq(&rc, spi_get_dreq(spi1, false));
    dma_channel_configure(spi_dma_rx_ch, &rc, spi_dma_rx,
                          &hw->dr, n, false);

    dma_channel_config tc = dma_channel_get_default_config(spi_dma_tx_ch);
    channel_config_set_transfer_data_size(&tc, DMA_SIZE_8);
    channel_config_set_read_increment(&tc, true);
    channel_config_set_write_increment(&tc, false);
    channel_config_set_dreq(&tc, spi_get_dreq(spi1, true));
    dma_channel_configure(spi_dma_tx_ch, &tc, &hw->dr,
                          spi_dma_tx, n, false);

    // Enable SPI1 DMA requests before asserting chip select.
    hw->dmacr = SPI_SSPDMACR_RXDMAE_BITS | SPI_SSPDMACR_TXDMAE_BITS;
    dma_start_channel_mask((1u << spi_dma_rx_ch) | (1u << spi_dma_tx_ch));
    // Ensure the first TX byte is already queued before CS goes low.
    absolute_time_t prime_deadline = make_timeout_time_us(10000);
    while ((hw->sr & SPI_SSPSR_TFE_BITS) && !time_reached(prime_deadline)) tight_loop_contents();

    gpio_put(SPI_MASTER_CS, false);
    busy_wait_us_32(10);
    int result = spi_write_read_blocking(spi0, tx, rx, n);
    busy_wait_us_32(10);
    gpio_put(SPI_MASTER_CS, true);

    // Wait briefly for the final SPI1 RX DMA request to be serviced.
    absolute_time_t deadline = make_timeout_time_us(10000);
    while (dma_channel_is_busy(spi_dma_rx_ch) && !time_reached(deadline)) tight_loop_contents();
    spi_dma_rx_done = n - dma_channel_hw_addr(spi_dma_rx_ch)->transfer_count;
    spi_dma_tx_done = n - dma_channel_hw_addr(spi_dma_tx_ch)->transfer_count;
    if (hw->ris & SPI_SSPRIS_RORRIS_BITS) ++spi_slave_overruns;
    hw->dmacr = 0;
    dma_channel_abort(spi_dma_tx_ch);
    dma_channel_abort(spi_dma_rx_ch);
    ++spi_dma_transactions;
    if (result != (int)n || spi_dma_rx_done != n || spi_dma_tx_done != n)
        ++spi_dma_failures;
    for (uint32_t i = 0; i < spi_dma_rx_done; ++i) {
        spi_slave_rx[(spi_slave_rx_count + i) % SPI_FIXED_MAX] = spi_dma_rx[i];
    }
    spi_slave_rx_count += spi_dma_rx_done;
    if (result != (int)n) printf("ERR SPI XFER=%d\n", result);
    else {
        printf("SPI_RX:");
        for (uint32_t i = 0; i < n; ++i) printf(" %02X", rx[i]);
        puts("");
        if (spi_dma_rx_done != n || spi_dma_tx_done != n)
            printf("ERR SPI_SLAVE_DMA RX=%lu/%lu TX=%lu/%lu\n",
                (unsigned long)spi_dma_rx_done, (unsigned long)n,
                (unsigned long)spi_dma_tx_done, (unsigned long)n);
        else puts("OK");
    }
}


// R8: pulse CS between bytes for CPHA=0 SPI slave framing.
// This path does not use SPI1 DMA. It primes the 4-byte slave FIFO once,
// and clocks one byte at a time with CS high between bytes.
static void spi_cs_pulse_test(const uint8_t *tx, uint32_t n) {
    if (n == 0 || n > 8) { puts("ERR SPI CS PULSE XFER requires 1..8 bytes"); return; }
    spi_hw_t *sh = spi_get_hw(spi1);
    uint8_t master_rx[8] = {0};
    uint8_t slave_rx[8] = {0};
    uint32_t received = 0;
    spi_slave_prime(false);
    sh->imsc = 0;
    sh->dmacr = 0;
    while (sh->sr & SPI_SSPSR_RNE_BITS) (void)sh->dr;
    uint32_t queued = 0;
    for (uint32_t i = 0; i < n; ++i) {
        if (!(sh->sr & SPI_SSPSR_TNF_BITS)) break;
        sh->dr = spi_fixed[i % spi_fixed_len];
        ++queued;
    }
    for (uint32_t i = 0; i < n; ++i) {
        gpio_put(SPI_MASTER_CS, false);
        busy_wait_us_32(20);
        int done = spi_write_read_blocking(spi0, &tx[i], &master_rx[i], 1);
        busy_wait_us_32(20);
        gpio_put(SPI_MASTER_CS, true);
        busy_wait_us_32(100);
        while ((sh->sr & SPI_SSPSR_RNE_BITS) && received < n)
            slave_rx[received++] = (uint8_t)sh->dr;
    }
    for (uint32_t i = 0; i < received; ++i)
        spi_slave_rx[(spi_slave_rx_count + i) % SPI_FIXED_MAX] = slave_rx[i];
    spi_slave_rx_count += received;
    printf("SPI_RX:");
    for (uint32_t i = 0; i < n; ++i) printf(" %02X", master_rx[i]);
    printf("\n");
    if (received != n || queued != n)
        printf("ERR SPI_SLAVE_RX=%lu/%lu TX_QUEUED=%lu/%lu\n",
               (unsigned long)received, (unsigned long)n,
               (unsigned long)queued, (unsigned long)n);
}

// I2C1 test target: simple 256-byte register map, first written byte selects offset.

// Only enabled during SELFTEST; normal I2C0 master commands remain unchanged.

static volatile uint8_t target_mem[256];

static volatile uint8_t target_reg;

static volatile bool target_expect_reg;

static volatile uint32_t target_reads, target_writes;



static void i2c1_target_irq(void) {

    i2c_hw_t *hw = i2c_get_hw(i2c1);

    uint32_t status = hw->raw_intr_stat;

    if (status & I2C_IC_RAW_INTR_STAT_RX_FULL_BITS) {

        while (hw->status & I2C_IC_STATUS_RFNE_BITS) {

            uint8_t value = (uint8_t)hw->data_cmd;

            if (target_expect_reg) { target_reg = value; target_expect_reg = false; }

            else { target_mem[target_reg++] = value; ++target_writes; }

        }

    }

    if (status & I2C_IC_RAW_INTR_STAT_RD_REQ_BITS) {

        (void)hw->clr_rd_req;

        hw->data_cmd = target_mem[target_reg++];

        ++target_reads;

    }

    if (status & I2C_IC_RAW_INTR_STAT_STOP_DET_BITS) {

        (void)hw->clr_stop_det;

        target_expect_reg = true;

    }

}

static void target_enable(void) {

    target_reg = 0;

    target_expect_reg = true;

    target_reads = target_writes = 0;

    for (int i=0;i<256;i++) target_mem[i] = 0;

    i2c_init(i2c1, 100000);

    gpio_set_function(SLAVE_SDA_PIN, GPIO_FUNC_I2C);

    gpio_set_function(SLAVE_SCL_PIN, GPIO_FUNC_I2C);

    gpio_pull_up(SLAVE_SDA_PIN);

    gpio_pull_up(SLAVE_SCL_PIN);

    i2c_set_slave_mode(i2c1, true, test_slave_addr);

    irq_set_exclusive_handler(I2C1_IRQ, i2c1_target_irq);

    i2c_get_hw(i2c1)->intr_mask = I2C_IC_INTR_MASK_M_RX_FULL_BITS |

        I2C_IC_INTR_MASK_M_RD_REQ_BITS | I2C_IC_INTR_MASK_M_STOP_DET_BITS;

    irq_set_enabled(I2C1_IRQ, true);

}

static void target_disable(void) {

    irq_set_enabled(I2C1_IRQ, false);

    i2c_get_hw(i2c1)->intr_mask = 0;

    i2c_set_slave_mode(i2c1, false, 0);

    i2c_deinit(i2c1);

    gpio_set_function(SLAVE_SDA_PIN, GPIO_FUNC_SIO);

    gpio_set_function(SLAVE_SCL_PIN, GPIO_FUNC_SIO);

    gpio_disable_pulls(SLAVE_SDA_PIN);

    gpio_disable_pulls(SLAVE_SCL_PIN);

}


// R6: dual chained finite DMA channels, IRQ rearming, foreground decode.
// A late IRQ or slow foreground decoder may still lose samples.
#define SNIFF_BLOCK_WORDS 1024u
#define SNIFF_SAMPLE_HZ 1000000u
#define LOGIC_SAMPLE_HZ 4000000u
static bool logic_armed, logic_triggered, logic_ready;
static uint8_t logic_previous = 3u;
static uint32_t logic_capture_id;
#define SNIFF_EVENT_LIMIT 32u
static uint32_t sniff_buffers[2][SNIFF_BLOCK_WORDS];
static bool sniff_active, sniff_decode = false;
static PIO sniff_pio = pio0;
static uint sniff_sm, sniff_offset;
static int sniff_dma[2] = {-1, -1};
static volatile uint32_t sniff_blocks, sniff_words, sniff_rearms, sniff_gaps, sniff_overruns;
static volatile uint32_t sniff_completed[2];
static uint32_t sniff_processed[2];
static volatile uint32_t sniff_irq_late;
static uint32_t sniff_events, sniff_suppressed;

static bool sniff_prev_valid, sniff_frame, sniff_address_next, sniff_expect_ack;
static uint8_t sniff_prev, sniff_bits, sniff_byte;
static uint32_t sniff_print_budget;
static uint32_t sniff_last_sequence;

static void sniff_emit(char type, uint8_t byte, bool ack) {
    ++sniff_events;
    if (!sniff_print_budget) { ++sniff_suppressed; return; }
    --sniff_print_budget;
    if (type == 'S') puts("SNIFF: START");
    else if (type == 'R') puts("SNIFF: RESTART");
    else if (type == 'P') puts("SNIFF: STOP");
    else if (type == 'A') printf("SNIFF: ADDR=0x%02X %s %s\n", byte >> 1,
                                 (byte & 1) ? "READ" : "WRITE", ack ? "ACK" : "NACK");
    else if (type == 'B') printf("SNIFF: DATA=%02X %s\n", byte, ack ? "ACK" : "NACK");
}
static void sniff_reset_decoder(void) {
    sniff_prev_valid = sniff_frame = sniff_address_next = sniff_expect_ack = false;
    sniff_prev = sniff_bits = sniff_byte = 0;
}
static void sniff_sample(uint8_t pins) {
    if (sniff_prev_valid) {
        bool osda = (sniff_prev & 1u) != 0, oscl = (sniff_prev & 2u) != 0;
        bool sda = (pins & 1u) != 0, scl = (pins & 2u) != 0;
        if (oscl && scl && osda != sda) {
            if (!sda) {
                sniff_emit(sniff_frame ? 'R' : 'S', 0, false);
                sniff_frame = sniff_address_next = true;
            } else if (sniff_frame) {
                sniff_emit('P', 0, false);
                sniff_frame = false;
            }
            sniff_bits = sniff_byte = 0;
            sniff_expect_ack = false;
        } else if (sniff_frame && !oscl && scl) {
            if (sniff_expect_ack) {
                sniff_emit(sniff_address_next ? 'A' : 'B', sniff_byte, !sda);
                sniff_address_next = false;
                sniff_expect_ack = false;
                sniff_bits = sniff_byte = 0;
            } else {
                sniff_byte = (uint8_t)((sniff_byte << 1) | (sda ? 1u : 0u));
                if (++sniff_bits == 8u) sniff_expect_ack = true;
            }
        }
    }
    sniff_prev = pins;
    sniff_prev_valid = true;
}
// R25: continuous I2C transaction streaming. Capture and USB transmission are
// decoupled: the DMA consumer only enqueues complete transactions.
#define LOGIC_MAX_EDGES 16384u
#define LOGIC_PACKET_EDGES 512u
// R5.3: 24 * ~8 KiB = ~192 KiB of static outgoing packet storage.
// Avoid 64 slots: that alone would exceed the RP2350's 520 KiB SRAM.
#define LOGIC_PACKET_SLOTS 24u
typedef struct { uint64_t tick; uint8_t pins; } logic_edge_t;
typedef struct {
    uint32_t packet_id, count;
    logic_edge_t edges[LOGIC_PACKET_EDGES];
} logic_packet_t;
static logic_packet_t logic_packets[LOGIC_PACKET_SLOTS];
static uint64_t logic_ticks;
static uint32_t logic_edge_count, logic_stops;
static uint32_t logic_packet_number, logic_dropped_packets;
static uint32_t logic_queue_head, logic_queue_tail, logic_queue_count;
static uint32_t logic_queue_high_water, logic_queue_full_drops, logic_edge_limit_drops;
static uint32_t logic_usb_sent;
static uint32_t logic_tx_index;
static bool logic_overflow, logic_packet_open, logic_packet_discard;
static uint8_t logic_tx_phase;
static void logic_record_buffer(uint32_t index) {
    if (!logic_armed) return;
    for (uint32_t i = 0; i < SNIFF_BLOCK_WORDS; ++i) {
        uint32_t word = sniff_buffers[index][i];
        for (int shift = 30; shift >= 0; shift -= 2) {
            uint8_t sample = (uint8_t)((word >> shift) & 3u);
            uint8_t previous = logic_previous;
            logic_previous = sample;
            // Timestamp advances even in idle time; the PC can measure real gaps.
            ++logic_ticks;
            if (!logic_packet_open) {
                if (previous == 3u && sample == 2u) {
                    logic_triggered = true;
                    logic_packet_open = true;
                    logic_packet_discard = false;
                    logic_edge_count = 0;
                    if (logic_queue_count == LOGIC_PACKET_SLOTS) {
                        logic_packet_discard = true;
                        ++logic_dropped_packets;
                        ++logic_queue_full_drops;
                    } else {
                        logic_packet_t *p = &logic_packets[logic_queue_head];
                        p->edges[logic_edge_count++] = (logic_edge_t){logic_ticks - 1u, 3u};
                        p->edges[logic_edge_count++] = (logic_edge_t){logic_ticks, 2u};
                    }
                }
                continue;
            }
            if (previous == sample) continue;
            if (!logic_packet_discard) {
                if (logic_edge_count >= LOGIC_PACKET_EDGES) {
                    logic_packet_discard = true;
                    logic_overflow = true;
                    ++logic_dropped_packets;
                    ++logic_edge_limit_drops;
                } else {
                    logic_packets[logic_queue_head].edges[logic_edge_count++] =
                        (logic_edge_t){logic_ticks, sample};
                }
            }
            // STOP: SDA rises with SCL high; repeated START remains in packet.
            if (previous == 2u && sample == 3u) {
                ++logic_stops;
                ++logic_packet_number;
                if (!logic_packet_discard) {
                    logic_packet_t *p = &logic_packets[logic_queue_head];
                    p->packet_id = logic_packet_number;
                    p->count = logic_edge_count;
                    logic_queue_head = (logic_queue_head + 1u) % LOGIC_PACKET_SLOTS;
                    ++logic_queue_count;
                    if (logic_queue_count > logic_queue_high_water)
                        logic_queue_high_water = logic_queue_count;
                }
                logic_packet_open = false;
                logic_edge_count = 0;
            }
        }
    }
}
// One USB line per foreground iteration: do not block DMA processing for
// an entire transaction. A slow host may still cause queue overruns.
static void logic_stream_step(void) {
    if (logic_queue_count == 0u) return;
    logic_packet_t *p = &logic_packets[logic_queue_tail];
    if (logic_tx_phase == 0u) {
        printf("LOGIC_BEGIN I2C ID=%lu PACKET=%lu FIRST=%u SAMPLE_HZ=%lu COUNT=%lu FORMAT=EDGE_TICKS\n",
               (unsigned long)logic_capture_id, (unsigned long)p->packet_id,
               p->packet_id == 1u ? 1u : 0u, (unsigned long)LOGIC_SAMPLE_HZ,
               (unsigned long)p->count);
        logic_tx_index = 0;
        logic_tx_phase = 1;
    } else if (logic_tx_phase == 1u) {
        logic_edge_t e = p->edges[logic_tx_index++];
        printf("LOGIC_EDGE %llu,%u\n", (unsigned long long)e.tick, (unsigned)e.pins);
        if (logic_tx_index == p->count) logic_tx_phase = 2;
    } else {
        printf("LOGIC_END ID=%lu PACKET=%lu COUNT=%lu STOPS=1 OVERFLOW=0\n",
               (unsigned long)logic_capture_id, (unsigned long)p->packet_id,
               (unsigned long)p->count);
        logic_queue_tail = (logic_queue_tail + 1u) % LOGIC_PACKET_SLOTS;
        --logic_queue_count;
        ++logic_usb_sent;
        logic_tx_phase = 0;
    }
}
static void sniff_decode_buffer(uint32_t index) {
    // PIO shifts left, so oldest 2-bit sample is in bits 31:30.
    for (uint32_t i = 0; i < SNIFF_BLOCK_WORDS; ++i) {
        uint32_t word = sniff_buffers[index][i];
        for (int shift = 30; shift >= 0; shift -= 2)
            sniff_sample((uint8_t)((word >> shift) & 3u));
    }
}
// DMA chaining: when channel A finishes it starts B, and vice versa.
// IRQ0 reloads the completed channel before it is needed again.
static void sniff_dma_irq(void) {
    for (unsigned i = 0; i < 2; ++i) {
        int ch = sniff_dma[i];
        if (ch < 0 || !(dma_hw->ints0 & (1u << ch))) continue;
        dma_hw->ints0 = 1u << ch;
        if (!sniff_active) continue;
        // A completed buffer remains valid until this same channel runs again.
        ++sniff_completed[i];
        ++sniff_blocks;
        sniff_words += SNIFF_BLOCK_WORDS;
        // Reload without triggering; the other channel will chain back here.
        dma_channel_set_write_addr(ch, sniff_buffers[i], false);
        dma_channel_set_trans_count(ch, SNIFF_BLOCK_WORDS, false);
        ++sniff_rearms;
        // If the opposite channel already completed, the IRQ was too late.
        if (!dma_channel_is_busy(sniff_dma[i ^ 1u])) ++sniff_irq_late;
    }
}
static void sniff_update(void) {
    if (!sniff_active) return;
    // Process completed buffers in acquisition order, not channel order.
    // The IRQ counts completions; processing cannot block DMA.
    for (unsigned budget = 0; budget < 2; ++budget) {
        uint32_t a = sniff_completed[0], b = sniff_completed[1];
        uint32_t pa = sniff_processed[0], pb = sniff_processed[1];
        if (a == pa && b == pb) break;
        unsigned index;
        if (a != pa && b != pb) index = (pa <= pb) ? 0u : 1u;
        else index = (a != pa) ? 0u : 1u;
        uint32_t pending = sniff_completed[index] - sniff_processed[index];
        if (pending > 1u || dma_channel_is_busy(sniff_dma[index])) {
            // Buffer already reused; don't decode torn samples.
            sniff_overruns += pending;
            sniff_gaps += pending;
            sniff_processed[index] = sniff_completed[index];
            sniff_reset_decoder();
            continue;
        }
        sniff_processed[index]++;
        logic_record_buffer(index);
        if (sniff_decode) {
            sniff_print_budget = SNIFF_EVENT_LIMIT;
            sniff_decode_buffer(index);
        }
    }
}
static void sniff_start(void) {
    if (sniff_active) return;
    sniff_blocks = sniff_words = sniff_rearms = sniff_gaps = sniff_overruns = 0;
    sniff_irq_late = sniff_events = sniff_suppressed = 0;
    sniff_completed[0] = sniff_completed[1] = 0;
    sniff_processed[0] = sniff_processed[1] = 0;
    sniff_last_sequence = 0;
    sniff_reset_decoder();
    gpio_init(SNIFF_SDA_PIN);
    gpio_init(SNIFF_SCL_PIN);
    gpio_set_dir(SNIFF_SDA_PIN, GPIO_IN);
    gpio_set_dir(SNIFF_SCL_PIN, GPIO_IN);
    gpio_disable_pulls(SNIFF_SDA_PIN);
    gpio_disable_pulls(SNIFF_SCL_PIN);
    pio_gpio_init(sniff_pio, SNIFF_SDA_PIN);
    pio_gpio_init(sniff_pio, SNIFF_SCL_PIN);
    sniff_sm = pio_claim_unused_sm(sniff_pio, true);
    sniff_offset = pio_add_program(sniff_pio, &i2c_sniff_program);
    pio_sm_config cfg = i2c_sniff_program_get_default_config(sniff_offset);
    sm_config_set_in_pins(&cfg, SNIFF_SDA_PIN);
    sm_config_set_in_shift(&cfg, false, true, 32);
    sm_config_set_fifo_join(&cfg, PIO_FIFO_JOIN_RX);
    sm_config_set_clkdiv(&cfg, (float)clock_get_hz(clk_sys) / (2.0f * (logic_armed ? LOGIC_SAMPLE_HZ : SNIFF_SAMPLE_HZ)));
    pio_sm_init(sniff_pio, sniff_sm, sniff_offset, &cfg);
    pio_sm_clear_fifos(sniff_pio, sniff_sm);
    sniff_dma[0] = dma_claim_unused_channel(true);
    sniff_dma[1] = dma_claim_unused_channel(true);
    for (unsigned i = 0; i < 2; ++i) {
        dma_channel_config dc = dma_channel_get_default_config(sniff_dma[i]);
        channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
        channel_config_set_read_increment(&dc, false);
        channel_config_set_write_increment(&dc, true);
        channel_config_set_dreq(&dc, pio_get_dreq(sniff_pio, sniff_sm, false));
        channel_config_set_chain_to(&dc, sniff_dma[i ^ 1u]);
        dma_channel_configure(sniff_dma[i], &dc, sniff_buffers[i],
                              &sniff_pio->rxf[sniff_sm], SNIFF_BLOCK_WORDS, false);
        dma_channel_set_irq0_enabled(sniff_dma[i], true);
    }
    irq_set_exclusive_handler(DMA_IRQ_0, sniff_dma_irq);
    irq_set_enabled(DMA_IRQ_0, true);
    sniff_active = true;
    dma_channel_start(sniff_dma[0]);
    pio_sm_set_enabled(sniff_pio, sniff_sm, true);
}
static void sniff_stop(void) {
    if (!sniff_active) return;
    pio_sm_set_enabled(sniff_pio, sniff_sm, false);
    sniff_active = false;
    irq_set_enabled(DMA_IRQ_0, false);
    for (unsigned i = 0; i < 2; ++i) {
        dma_channel_set_irq0_enabled(sniff_dma[i], false);
        dma_channel_abort(sniff_dma[i]);
        dma_hw->ints0 = 1u << sniff_dma[i];
        dma_channel_unclaim(sniff_dma[i]);
        sniff_dma[i] = -1;
    }
    irq_remove_handler(DMA_IRQ_0, sniff_dma_irq);
    pio_sm_unclaim(sniff_pio, sniff_sm);
    pio_remove_program(sniff_pio, &i2c_sniff_program, sniff_offset);
    printf("OK SNIFF=STOPPED BLOCKS=%lu WORDS=%lu GAPS=%lu OVERRUNS=%lu IRQ_LATE=%lu EVENTS=%lu SUPPRESSED=%lu\n",
           (unsigned long)sniff_blocks, (unsigned long)sniff_words,
           (unsigned long)sniff_gaps, (unsigned long)sniff_overruns,
           (unsigned long)sniff_irq_late, (unsigned long)sniff_events,
           (unsigned long)sniff_suppressed);
}
static void sniff_drain(void) { sniff_update(); }

static void host_rx(uint8_t addr, const uint8_t *data, size_t count) {
    printf("HOST_RX: ADDR=0x%02X DATA=", addr);
    for (size_t i = 0; i < count; ++i) printf("%s%02X", i ? " " : "", data[i]);
    putchar('\n');
}

static void selftest(uint32_t iterations, uint32_t delay_ms) {

    uint8_t tx[17], rx[16], reg = 0x20;

    uint32_t passed = 0, failed = 0;

    uint64_t start = time_us_64();

    printf("SELFTEST START I2C0=MASTER I2C1=SLAVE ADDR=0x%02X SPEED=%lu ITER=%lu\n",

        test_slave_addr, (unsigned long)bus_hz, (unsigned long)iterations);

    for (uint32_t n=0; n<iterations; ++n) {

        tx[0] = reg;

        for (unsigned j=0; j<16; ++j) tx[j+1] = (uint8_t)(n*17u+j*29u+3u);

        int w = i2c_write_timeout_us(i2c0, test_slave_addr, tx, sizeof(tx), false, 100000);

        // A STOP ends the register write. Re-select the register and issue repeated START.

        int p = (w == (int)sizeof(tx)) ?

            i2c_write_timeout_us(i2c0, test_slave_addr, &reg, 1, true, 100000) : w;

        int r = (p == 1) ?

            i2c_read_timeout_us(i2c0, test_slave_addr, rx, sizeof(rx), false, 100000) : p;

        bool equal = (w == (int)sizeof(tx) && p == 1 && r == (int)sizeof(rx) &&

                      memcmp(rx, &tx[1], sizeof(rx)) == 0);

        if (equal) ++passed;

        else {

            ++failed;

            printf("SELFTEST FAIL N=%lu WRITE=%d POINTER=%d READ=%d MATCH=%s\n",

                (unsigned long)(n+1), w, p, r,

                r == (int)sizeof(rx) ? (memcmp(rx, &tx[1], sizeof(rx)) ? "NO" : "YES") : "N/A");

        }

        if ((n+1)%100 == 0 || n+1 == iterations)

            printf("SELFTEST PROGRESS %lu PASS=%lu FAIL=%lu\n", (unsigned long)(n+1),

                (unsigned long)passed, (unsigned long)failed);

        // Drain sniffer events during SELFTEST, not just in the USB command loop.
        // The GPIO IRQ producer remains active while I2C0/I2C1 transact.
        if (sniff_active) sniff_drain();
        if (delay_ms && n+1 < iterations) {
            for (uint32_t ms=0; ms<delay_ms; ++ms) {
                sleep_ms(1);
                if (sniff_active) sniff_drain();
            }
        }

    }

    if (sniff_active) sniff_drain();
    printf("SELFTEST DONE PASS=%lu FAIL=%lu ELAPSED_MS=%llu\n",

        (unsigned long)passed, (unsigned long)failed,

        (unsigned long long)((time_us_64()-start)/1000));

    puts(failed ? "ERR SELFTEST_FAILED" : "OK");

}



static bool parse_number(const char *s, uint32_t *out) {

    if (!s || !*s || *s == '-') return false;

    char *end;

    unsigned long v = strtoul(s, &end, 0);

    if (end == s || *end != '\0') return false;

    *out = (uint32_t)v;

    return true;

}

static char *token(char **ctx) {

    char *s = *ctx;

    while (*s == ' ' || *s == '\t' || *s == ',') ++s;

    if (!*s) { *ctx = s; return NULL; }

    char *start = s;

    while (*s && *s != ' ' && *s != '\t' && *s != ',') ++s;

    if (*s) *s++ = 0;

    *ctx = s;

    return start;

}

static bool get_num(char **ctx, uint32_t *n) { return parse_number(token(ctx), n); }

static bool address_ok(uint32_t a) { return a >= 0x08 && a <= 0x77; }

static void report_result(int r, size_t expected) {

    if (r == PICO_ERROR_GENERIC) puts("ERR NACK_OR_BUS");

    else if (r == PICO_ERROR_TIMEOUT) puts("ERR TIMEOUT");

    else if (r < 0) printf("ERR I2C %d\n", r);

    else if ((size_t)r != expected) printf("ERR SHORT %d/%u\n", r, (unsigned)expected);

    else puts("OK");

}

static bool soh_led_state = false;
static bool soh_led_ready = false;
static uint64_t soh_led_next_ms = 0;

static void soh_led_update(void) {
    uint64_t now = to_ms_since_boot(get_absolute_time());
    if (soh_led_ready && now >= soh_led_next_ms) {
        soh_led_state = !soh_led_state;
        cyw43_arch_gpio_put(SOH_LED_PIN, soh_led_state);
        soh_led_next_ms = now + 500;
    }
}


// R18 experimental: one PIO IN per sysclk cycle, three parallel inputs:
// GP20 MOSI (bit0), GP21 MISO (bit1), GP22 SCK (bit2).
// PIO waits for GP26 CS low before entering its single-cycle sampling loop.
// DMA stores ten 3-bit samples per 32-bit word (autopush at 30 bits).
// CPU reconstructs edges after CS rises. No per-clock IRQs.
#define SPI_SNIFF_RAW_WORDS 16384u
static volatile bool spi_sniff_active;

static volatile uint32_t spi_sniff_frames, spi_sniff_partial, spi_sniff_overruns;
static volatile uint32_t spi_sniff_cs_fall, spi_sniff_cs_rise;
static volatile uint32_t spi_sniff_words_last, spi_sniff_suppressed;
static volatile uint32_t spi_sniff_full_frames, spi_sniff_decoded_bytes_last;
static volatile bool spi_sniff_ready, spi_sniff_in_frame;
static volatile uint32_t spi_sniff_ready_words;
// R21: Snapshot DMA/PIO state at CS rise, before stopping the capture.
static volatile uint32_t spi_sniff_dma_remaining_last;
static volatile uint32_t spi_sniff_fifo_level_last;
static volatile uint32_t spi_sniff_fifo_overflow_last;
static volatile uint32_t spi_sniff_dma_busy_last;
static volatile uint32_t spi_sniff_raw_words_at_rise_last;
static uint32_t spi_sniff_buffer[SPI_SNIFF_RAW_WORDS];
static PIO spi_sniff_pio = pio1;
static uint spi_sniff_sm, spi_sniff_offset;
static int spi_sniff_dma = -1;
static uint spi_sniff_program_mode;
static uint16_t spi_sniff_raw_instructions[2];
static const struct pio_program spi_sniff_raw_program = {
    .instructions = spi_sniff_raw_instructions, .length = 2, .origin = -1,
};
static void spi_sniff_rearm(void) {
    pio_sm_set_enabled(spi_sniff_pio, spi_sniff_sm, false);
    dma_channel_abort(spi_sniff_dma);
    pio_sm_restart(spi_sniff_pio, spi_sniff_sm);
    pio_sm_clear_fifos(spi_sniff_pio, spi_sniff_sm);
    pio_sm_exec(spi_sniff_pio, spi_sniff_sm, pio_encode_jmp(spi_sniff_offset));
    dma_channel_set_write_addr(spi_sniff_dma, spi_sniff_buffer, false);
    dma_channel_set_trans_count(spi_sniff_dma, SPI_SNIFF_RAW_WORDS, false);
    dma_channel_start(spi_sniff_dma);
    pio_sm_set_enabled(spi_sniff_pio, spi_sniff_sm, true);
}
static void spi_sniff_gpio_irq(uint gpio, uint32_t events) {
    if (!spi_sniff_active || gpio != 26) return;
    if (events & GPIO_IRQ_EDGE_FALL) {
        ++spi_sniff_cs_fall;
        spi_sla_frame_start_us = time_us_64();
        spi_sniff_in_frame = true;
    }
    if (events & GPIO_IRQ_EDGE_RISE) {
        ++spi_sniff_cs_rise;
        if (!spi_sniff_in_frame) return;
        spi_sniff_in_frame = false;
        // Snapshot the hardware immediately at CS rise. This distinguishes
        // early capture termination from PIO RX FIFO backpressure at 50 MHz.
        spi_sniff_dma_remaining_last = dma_channel_hw_addr(spi_sniff_dma)->transfer_count;
        spi_sniff_raw_words_at_rise_last = SPI_SNIFF_RAW_WORDS - spi_sniff_dma_remaining_last;
        spi_sniff_fifo_level_last = pio_sm_get_rx_fifo_level(spi_sniff_pio, spi_sniff_sm);
        spi_sniff_fifo_overflow_last = (spi_sniff_pio->fdebug >> (PIO_FDEBUG_RXSTALL_LSB + spi_sniff_sm)) & 1u;
        spi_sniff_dma_busy_last = dma_channel_is_busy(spi_sniff_dma) ? 1u : 0u;
        // Allow pending complete autopush words to reach RAM after CS rises.
        busy_wait_us_32(2);
        pio_sm_set_enabled(spi_sniff_pio, spi_sniff_sm, false);
        dma_channel_abort(spi_sniff_dma);
        uintptr_t end = (uintptr_t)dma_channel_hw_addr(spi_sniff_dma)->write_addr;
        uintptr_t begin = (uintptr_t)spi_sniff_buffer;
        uint32_t n = 0;
        if (end >= begin && end <= begin + sizeof(spi_sniff_buffer))
            n = (uint32_t)((end-begin)/sizeof(spi_sniff_buffer[0]));
        spi_sniff_words_last = n;
        ++spi_sniff_frames;
        if (spi_sniff_ready) ++spi_sniff_overruns;
        else {
            spi_sniff_ready_words = n;
            spi_sla_ready_start_us = spi_sla_frame_start_us;
            spi_sla_ready_end_us = time_us_64();
            __compiler_memory_barrier();
            spi_sniff_ready = true;
        }
    }
}
static void spi_sniff_update(void) {
    if (!spi_sniff_ready) return;
    uint32_t words = spi_sniff_ready_words;
    // Reconstruct bytes from SCK transitions. In modes 0/3 sample on
    // rising SCK; in modes 1/2 sample on falling SCK.
    uint8_t mosi[128], miso[128];
    uint32_t bits=0;
    uint8_t m=0, r=0;
    bool prev_clock = (spi_sniff_program_mode & 2u) != 0;
    bool sample_rising = (spi_sniff_program_mode == 0 || spi_sniff_program_mode == 3);
    for (uint32_t w=0; w<words && bits<1024; ++w) {
        uint32_t raw=spi_sniff_buffer[w];
        // PIO shift-left autopush 30: earliest of 10 samples at bits 29..27.
        for (int k=9;k>=0 && bits<1024;--k) {
            uint32_t pins=(raw>>(k*3))&7u;
            bool clk=(pins&4u)!=0;
            bool edge=sample_rising ? (!prev_clock && clk) : (prev_clock && !clk);
            prev_clock=clk;
            if (edge) {
                m=(uint8_t)((m<<1)|(pins&1u));
                r=(uint8_t)((r<<1)|((pins>>1)&1u));
                ++bits;
                if (!(bits&7u)) {
                    mosi[(bits/8u)-1u]=m;
                    miso[(bits/8u)-1u]=r;
                    m=r=0;
                }
            }
        }
    }
    uint32_t bytes=bits/8u;
    spi_sniff_decoded_bytes_last=bytes;
    spi_sniff_partial=bits&7u;
    bool raw_full=(words>=SPI_SNIFF_RAW_WORDS);
    if(raw_full) ++spi_sniff_full_frames;
    if (spi_sla_stream) {
        // ASCII wire protocol v1. Each RAW line contains 10 three-bit samples
        // per 32-bit word, earliest sample in bits 29:27.
        // Do not rearm until the complete frame has been emitted.
        uint32_t packet = ++spi_sla_packet_id;
        printf("SLA_BEGIN ID=%lu PACKET=%lu START_US=%llu END_US=%llu MODE=%lu WORDS=%lu SAMPLE_HZ=%lu PARTIAL_BITS=%lu TRUNCATED=%u\n",
            (unsigned long)spi_sla_capture_id, (unsigned long)packet,
            (unsigned long long)spi_sla_ready_start_us,
            (unsigned long long)spi_sla_ready_end_us,
            (unsigned long)spi_sniff_program_mode, (unsigned long)words,
            (unsigned long)clock_get_hz(clk_sys), (unsigned long)spi_sniff_partial,
            (unsigned)(raw_full ? 1 : 0));
        for (uint32_t i = 0; i < words; i += 16u) {
            uint32_t count = words - i;
            if (count > 16u) count = 16u;
            printf("SLA_RAW %lu", (unsigned long)i);
            for (uint32_t j = 0; j < count; ++j)
                printf(" %08lX", (unsigned long)spi_sniff_buffer[i + j]);
            puts("");
        }
        printf("SLA_END ID=%lu PACKET=%lu WORDS=%lu\n",
            (unsigned long)spi_sla_capture_id, (unsigned long)packet,
            (unsigned long)words);
    } else ++spi_sniff_suppressed;
    spi_sniff_ready=false;
    if(spi_sniff_active && !spi_sniff_in_frame && gpio_get(26)) spi_sniff_rearm();
}
static void spi_sniff_start(void) {
    if(spi_sniff_active) return;
    spi_trace_off();
    spi_sniff_frames=spi_sniff_partial=spi_sniff_overruns=0;
    spi_sniff_cs_fall=spi_sniff_cs_rise=spi_sniff_words_last=0;
    spi_sniff_suppressed=0;
    spi_sniff_full_frames=spi_sniff_decoded_bytes_last=0;
    spi_sniff_dma_remaining_last=spi_sniff_fifo_level_last=0;
    spi_sniff_fifo_overflow_last=spi_sniff_dma_busy_last=0;
    spi_sniff_raw_words_at_rise_last=0;
    spi_sniff_ready=spi_sniff_in_frame=false;
    spi_sniff_program_mode=spi_mode;
    spi_sniff_raw_instructions[0]=pio_encode_wait_gpio(false,26);
    spi_sniff_raw_instructions[1]=pio_encode_in(pio_pins,3);
    for(uint pin=20;pin<=22;++pin) {
        gpio_init(pin);gpio_disable_pulls(pin);pio_gpio_init(spi_sniff_pio,pin);
    }
    gpio_init(26);gpio_set_dir(26,GPIO_IN);gpio_disable_pulls(26);
    spi_sniff_sm=pio_claim_unused_sm(spi_sniff_pio,true);
    spi_sniff_offset=pio_add_program(spi_sniff_pio,&spi_sniff_raw_program);
    pio_sm_config cfg=pio_get_default_sm_config();
    sm_config_set_wrap(&cfg,spi_sniff_offset+1,spi_sniff_offset+1);
    sm_config_set_in_pins(&cfg,20);
    sm_config_set_in_shift(&cfg,false,true,30);
    sm_config_set_fifo_join(&cfg,PIO_FIFO_JOIN_RX);
    sm_config_set_clkdiv(&cfg,1.0f);
    pio_sm_init(spi_sniff_pio,spi_sniff_sm,spi_sniff_offset,&cfg);
    spi_sniff_dma=dma_claim_unused_channel(true);
    dma_channel_config dc=dma_channel_get_default_config(spi_sniff_dma);
    channel_config_set_transfer_data_size(&dc,DMA_SIZE_32);
    channel_config_set_read_increment(&dc,false);
    channel_config_set_write_increment(&dc,true);
    channel_config_set_dreq(&dc,pio_get_dreq(spi_sniff_pio,spi_sniff_sm,false));
    dma_channel_configure(spi_sniff_dma,&dc,spi_sniff_buffer,
                          &spi_sniff_pio->rxf[spi_sniff_sm],SPI_SNIFF_RAW_WORDS,false);
    spi_sniff_active=true;
    spi_sniff_rearm();
    gpio_set_irq_enabled_with_callback(26,GPIO_IRQ_EDGE_RISE|GPIO_IRQ_EDGE_FALL,true,spi_sniff_gpio_irq);
}
static void spi_sniff_stop(void) {
    if(!spi_sniff_active)return;
    spi_sniff_active=false;
    gpio_set_irq_enabled(26,GPIO_IRQ_EDGE_RISE|GPIO_IRQ_EDGE_FALL,false);
    pio_sm_set_enabled(spi_sniff_pio,spi_sniff_sm,false);
    dma_channel_abort(spi_sniff_dma);
    dma_channel_unclaim(spi_sniff_dma);spi_sniff_dma=-1;
    pio_sm_unclaim(spi_sniff_pio,spi_sniff_sm);
    pio_remove_program(spi_sniff_pio,&spi_sniff_raw_program,spi_sniff_offset);
    spi_sniff_update();
    printf("OK SPI CAPTURE ENGINE STOPPED FRAMES=%lu CS_FALL=%lu CS_RISE=%lu RAW_WORDS_LAST=%lu OVERRUNS=%lu\n",
        (unsigned long)spi_sniff_frames,(unsigned long)spi_sniff_cs_fall,
        (unsigned long)spi_sniff_cs_rise,(unsigned long)spi_sniff_words_last,
        (unsigned long)spi_sniff_overruns);
}
static void spi_command(char **ctx) {
    char *op = token(ctx);
    if (!op) { puts("ERR SPI expected STATUS|MODE|SPEED|SLAVE|CS|TRACE|XFER"); return; }
    for (char *p=op; *p; ++p) *p=(char)toupper((unsigned char)*p);
    if (!strcmp(op,"STATUS")) {
        printf("OK SPI0=MASTER SPI1=SLAVE MODE=%lu BAUD=%lu SLAVE_TX_LEN=%lu SLAVE_RX_COUNT=%lu SNIFFER=%s\n",
            (unsigned long)spi_mode,(unsigned long)spi_baud,(unsigned long)spi_fixed_len,(unsigned long)spi_slave_rx_count,spi_sniff_active ? "RUNNING" : "STOPPED");
        printf("SPI_CAPTURE ID=%lu STATE=%s PACKETS=%lu OVERRUNS=%lu PARTIAL_BITS=%lu RAW_WORDS_LAST=%lu\n",
            (unsigned long)spi_sla_capture_id,spi_sniff_active?"STREAMING":"STOPPED",
            (unsigned long)spi_sla_packet_id,(unsigned long)spi_sniff_overruns,
            (unsigned long)spi_sniff_partial,(unsigned long)spi_sniff_words_last);
        spi_hw_t *shw = spi_get_hw(spi1);
        printf("SPI_HW: SSE=%u MS=%u SR=0x%02lX DMA_TX=%lu DMA_RX=%lu DMA_XFERS=%lu DMA_FAIL=%lu RX_OVERRUN=%lu\n",
            !!(shw->cr1 & SPI_SSPCR1_SSE_BITS),
            !!(shw->cr1 & SPI_SSPCR1_MS_BITS),
            (unsigned long)shw->sr,
            (unsigned long)spi_dma_tx_done, (unsigned long)spi_dma_rx_done,
            (unsigned long)spi_dma_transactions, (unsigned long)spi_dma_failures,
            (unsigned long)spi_slave_overruns);
    } else if (!strcmp(op,"TRACE")) {
        char *sub = token(ctx);
        if (!sub || token(ctx)) { puts("ERR SPI TRACE ARM|STATUS|DATA|OFF"); return; }
        for (char *p=sub; *p; ++p) *p=(char)toupper((unsigned char)*p);
        if (!strcmp(sub,"ARM")) { spi_trace_arm(); puts("OK SPI_TRACE=ARMED"); }
        else if (!strcmp(sub,"OFF")) { spi_trace_off(); puts("OK SPI_TRACE=OFF"); }
        else if (!strcmp(sub,"STATUS")) spi_trace_status();
        else if (!strcmp(sub,"DATA")) spi_trace_data();
        else puts("ERR SPI TRACE ARM|STATUS|DATA|OFF");
    } else if (!strcmp(op,"MODE?")) {
        if (token(ctx)) { puts("ERR ARGS"); return; }
        printf("OK SPI MODE=%lu\n", (unsigned long)spi_mode);
    } else if (!strcmp(op,"SPEED?")) {
        if (token(ctx)) { puts("ERR ARGS"); return; }
        printf("OK SPI SPEED=%lu\n", (unsigned long)spi_baud);
    } else if (!strcmp(op,"MODE")) {
        uint32_t mode;
        if (!get_num(ctx,&mode) || mode>3 || token(ctx)) { puts("ERR SPI MODE 0..3"); return; }
        spi_set_mode(mode);
        printf("OK SPI MODE=%lu\n",(unsigned long)mode);
        if (!(mode & 1u)) puts("WARN SPI CPHA=0: continuous CS across multiple 8-bit words may fail with RP2040 SPI1 slave; use SPI CS PULSE XFER or MODE 1/3.");
    } else if (!strcmp(op,"SPEED")) {
        uint32_t hz;
        // Maximum validated continuous 64-byte MOSI sniff speed: 33.333333 MHz.
        if (!get_num(ctx,&hz) || hz<1000 || hz>33333333u || token(ctx)) {
            puts("ERR SPI SPEED 1000..33333333 (33.333 MHz maximum validated)");
            return;
        }
        // The SPI SDK rounds requested rates downward. At 200 MHz clk_peri,
        // asking for 33333333 can select 25 MHz due to integer truncation.
        // Request a value inside the divider-6 bucket to obtain 33.333333 MHz.
        uint32_t requested_hz = (hz == 33333333u) ? 40000000u : hz;
        spi_baud = spi_set_baudrate(spi0, requested_hz);
        printf("OK SPI SPEED=%lu PERI_CLOCK=%lu\n",
               (unsigned long)spi_baud,(unsigned long)clock_get_hz(clk_peri));
        if (spi_baud > 10000000u) puts("WARN SPI1 slave loopback exceeds approximate RP2040 slave clock specification; sniffer and slave data not guaranteed");
    } else if (!strcmp(op,"SLAVE")) {
        char *sub=token(ctx);
        if (!sub) { puts("ERR SPI SLAVE expected TX|TX?|RX?|STATUS"); return; }
        for(char *p=sub;*p;++p)*p=(char)toupper((unsigned char)*p);
        if (!strcmp(sub,"TX")) {
            uint8_t bytes[SPI_FIXED_MAX]; uint32_t n=0,v;
            while (n<SPI_FIXED_MAX) {
                char *t=token(ctx); if (!t) break;
                if (!parse_number(t,&v) || v>255) { puts("ERR SPI TX BYTE"); return; }
                bytes[n++]=(uint8_t)v;
            }
            if (!n || token(ctx)) { puts("ERR SPI SLAVE TX requires 1..128 bytes"); return; }
            memcpy(spi_fixed,bytes,n); spi_fixed_len=n;
            spi_slave_prime(true);
            printf("OK SPI SLAVE TX LEN=%lu\n",(unsigned long)n);
        } else if (!strcmp(sub,"TX?")) {
            printf("SPI_SLAVE_TX:");
            for(uint32_t i=0;i<spi_fixed_len;++i) printf(" %02X",spi_fixed[i]);
            puts("");
        } else if (!strcmp(sub,"RX?")) {
            printf("SPI_SLAVE_RX COUNT=%lu LAST=",(unsigned long)spi_slave_rx_count);
            uint32_t n=spi_slave_rx_count<SPI_FIXED_MAX?spi_slave_rx_count:SPI_FIXED_MAX;
            uint32_t start=spi_slave_rx_count-n;
            for(uint32_t i=0;i<n;++i) printf(" %02X",spi_slave_rx[(start+i)%SPI_FIXED_MAX]);
            puts("");
        } else if (!strcmp(sub,"STATUS")) {
            printf("OK SPI SLAVE TX_LEN=%lu RX_COUNT=%lu\n",(unsigned long)spi_fixed_len,(unsigned long)spi_slave_rx_count);
        } else puts("ERR SPI SLAVE expected TX|TX?|RX?|STATUS");
    } else if (!strcmp(op,"CS")) {
        char *sub=token(ctx);
        if (!sub) { puts("ERR SPI CS PULSE XFER <1..8 bytes>"); return; }
        for(char *q=sub;*q;++q)*q=(char)toupper((unsigned char)*q);
        if (strcmp(sub,"PULSE")) { puts("ERR SPI CS PULSE XFER <1..8 bytes>"); return; }
        sub=token(ctx);
        if (!sub) { puts("ERR SPI CS PULSE XFER <1..8 bytes>"); return; }
        for(char *q=sub;*q;++q)*q=(char)toupper((unsigned char)*q);
        if (strcmp(sub,"XFER")) { puts("ERR SPI CS PULSE XFER <1..8 bytes>"); return; }
        uint8_t tx[8]; uint32_t n=0,v; char *t;
        while ((t=token(ctx)) != NULL) {
            if (n>=8 || !parse_number(t,&v) || v>255) {
                puts("ERR SPI CS PULSE XFER requires 1..8 bytes"); return;
            }
            tx[n++]=(uint8_t)v;
        }
        if (!n) { puts("ERR SPI CS PULSE XFER requires 1..8 bytes"); return; }
        spi_cs_pulse_test(tx,n);
    } else if (!strcmp(op,"XFER")) {
        uint8_t tx[SPI_FIXED_MAX],rx[SPI_FIXED_MAX]; uint32_t n=0,v;
        while(n<SPI_FIXED_MAX) {
            char *t=token(ctx); if(!t)break;
            if(!parse_number(t,&v)||v>255){puts("ERR SPI XFER BYTE");return;}
            tx[n++]=(uint8_t)v;
        }
        if(!n || token(ctx)){puts("ERR SPI XFER requires 1..128 bytes");return;}
        if (n > 1 && !(spi_mode & 1u)) puts("WARN SPI CONTINUOUS_CS CPHA=0: RP2040 SPI1 slave expects CS to pulse between words; use MODE 1/3 or SPI CS PULSE XFER.");
        spi_dma_transfer(tx, rx, n);
    } else puts("ERR SPI expected STATUS|MODE|SPEED|SLAVE|CS|TRACE|XFER");
}

static void handle(char *line) {

    char *ctx = line;

    char *cmd = token(&ctx);

    if (!cmd) return;

    for (char *p=cmd; *p; ++p) *p=(char)toupper((unsigned char)*p);

    if (!strcmp(cmd,"SPI")) {
        // Probe a separate buffer: token() replaces delimiters with NUL.
        // Never tokenize ctx before handing it to spi_command(), or commands
        // such as SPI XFER and SPI SLAVE TX lose their arguments.
        char probe_buffer[64];
        snprintf(probe_buffer, sizeof(probe_buffer), "%s", ctx);
        char *probe = probe_buffer;
        char *action = token(&probe);
        if (action) for(char *p=action;*p;++p)*p=(char)toupper((unsigned char)*p);
        if (action && !strcmp(action,"NEW")) {
            char *next=token(&probe);
            if (next) for(char *p=next;*p;++p)*p=(char)toupper((unsigned char)*p);
            if (!next || strcmp(next,"CAPTURE") || token(&probe)) {
                puts("ERR SPI expected NEW CAPTURE"); return;
            }
            if (spi_sniff_active) spi_sniff_stop();
            spi_sla_stream=true;
            ++spi_sla_capture_id;
            spi_sla_packet_id=0;
            spi_sniff_start();
            printf("OK SPI CAPTURE STARTED ID=%lu MODE=STREAM SAMPLE_HZ=%lu PINS=20,21,22,26\n",
                (unsigned long)spi_sla_capture_id,(unsigned long)clock_get_hz(clk_sys));
        } else if (action && !strcmp(action,"STOP") && !token(&probe)) {
            if (spi_sla_stream && spi_sniff_active) spi_sniff_stop();
            spi_sla_stream=false;
            printf("OK SPI CAPTURE STOPPED ID=%lu PACKETS=%lu OVERRUNS=%lu\n",
                (unsigned long)spi_sla_capture_id,(unsigned long)spi_sla_packet_id,
                (unsigned long)spi_sniff_overruns);
        } else spi_command(&ctx);
    } else if (!strcmp(cmd,"HELP")) {

        puts("GENERAL: HELP, INFO, LEDSTATUS, LEDTEST");
        puts("I2C:");
        puts("  SPEED <100000|400000>");
        puts("  SLAVEADDR <0x08..0x77> | SLAVEADDR?");
        puts("  SCAN | SELFTEST <iterations> <delay_ms>");
        puts("  LOOP <addr> <reg> <read_count> <iterations> <delay_ms>");
        puts("  WRITE <addr> <byte>... | READ <addr> <count>");
        puts("  WRITEREAD <addr> <txcount> <txbytes...> <rxcount>");
        puts("I2C LOGIC CAPTURE: I2C NEW CAPTURE | I2C STOP | I2C STATUS");
        puts("I2C SNIFFER:");
        puts("  SNIFF START | SNIFF STOP | SNIFF STATUS");
        puts("  SNIFF DECODE ON|OFF (START auto-enables decode)");
        puts("SPI:");
        puts("  SPI STATUS | SPI MODE <0..3> | SPI MODE? | SPI SPEED <1000..33333333> | SPI SPEED?");
        puts("  SPI maximum validated sniffer speed: 33.333 MHz; actual clock is reported");
        puts("  Default SPI MODE 1 (CPHA=1) for continuous-CS multi-byte transfers");
        puts("  SPI SLAVE TX <byte>... | SPI SLAVE TX? | SPI SLAVE RX?");
        puts("  SPI XFER <byte>...");
        puts("  SPI CS PULSE XFER <1..8 bytes> (for CPHA=0 multi-byte slave transfers)");
        puts("  SPI TRACE ARM | SPI TRACE STATUS | SPI TRACE DATA | SPI TRACE OFF");
        puts("SPI LOGIC CAPTURE: SPI NEW CAPTURE | SPI STOP | SPI STATUS");
        puts("  SPI XFER operates independently of capture and the SLA GUI");
        puts("Numbers: decimal or 0x-prefixed hex. I2C address is 7-bit. Max 128 data bytes.");

    } else if (!strcmp(cmd,"LEDSTATUS")) {
        printf("OK FW=%s LED=CYW43 TIMER=%s STATE=%s\r\n",
               FW_VERSION, soh_led_ready ? "RUNNING" : "FAILED",
               soh_led_state ? "ON" : "OFF");
    } else if (!strcmp(cmd,"LEDTEST")) {
        for (int i = 0; i < 5; ++i) {
            cyw43_arch_gpio_put(SOH_LED_PIN, true);
            sleep_ms(150);
            cyw43_arch_gpio_put(SOH_LED_PIN, false);
            sleep_ms(150);
        }
        soh_led_state = false;
        soh_led_next_ms = to_ms_since_boot(get_absolute_time()) + 500;
        printf("OK LEDTEST FW=%s\r\n", FW_VERSION);
    } else if (!strcmp(cmd,"INFO")) {
        printf("OK PICO2W FW=%s LED=CYW43 STATUS=%s\r\n",
               FW_VERSION, soh_led_ready ? "RUNNING" : "FAILED");
        printf("I2C0=MASTER SDA=GP4 SCL=GP5 SPEED=%lu\r\n", (unsigned long)bus_hz);
        printf("I2C1=SLAVE SDA=GP6 SCL=GP7 ADDR=0x%02X\r\n", test_slave_addr);
        printf("I2C_SNIFF SDA=GP8 SCL=GP9 MODE=PIO_DMA_DECODE\r\n");
        printf("SPI0=MASTER MISO=GP16 CS=GP17 SCK=GP18 MOSI=GP19 SPEED=%lu MODE=%lu\r\n", (unsigned long)spi_baud, (unsigned long)spi_mode);
        printf("SPI1=SLAVE MOSI=GP12 CS=GP13 SCK=GP14 MISO=GP15 MODE=%lu\r\n", (unsigned long)spi_mode);
        printf("SPI_CAPTURE=PIO_DMA MOSI=GP20 MISO=GP21 SCK=GP22 CS=GP26\r\n");
    } else if (!strcmp(cmd,"I2C")) {
        char *action = token(&ctx);
        if (!action) { puts("ERR I2C expected NEW CAPTURE|STOP|STATUS"); return; }
        for (char *q=action; *q; ++q) *q=(char)toupper((unsigned char)*q);
        if (!strcmp(action,"NEW")) {
            char *next = token(&ctx);
            if (!next) { puts("ERR I2C expected NEW CAPTURE"); return; }
            for (char *q=next; *q; ++q) *q=(char)toupper((unsigned char)*q);
            if (strcmp(next,"CAPTURE") || token(&ctx)) {
                puts("ERR I2C expected NEW CAPTURE"); return;
            }
            if (sniff_active) sniff_stop();
            logic_previous = 3u; logic_ready = logic_triggered = false;
            logic_ticks = logic_edge_count = logic_stops = 0;
            logic_packet_number = logic_dropped_packets = 0;
            logic_queue_head = logic_queue_tail = logic_queue_count = 0;
            logic_queue_high_water = logic_queue_full_drops = logic_edge_limit_drops = 0;
            logic_usb_sent = 0;
            logic_tx_index = 0; logic_tx_phase = 0;
            logic_packet_open = logic_packet_discard = logic_overflow = false;
            logic_armed = true; ++logic_capture_id;
            sniff_decode = false;
            sniff_start();
            printf("OK I2C CAPTURE STARTED ID=%lu MODE=STREAM SAMPLE_HZ=%lu QUEUE=%lu\n",
                   (unsigned long)logic_capture_id, (unsigned long)LOGIC_SAMPLE_HZ,
                   (unsigned long)LOGIC_PACKET_SLOTS);
        } else if (!strcmp(action,"STATUS") && !token(&ctx)) {
            printf("OK I2C ID=%lu STATE=%s TRIGGERED=%u PACKETS=%lu QUEUED=%lu DROPPED=%lu SAMPLE_HZ=%lu GAPS=%lu OVERRUNS=%lu STOPS=%lu\n",
                   (unsigned long)logic_capture_id, logic_armed ? "STREAMING" : "STOPPED",
                   logic_triggered ? 1u : 0u, (unsigned long)logic_packet_number,
                   (unsigned long)logic_queue_count, (unsigned long)logic_dropped_packets,
                   (unsigned long)LOGIC_SAMPLE_HZ, (unsigned long)sniff_gaps,
                   (unsigned long)sniff_overruns, (unsigned long)logic_stops);
        } else if (!strcmp(action,"STOP") && !token(&ctx)) {
            logic_armed = false;
            if (sniff_active) sniff_stop();
            printf("OK I2C CAPTURE STOPPED ID=%lu PACKETS=%lu DROPPED=%lu\n",
                   (unsigned long)logic_capture_id, (unsigned long)logic_packet_number,
                   (unsigned long)logic_dropped_packets);
        } else puts("ERR I2C expected NEW CAPTURE|STOP|STATUS");
    } else if (!strcmp(cmd,"SNIFF")) {
        char *arg = token(&ctx);
        if (!arg) { puts("ERR SNIFF expected START|STOP|STATUS|DECODE"); return; }
        for (char *q=arg; *q; ++q) *q=(char)toupper((unsigned char)*q);
        if (!strcmp(arg,"START")) {
            // Normal operation: START enables both capture and decoding.
            // Preserve DECODE OFF as a separate diagnostic command.
            if (!sniff_active) {
                sniff_decode = true;
                sniff_start();
            } else if (!sniff_decode) {
                sniff_decode = true;
                sniff_reset_decoder();
            }
            printf("OK SNIFF=RUNNING MODE=CHAINED_DMA DECODE=%s\n", sniff_decode ? "ON" : "OFF");
        } else if (!strcmp(arg,"STOP")) {
            if (sniff_active) sniff_stop();
            else puts("OK SNIFF=STOPPED");
        } else if (!strcmp(arg,"STATUS")) {
            sniff_update();
            printf("OK SNIFF=%s DMA=%s BLOCKS=%lu WORDS=%lu REARMS=%lu FIFO=%u DECODE=%s GAPS=%lu OVERRUNS=%lu IRQ_LATE=%lu EVENTS=%lu SUPPRESSED=%lu\n",
                   sniff_active ? "RUNNING" : "STOPPED",
                   sniff_active ? "RUNNING" : "OFF",
                   (unsigned long)sniff_blocks, (unsigned long)sniff_words,
                   (unsigned long)sniff_rearms,
                   sniff_active ? pio_sm_get_rx_fifo_level(sniff_pio, sniff_sm) : 0u,
                   sniff_decode ? "ON" : "OFF", (unsigned long)sniff_gaps,
                   (unsigned long)sniff_overruns, (unsigned long)sniff_irq_late,
                   (unsigned long)sniff_events, (unsigned long)sniff_suppressed);
        } else if (!strcmp(arg,"DECODE")) {
            char *mode = token(&ctx);
            if (!mode || token(&ctx)) { puts("ERR SNIFF DECODE expected ON|OFF"); return; }
            for (char *q=mode; *q; ++q) *q=(char)toupper((unsigned char)*q);
            if (!strcmp(mode,"ON")) sniff_decode = true;
            else if (!strcmp(mode,"OFF")) sniff_decode = false;
            else { puts("ERR SNIFF DECODE expected ON|OFF"); return; }
            sniff_reset_decoder();
            printf("OK SNIFF DECODE=%s\n", sniff_decode ? "ON" : "OFF");
        } else puts("ERR SNIFF expected START|STOP|STATUS|DECODE");
    } else if (!strcmp(cmd,"SPEED")) {

        uint32_t n;

        if (!get_num(&ctx,&n) || (n != 100000 && n != 400000) || token(&ctx)) { puts("ERR SPEED (100000 or 400000)"); return; }

        // RP2040 nominal baud excludes physical bus rise/fall overhead.
        // Experimental compensation for 400 kHz mode only. Verify SCL with
        // an oscilloscope; adjust or revert if the bus exceeds 400 kHz.
        const uint32_t requested_hz = n;
        const uint32_t programmed_hz = (n == 400000u) ? 450000u : n;
        bus_hz = i2c_set_baudrate(i2c0, programmed_hz);

        printf("OK SPEED TARGET=%lu SDK=%lu PERI=%lu COMPENSATED=%s\n",
               (unsigned long)requested_hz, (unsigned long)bus_hz,
               (unsigned long)clock_get_hz(clk_peri),
               programmed_hz != requested_hz ? "YES" : "NO");

    } else if (!strcmp(cmd,"SLAVEADDR?")) {
        if (token(&ctx)) { puts("ERR ARGS"); return; }
        printf("OK SLAVEADDR=0x%02X\n", test_slave_addr);
    } else if (!strcmp(cmd,"SLAVEADDR")) {
        uint32_t addr;
        if (!get_num(&ctx, &addr) || !address_ok(addr) || token(&ctx)) {
            puts("ERR SLAVEADDR expected 7-bit address 0x08..0x77");
            return;
        }
        target_disable();
        test_slave_addr = (uint8_t)addr;
        target_enable();
        printf("OK SLAVEADDR=0x%02X\n", test_slave_addr);
    } else if (!strcmp(cmd,"SCAN")) {

        if (token(&ctx)) { puts("ERR ARGS"); return; }

        uint8_t dummy;

        unsigned count=0;

        printf("FOUND");

        for (uint8_t a=0x08; a<=0x77; ++a) {

            // A one-byte read is a practical scan but may have side effects on some devices.

            if (i2c_read_blocking(i2c0,a,&dummy,1,false)==1) { printf(" 0x%02X",a); ++count; }
            if (sniff_active) sniff_drain();

        }

        printf("\nOK COUNT=%u\n",count);

    } else if (!strcmp(cmd,"SELFTEST")) {

        uint32_t iterations, delay_ms;

        if (!get_num(&ctx,&iterations) || iterations<1 || iterations>100000 ||

            !get_num(&ctx,&delay_ms) || delay_ms>60000 || token(&ctx)) {

            puts("ERR SELFTEST_ARGS expected: SELFTEST <iterations:1..100000> <delay_ms:0..60000>");

            return;

        }

        selftest(iterations, delay_ms);

    } else if (!strcmp(cmd,"LOOP")) {

        uint32_t addr, reg, read_count, iterations, delay_ms;

        uint8_t rx[MAX_DATA];

        if (!get_num(&ctx,&addr) || !address_ok(addr) ||

            !get_num(&ctx,&reg) || reg>255 ||

            !get_num(&ctx,&read_count) || read_count<1 || read_count>MAX_DATA ||

            !get_num(&ctx,&iterations) || iterations<1 || iterations>100000 ||

            !get_num(&ctx,&delay_ms) || delay_ms>60000 || token(&ctx)) {

            puts("ERR LOOP_ARGS expected: LOOP <addr> <reg> <read_count> <iterations:1..100000> <delay_ms:0..60000>");

            return;

        }

        unsigned ok=0, fail=0;

        uint8_t tx=(uint8_t)reg;

        uint64_t start=time_us_64();

        printf("LOOP START ADDR=0x%02lX REG=0x%02lX BYTES=%lu ITERATIONS=%lu DELAY_MS=%lu\n",

            (unsigned long)addr,(unsigned long)reg,(unsigned long)read_count,

            (unsigned long)iterations,(unsigned long)delay_ms);

        for (uint32_t i=0; i<iterations; ++i) {

            int w=i2c_write_timeout_us(i2c0,(uint8_t)addr,&tx,1,true,100000);

            int r=(w==1) ? i2c_read_timeout_us(i2c0,(uint8_t)addr,rx,read_count,false,100000) : w;

            if (w==1 && r==(int)read_count) {

                ++ok;

                if (i==0 || i+1==iterations || (i+1)%100==0) {

                    printf("LOOP PASS %lu\n", (unsigned long)(i+1));
                    host_rx((uint8_t)addr, rx, read_count);

                }

            } else {

                ++fail;

                printf("LOOP FAIL %lu WRITE=%d READ=%d\n",(unsigned long)(i+1),w,r);

            }

            if (sniff_active) sniff_drain();
            if (delay_ms && i+1<iterations) {
                for (uint32_t ms=0; ms<delay_ms; ++ms) {
                    sleep_ms(1);
                    if (sniff_active) sniff_drain();
                }
            }

        }

        printf("LOOP DONE PASS=%u FAIL=%u ELAPSED_MS=%llu\n",ok,fail,

            (unsigned long long)((time_us_64()-start)/1000));

        puts(fail ? "ERR LOOP_FAILED" : "OK");

    } else if (!strcmp(cmd,"WRITE")) {

        uint32_t a,v; uint8_t data[MAX_DATA]; size_t n=0;

        if (!get_num(&ctx,&a) || !address_ok(a)) { puts("ERR ADDRESS"); return; }

        char *s;

        while ((s=token(&ctx))) {

            if (n>=MAX_DATA || !parse_number(s,&v) || v>255) { puts("ERR BYTE_OR_LENGTH"); return; }

            data[n++]=(uint8_t)v;

        }

        if (!n) { puts("ERR EMPTY_WRITE"); return; }

        report_result(i2c_write_blocking(i2c0,(uint8_t)a,data,n,false),n);

    } else if (!strcmp(cmd,"READ")) {

        uint32_t a,n; uint8_t data[MAX_DATA];

        if (!get_num(&ctx,&a) || !address_ok(a) || !get_num(&ctx,&n) || n<1 || n>MAX_DATA || token(&ctx)) { puts("ERR ARGS"); return; }

        int r=i2c_read_blocking(i2c0,(uint8_t)a,data,n,false);

        if (r<0 || (uint32_t)r!=n) { report_result(r,n); return; }

        host_rx((uint8_t)a, data, n);
        puts("OK");

    } else if (!strcmp(cmd,"WRITEREAD")) {

        uint32_t a,txlen,rxlen,v; uint8_t tx[MAX_DATA],rx[MAX_DATA];

        if (!get_num(&ctx,&a) || !address_ok(a) || !get_num(&ctx,&txlen) || txlen<1 || txlen>MAX_DATA) { puts("ERR ARGS"); return; }

        for (uint32_t i=0;i<txlen;++i) {

            if (!get_num(&ctx,&v) || v>255) { puts("ERR TX_BYTES"); return; }

            tx[i]=(uint8_t)v;

        }

        if (!get_num(&ctx,&rxlen) || rxlen<1 || rxlen>MAX_DATA || token(&ctx)) { puts("ERR RX_LENGTH"); return; }

        // nostop=true preserves the bus for a repeated START on the following read.

        int r=i2c_write_blocking(i2c0,(uint8_t)a,tx,txlen,true);

        if (r<0 || (uint32_t)r!=txlen) { report_result(r,txlen); return; }

        r=i2c_read_blocking(i2c0,(uint8_t)a,rx,rxlen,false);

        if (r<0 || (uint32_t)r!=rxlen) { report_result(r,rxlen); return; }

        host_rx((uint8_t)a, rx, rxlen);
        puts("OK");

    } else puts("ERR UNKNOWN_COMMAND");

}

int main(void) {
#if SOH_EXPERIMENTAL_200MHZ
    // Set clk_sys to 200 MHz; clk_peri must be configured separately.
    // USB retains its separate 48 MHz clock source.
    if (!set_sys_clock_khz(200000, false)) {
        // Do not silently proceed if the requested PLL configuration fails.
        // The serial startup banner below reports the actual frequency.
    }
#endif
    // Pico 2 W onboard LED is controlled through the CYW43 wireless chip.
    // Initialize after stdio_init_all() below.

#if SOH_EXPERIMENTAL_200MHZ
    // On this board clk_peri remained at 48 MHz after changing clk_sys.
    // Explicitly source the peripheral clock from the current system clock.
    // This must happen before SPI and I2C initialization.
    const uint32_t sys_hz = clock_get_hz(clk_sys);
    clock_configure(clk_peri, 0,
                    CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
                    sys_hz, sys_hz);
#endif

    stdio_init_all();
    soh_led_ready = (cyw43_arch_init() == 0);
    if (soh_led_ready) {
        cyw43_arch_gpio_put(SOH_LED_PIN, false);
        soh_led_next_ms = to_ms_since_boot(get_absolute_time()) + 500;
    }

    // Wait briefly for a USB CDC terminal so the startup message is visible.
    // Continue after 5 seconds even if no terminal is connected.
    absolute_time_t deadline = make_timeout_time_ms(5000);
    while (!stdio_usb_connected() && !time_reached(deadline)) {
        sleep_ms(10);
    }
    printf("Pico 2 W RP2350 I2C Controller Ready\r\n");
    printf("FW=%s SOH LED CYW43: %s\r\n", FW_VERSION, soh_led_ready ? "1 Hz heartbeat" : "INIT ERROR");
    printf("CLOCK SYS=%lu Hz PERI=%lu Hz %s\r\n",
        (unsigned long)clock_get_hz(clk_sys),
        (unsigned long)clock_get_hz(clk_peri),
#if SOH_EXPERIMENTAL_200MHZ
        "EXPERIMENTAL_OVERCLOCK");
#else
        "STANDARD");
#endif
    printf("I2C0 HOST GP4/GP5 | I2C1 SLAVE GP6/GP7 | SNIFFER GP8/GP9\r\n");
    printf("Type HELP for commands\r\n");
    fflush(stdout);

    i2c_init(i2c0,bus_hz);

    gpio_set_function(SDA_PIN,GPIO_FUNC_I2C);

    gpio_set_function(SCL_PIN,GPIO_FUNC_I2C);

    gpio_pull_up(SDA_PIN);

    gpio_pull_up(SCL_PIN);

    // Keep I2C1 responding independently of SELFTEST.
    target_enable();
    spi_setup();
    // Do not auto-start high-rate capture before USB commands are available.
    // Start explicitly with SNIFF START; I2C host/slave remain usable.

    char line[LINE_SIZE]; size_t used=0;

    while (true) {
        soh_led_update();
        sniff_drain();
        logic_stream_step();
        spi_sniff_update();
        // R5.3: Do not idle for up to 1 ms on every pass. USB packet
        // transmission is serviced on each foreground iteration.
        int c=getchar_timeout_us(0);

        if (c==PICO_ERROR_TIMEOUT) continue;

        if (c=='\r' || c=='\n') {

            if (used) { line[used]=0; handle(line); used=0; }

        } else if (c>=32 && c<=126) {

            if (used<LINE_SIZE-1) line[used++]=(char)c;

            else { used=0; puts("ERR LINE_TOO_LONG"); }

        }

    }

}
