/*
 * main.c - cc1101_radio: CC1101 sub-GHz transceiver on the MSP-EXP430FR5994
 *
 * Drivers in drivers/ are a copy of driver_development's (snapshot of
 * 2026-11-22), plus spi.c from epaper_display, cc1101.c and
 * cc1101_settings.c.
 *
 * What this build does: packet TX/RX at 433.92 MHz, GFSK 1.2 kBaud,
 * 0 dBm, variable length with CRC. One image for both boards; the role
 * is picked at reset:
 *
 *   S1 (P5.6) not held -> TX: one packet a second, payload = 16-bit
 *                         sequence number + "hello 433". The MCU sleeps
 *                         in LPM3 while the packet goes out and wakes on
 *                         the falling edge of GDO0 (P1.3) = end of packet.
 *   S1 held at reset   -> RX: listens continuously, sleeps in LPM3 and
 *                         wakes on the GDO0 falling edge, then reads the
 *                         packet and prints it.
 *
 * Boot output (both roles):
 *   cc1101_radio: 433.92 MHz GFSK 1.2 kBaud, role TX
 *   CC1101 reset done
 *   CC1101 config written, SCAL OK FSCAL3=.. FSCAL2=.. FSCAL1=..
 *   CC1101 PARTNUM=00 VERSION=14 CFG=OK PA=OK
 *
 * TX, once a second:
 *   TX #12 done MARCSTATE=01 TXBYTES=00
 *     (MARCSTATE 01 = IDLE again, TX FIFO empty: the whole packet left)
 *   "TX #12 TIMEOUT" = no GDO0 falling edge within a second: GDO0
 *   wiring, or the chip never reached TX.
 *
 * RX, per packet:
 *   RX #12 len=11 RSSI=-45 dBm LQI=3 CRC=OK GDO2=1 lost=0 "hello 433"
 *   and every 5 s without packets:
 *   RX listening MARCSTATE=0D RSSI=-104 dBm   (0D = RX; RSSI = noise floor)
 *
 * LEDs: LED1 toggles per packet sent (TX) or received with CRC OK (RX).
 * LED2 (green) on = the last operation failed; after a chip timeout it is
 * reset and configured again on the next tick.
 *
 * The two-modules-on-one-board link test (A on eUSCI_A3, B on eUSCI_B1)
 * that was used to check the radio link with one LaunchPad is kept, not
 * compiled, in two_module_version/.
 */
#include <msp430.h>
#include <stddef.h>
#include "drivers/board/board.h"
#include "drivers/msp430/cs.h"
#include "drivers/msp430/gpio.h"
#include "drivers/msp430/pmm.h"
#include "drivers/msp430/timer.h"
#include "drivers/msp430/uart.h"
#include "drivers/devices/cc1101.h"
#include "drivers/devices/cc1101_settings.h"

#define TMR_TICK        TIMER_A0
#define TICK_MS         (1000u)
#define DBG_UART        UART_A0
#define MAX_REG_LINES   (8u)            /* mismatch lines printed per check */
#define RX_IDLE_REPORT  (5u)            /* ticks without a packet per RX report */

#define RF_PATABLE      cc1101_patable_433_0dbm

static const char k_text[] = "hello 433";
#define TEXT_LEN        (sizeof k_text - 1u)
#define PAYLOAD_LEN     (2u + TEXT_LEN)

typedef enum { ROLE_TX = 0, ROLE_RX } role_t;

static volatile bool s_tick;
static volatile bool s_gdo0_fell;
static uint8_t s_uart_buf[256];
static uint8_t s_cfg[CC1101_CONFIG_SIZE];   /* image actually written */

static bool on_tick(timer_id_t id)
{
    (void)id;
    s_tick = true;
    return true;                        /* wake main */
}

static bool on_gdo0(gpio_port_t port, uint8_t pin_number)
{
    (void)port;
    (void)pin_number;
    s_gdo0_fell = true;                 /* end of packet (TX or RX) */
    return true;                        /* wake main */
}

/* ---- Tiny print helpers (no printf: too big for a debug line) ------- */

static void print(const char *s)
{
    uint16_t n = 0u;
    while (s[n] != '\0') { n++; }
    uart_write_all(DBG_UART, s, n);
}

static void print_hex8(uint8_t v)
{
    static const char k_hex[] = "0123456789ABCDEF";
    char out[2];
    out[0] = k_hex[(unsigned)(v >> 4)];
    out[1] = k_hex[(unsigned)(v & 0x0Fu)];
    uart_write_all(DBG_UART, out, 2u);
}

static void print_field(const char *name, uint8_t v)
{
    print(name);
    print_hex8(v);
    print(" ");
}

static void print_u16(uint16_t v)
{
    char out[5];
    uint8_t n = 0u;
    do
    {
        out[n++] = (char)('0' + (v % 10u));
        v /= 10u;
    } while (v != 0u);
    while (n != 0u) { n--; uart_write_all(DBG_UART, &out[n], 1u); }
}

static void print_i16(int16_t v)
{
    if (v < 0) { print("-"); print_u16((uint16_t)(-v)); }
    else       { print_u16((uint16_t)v); }
}

/* Printable ASCII only, anything else as '.' */
static void print_text(const uint8_t *p, uint8_t len)
{
    print("\"");
    while (len--)
    {
        char c = (*p >= 0x20u && *p < 0x7Fu) ? (char)*p : '.';
        uart_write_all(DBG_UART, &c, 1u);
        p++;
    }
    print("\"");
}

/* ---- CC1101 bring-up ------------------------------------------------ */

static void gdo0_irq(bool on)
{
    if (on)
    {
        gpio_set_callback(CC1101_GDO0_PORT, CC1101_GDO0_PIN, on_gdo0);
        gpio_set_interrupt_edge(CC1101_GDO0_PORT, CC1101_GDO0_PIN, GPIO_EDGE_FALLING);
        gpio_interrupt_clear(CC1101_GDO0_PORT, CC1101_GDO0_PIN);   /* IES change can set IFG */
        s_gdo0_fell = false;
        gpio_interrupt_enable(CC1101_GDO0_PORT, CC1101_GDO0_PIN);
    }
    else
    {
        gpio_interrupt_disable(CC1101_GDO0_PORT, CC1101_GDO0_PIN);
        gpio_interrupt_clear(CC1101_GDO0_PORT, CC1101_GDO0_PIN);
        s_gdo0_fell = false;
    }
}

/* Readback check of the image and PATABLE; prints one summary line */
static bool config_check(void)
{
    uint8_t partnum = 0u, version = 0u;
    uint8_t got[CC1101_CONFIG_SIZE];
    uint8_t pa[CC1101_PATABLE_SIZE];
    uint8_t a, bad = 0u, pa_bad = 0u;

    if (cc1101_read_status(CC1101_PARTNUM, &partnum) == CC1101_STATUS_TIMEOUT ||
        cc1101_read_status(CC1101_VERSION, &version) == CC1101_STATUS_TIMEOUT ||
        cc1101_read_config(got) == CC1101_STATUS_TIMEOUT ||
        cc1101_read_patable(pa, CC1101_PATABLE_SIZE) == CC1101_STATUS_TIMEOUT)
    {
        print("CC1101 readback TIMEOUT\r\n");
        return false;
    }

    for (a = 0u; a < CC1101_CONFIG_SIZE; a++)
    {
        if (((got[a] ^ s_cfg[a]) & cc1101_config_mask(a)) != 0u) { bad++; }
    }
    for (a = 0u; a < CC1101_PATABLE_SIZE; a++)
    {
        if (pa[a] != RF_PATABLE[a]) { pa_bad++; }
    }

    print("CC1101 ");
    print_field("PARTNUM=", partnum);
    print_field("VERSION=", version);
    print(bad == 0u ? "CFG=OK " : "CFG=BAD ");
    print(pa_bad == 0u ? "PA=OK\r\n" : "PA=BAD\r\n");

    uint8_t lines = 0u;
    for (a = 0u; a < CC1101_CONFIG_SIZE && lines < MAX_REG_LINES; a++)
    {
        if (((got[a] ^ s_cfg[a]) & cc1101_config_mask(a)) == 0u) { continue; }
        print("  REG ");
        print_hex8(a);
        print(" got ");
        print_hex8(got[a]);
        print(" want ");
        print_hex8(s_cfg[a]);
        print("\r\n");
        lines++;
    }

    return partnum == CC1101_PARTNUM_EXPECTED && bad == 0u && pa_bad == 0u;
}

/* Reset, write the packet-mode image and PATABLE, calibrate, verify. */
static bool cc1101_bring_up(void)
{
    gdo0_irq(false);                    /* GDO0 toggles until configured */

    if (cc1101_reset() == CC1101_STATUS_TIMEOUT)
    {
        print("CC1101 reset TIMEOUT\r\n");
        return false;
    }
    print("CC1101 reset done\r\n");

    if (cc1101_write_config(s_cfg) == CC1101_STATUS_TIMEOUT ||
        cc1101_write_patable(RF_PATABLE, CC1101_PATABLE_SIZE) == CC1101_STATUS_TIMEOUT)
    {
        print("CC1101 config write TIMEOUT\r\n");
        return false;
    }

    print("CC1101 config written, SCAL ");
    if (!cc1101_calibrate())
    {
        print("FAILED\r\n");
        return false;
    }
    uint8_t f3 = 0u, f2 = 0u, f1 = 0u;
    (void)cc1101_read_reg(CC1101_FSCAL3, &f3);
    (void)cc1101_read_reg(CC1101_FSCAL2, &f2);
    (void)cc1101_read_reg(CC1101_FSCAL1, &f1);
    print("OK ");
    print_field("FSCAL3=", f3);
    print_field("FSCAL2=", f2);
    print_field("FSCAL1=", f1);
    print("\r\n");

    if (!config_check()) { return false; }

    gdo0_irq(true);
    return true;
}

/* ---- TX role -------------------------------------------------------- */

static uint16_t s_tx_seq;
static bool     s_tx_busy;

static bool tx_send(void)
{
    uint8_t payload[PAYLOAD_LEN];
    uint8_t i;

    payload[0] = (uint8_t)(s_tx_seq >> 8);
    payload[1] = (uint8_t)(s_tx_seq & 0xFFu);
    for (i = 0u; i < TEXT_LEN; i++) { payload[2u + i] = (uint8_t)k_text[i]; }

    s_gdo0_fell = false;
    if (cc1101_tx_packet(payload, (uint8_t)PAYLOAD_LEN) == CC1101_STATUS_TIMEOUT)
    {
        print("TX start TIMEOUT\r\n");
        return false;
    }
    s_tx_busy = true;
    return true;
}

static void tx_done(void)
{
    uint8_t marc = 0xFFu, txbytes = 0xFFu;
    (void)cc1101_read_status_stable(CC1101_MARCSTATE, &marc);
    (void)cc1101_read_status_stable(CC1101_TXBYTES, &txbytes);

    print("TX #");
    print_u16(s_tx_seq);
    print(" done ");
    print_field("MARCSTATE=", (uint8_t)(marc & CC1101_MARC_MASK));
    print_field("TXBYTES=", txbytes);
    print("\r\n");

    bool ok = (marc & CC1101_MARC_MASK) == CC1101_MARC_IDLE && txbytes == 0u;
    if (!ok) { (void)cc1101_strobe(CC1101_SIDLE); (void)cc1101_strobe(CC1101_SFTX); }
    board_led1_toggle();
    board_led2(!ok);

    s_tx_busy = false;
    s_tx_seq++;
}

/* ---- RX role -------------------------------------------------------- */

static uint16_t s_rx_last_seq;
static bool     s_rx_have_seq;
static uint8_t  s_rx_quiet_ticks;

static void rx_packet(void)
{
    uint8_t payload[CC1101_MAX_PAYLOAD];
    cc1101_rx_info_t info;

    /* GDO2 = CRC OK; it drops when the first FIFO byte is read */
    bool gdo2 = cc1101_gdo2();
    cc1101_rx_result_t r = cc1101_rx_read_packet(payload, (uint8_t)sizeof payload, &info);

    switch (r)
    {
    case CC1101_RX_OK:
    case CC1101_RX_CRC_ERROR:
    {
        uint16_t seq = (info.len >= 2u)
                     ? (uint16_t)(((uint16_t)payload[0] << 8) | payload[1]) : 0u;
        print("RX #");
        print_u16(seq);
        print(" len=");
        print_u16(info.len);
        print(" RSSI=");
        print_i16(cc1101_rssi_dbm(info.rssi_raw));
        print(" dBm LQI=");
        print_u16(info.lqi);
        print(info.crc_ok ? " CRC=OK" : " CRC=BAD");
        print(gdo2 ? " GDO2=1" : " GDO2=0");
        if (info.crc_ok)
        {
            print(" lost=");
            print_u16(s_rx_have_seq ? (uint16_t)(seq - s_rx_last_seq - 1u) : 0u);
            s_rx_last_seq = seq;
            s_rx_have_seq = true;
            board_led1_toggle();
        }
        print(" ");
        if (info.len > 2u) { print_text(&payload[2], (uint8_t)(info.len - 2u)); }
        print("\r\n");
        board_led2(!info.crc_ok);
        break;
    }
    case CC1101_RX_EMPTY:       break;  /* sync without a kept packet */
    case CC1101_RX_OVERFLOW:    print("RX FIFO overflow, flushed\r\n"); break;
    case CC1101_RX_BAD_LENGTH:  print("RX bad length, flushed\r\n");   break;
    default:                    print("RX TIMEOUT\r\n");               break;
    }

    s_rx_quiet_ticks = 0u;
    (void)cc1101_rx_start();
}

static void rx_tick(void)
{
    if (++s_rx_quiet_ticks < RX_IDLE_REPORT) { return; }
    s_rx_quiet_ticks = 0u;

    uint8_t marc = 0xFFu, rssi = 0u;
    (void)cc1101_read_status_stable(CC1101_MARCSTATE, &marc);
    (void)cc1101_read_status_stable(CC1101_RSSI, &rssi);
    marc &= CC1101_MARC_MASK;

    print("RX listening ");
    print_field("MARCSTATE=", marc);
    print("RSSI=");
    print_i16(cc1101_rssi_dbm(rssi));
    print(" dBm\r\n");

    if (marc != CC1101_MARC_RX && !s_gdo0_fell)
    {
        print("RX not in RX state, restarting\r\n");
        (void)cc1101_rx_start();
    }
}

/* ---- Main ----------------------------------------------------------- */

int main(void)
{
    board_init(cc1101_pins_safe);       /* radio pins safe before I/O unlock */

    const cs_config_t clk = CS_CONFIG_DEFAULT;
    (void)cs_init(&clk);                /* LFXT fault falls back to VLO */

    /* ACLK can only do 9600 baud from the 32 kHz crystal; on VLO use
     * SMCLK (keeps the DCO on in LPM3, fine for a debug build). */
    board_uart_pins(true);
    const uart_config_t ucfg = {
        9600ul,
        cs_lfxt_running() ? UART_CLK_ACLK : UART_CLK_SMCLK,
        s_uart_buf, sizeof s_uart_buf,
        NULL
    };
    (void)uart_init(DBG_UART, &ucfg);

    /* S1 has its pull-up since board_init(); held at reset = RX */
    const role_t role = board_s1_pressed() ? ROLE_RX : ROLE_TX;
    print("\r\ncc1101_radio: 433.92 MHz GFSK 1.2 kBaud, role ");
    print(role == ROLE_RX ? "RX\r\n" : "TX\r\n");

    /* SmartRF image + this project's packet mode */
    {
        uint8_t a;
        for (a = 0u; a < CC1101_CONFIG_SIZE; a++) { s_cfg[a] = cc1101_cfg_433_gfsk_1k2[a]; }
        cc1101_cfg_packet_mode(s_cfg);
    }

    if (!cc1101_pins_init())
    {
        print("SPI init failed\r\n");
        board_led2(true);               /* SPI setup failed: green LED on */
        for (;;) { pmm_enter_lpm(PMM_LPM3); }
    }

    bool need_setup = true;

    timer_start_periodic_ms(TMR_TICK, TICK_MS, on_tick);

    for (;;)
    {
        /* disable -> check -> sleep: an event between the check and the
         * sleep would otherwise be missed until the next one */
        __disable_interrupt();
        if (!s_tick && !s_gdo0_fell)
        {
            pmm_enter_lpm(PMM_LPM3);    /* sets GIE atomically */
        }
        else
        {
            __enable_interrupt();
        }

        if (s_gdo0_fell)
        {
            s_gdo0_fell = false;
            if (!need_setup)
            {
                if (role == ROLE_TX) { if (s_tx_busy) { tx_done(); } }
                else                 { rx_packet(); }
            }
        }

        if (!s_tick) { continue; }
        s_tick = false;

        if (need_setup)
        {
            if (!cc1101_bring_up())
            {
                board_led2(true);
                continue;               /* retry on the next tick */
            }
            need_setup = false;
            board_led2(false);
            if (role == ROLE_RX)
            {
                if (cc1101_rx_start() == CC1101_STATUS_TIMEOUT) { need_setup = true; }
                else { print("RX started\r\n"); }
                continue;
            }
        }

        if (role == ROLE_TX)
        {
            if (s_tx_busy)
            {
                /* ~150 ms on air; a whole tick without GDO0 falling is a fault */
                print("TX #");
                print_u16(s_tx_seq);
                print(" TIMEOUT (no GDO0 falling edge)\r\n");
                board_led2(true);
                s_tx_busy = false;
                s_tx_seq++;
                need_setup = true;      /* reset and reconfigure */
                continue;
            }
            if (!tx_send())
            {
                board_led2(true);
                need_setup = true;
            }
        }
        else
        {
            rx_tick();
        }
    }
}
