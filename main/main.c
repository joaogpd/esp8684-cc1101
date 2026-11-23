/*
 * main.c - esp8684-cc1101: CC1101 link test on the NodeMCU ESP32-C2 board
 *
 * Port of the MSP430 cc1101_radio test app (reference/msp430/cc1101_radio/
 * main.c), same radio settings and same packet: 433.92 MHz, GFSK
 * 1.2 kBaud, 0 dBm, variable length with CRC, payload = 16-bit sequence
 * number + "hello 433". Wiring: see components/cc1101/include/cc1101.h.
 *
 * Role: starts in RX (the first interop test: the MSP430 LaunchPad running
 * cc1101_radio in TX sends one packet a second). Press BOOT (IO9) any time
 * after start-up to switch between RX and TX. Do not hold BOOT while
 * pressing EN/reset: that enters the ROM download mode instead.
 *
 * Console: 115200 8N1 on the board's USB port (idf.py -p COMx monitor).
 *
 * Boot output:
 *   esp8684-cc1101: 433.92 MHz GFSK 1.2 kBaud, role RX (BOOT switches)
 *   CC1101 reset done
 *   CC1101 config written, SCAL OK FSCAL3=.. FSCAL2=.. FSCAL1=..
 *   CC1101 PARTNUM=00 VERSION=14 CFG=OK PA=OK
 *   RX started
 *
 * RX, per packet:
 *   RX #12 len=11 RSSI=-45 dBm LQI=3 CRC=OK GDO2=1 lost=0 "hello 433"
 *   and every 5 s without packets:
 *   RX listening MARCSTATE=0D RSSI=-104 dBm   (0D = RX; RSSI = noise floor)
 *
 * TX, once a second:
 *   TX #12 done MARCSTATE=01 TXBYTES=00
 *     (MARCSTATE 01 = IDLE again, TX FIFO empty: the whole packet left)
 *   "TX #12 TIMEOUT" = no GDO0 falling edge within a second: GDO0
 *   wiring, or the chip never reached TX.
 *
 * Faults:
 *   "CC1101 reset TIMEOUT" = SO never went low after CSn fell: no supply,
 *   CSn or SO not connected, or the module is missing. Retried each second.
 *   "CFG=BAD" + "REG aa got xx want yy" lines = SPI data corrupted (SI/SCLK
 *   wiring, long wires).
 */
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "cc1101.h"
#include "cc1101_settings.h"

#define PIN_BOOT        GPIO_NUM_9      /* BOOT button, low when pressed */

#define POLL_MS         (50u)           /* button poll / wait slice */
#define TICK_MS         (1000u)
#define MAX_REG_LINES   (8u)            /* mismatch lines printed per check */
#define RX_IDLE_REPORT  (5u)            /* ticks without a packet per RX report */

#define RF_PATABLE      cc1101_patable_433_0dbm

static const char k_text[] = "hello 433";
#define TEXT_LEN        (sizeof k_text - 1u)
#define PAYLOAD_LEN     (2u + TEXT_LEN)

typedef enum { ROLE_RX = 0, ROLE_TX } role_t;

static TaskHandle_t s_radio_task;
static uint8_t s_cfg[CC1101_CONFIG_SIZE];   /* image actually written */

/* GDO0 falling edge = end of packet (TX or RX): wake the radio task */
static void IRAM_ATTR on_gdo0(void *arg)
{
    (void)arg;
    if (s_radio_task == NULL) { return; }
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_radio_task, &woken);
    portYIELD_FROM_ISR(woken);
}

static void gdo0_irq(bool on)
{
    if (on)
    {
        (void)ulTaskNotifyTake(pdTRUE, 0);      /* drop a stale edge */
        gpio_set_intr_type(CC1101_PIN_GDO0, GPIO_INTR_NEGEDGE);
        gpio_intr_enable(CC1101_PIN_GDO0);
    }
    else
    {
        gpio_intr_disable(CC1101_PIN_GDO0);
        (void)ulTaskNotifyTake(pdTRUE, 0);
    }
}

/* Printable ASCII only, anything else as '.' */
static void print_text(const uint8_t *p, uint8_t len)
{
    putchar('"');
    while (len--)
    {
        putchar((*p >= 0x20u && *p < 0x7Fu) ? (char)*p : '.');
        p++;
    }
    putchar('"');
}

/* ---- CC1101 bring-up ------------------------------------------------ */

/* Readback check of the image and PATABLE; prints one summary line */
static bool config_check(void)
{
    uint8_t partnum = 0u, version = 0u;
    uint8_t got[CC1101_CONFIG_SIZE];
    uint8_t pa[CC1101_PATABLE_SIZE];
    unsigned bad = 0u, pa_bad = 0u;

    if (cc1101_read_status(CC1101_PARTNUM, &partnum) == CC1101_STATUS_TIMEOUT ||
        cc1101_read_status(CC1101_VERSION, &version) == CC1101_STATUS_TIMEOUT ||
        cc1101_read_config(got) == CC1101_STATUS_TIMEOUT ||
        cc1101_read_patable(pa, CC1101_PATABLE_SIZE) == CC1101_STATUS_TIMEOUT)
    {
        printf("CC1101 readback TIMEOUT\n");
        return false;
    }

    for (unsigned a = 0u; a < CC1101_CONFIG_SIZE; a++)
    {
        if (((got[a] ^ s_cfg[a]) & cc1101_config_mask((uint8_t)a)) != 0u) { bad++; }
    }
    for (unsigned a = 0u; a < CC1101_PATABLE_SIZE; a++)
    {
        if (pa[a] != RF_PATABLE[a]) { pa_bad++; }
    }

    printf("CC1101 PARTNUM=%02X VERSION=%02X %s %s\n", partnum, version,
           bad == 0u ? "CFG=OK" : "CFG=BAD", pa_bad == 0u ? "PA=OK" : "PA=BAD");

    unsigned lines = 0u;
    for (unsigned a = 0u; a < CC1101_CONFIG_SIZE && lines < MAX_REG_LINES; a++)
    {
        if (((got[a] ^ s_cfg[a]) & cc1101_config_mask((uint8_t)a)) == 0u) { continue; }
        printf("  REG %02X got %02X want %02X\n", a, got[a], s_cfg[a]);
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
        printf("CC1101 reset TIMEOUT\n");
        return false;
    }
    printf("CC1101 reset done\n");

    if (cc1101_write_config(s_cfg) == CC1101_STATUS_TIMEOUT ||
        cc1101_write_patable(RF_PATABLE, CC1101_PATABLE_SIZE) == CC1101_STATUS_TIMEOUT)
    {
        printf("CC1101 config write TIMEOUT\n");
        return false;
    }

    printf("CC1101 config written, SCAL ");
    if (!cc1101_calibrate())
    {
        printf("FAILED\n");
        return false;
    }
    uint8_t f3 = 0u, f2 = 0u, f1 = 0u;
    (void)cc1101_read_reg(CC1101_FSCAL3, &f3);
    (void)cc1101_read_reg(CC1101_FSCAL2, &f2);
    (void)cc1101_read_reg(CC1101_FSCAL1, &f1);
    printf("OK FSCAL3=%02X FSCAL2=%02X FSCAL1=%02X\n", f3, f2, f1);

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

    payload[0] = (uint8_t)(s_tx_seq >> 8);
    payload[1] = (uint8_t)(s_tx_seq & 0xFFu);
    for (unsigned i = 0u; i < TEXT_LEN; i++) { payload[2u + i] = (uint8_t)k_text[i]; }

    (void)ulTaskNotifyTake(pdTRUE, 0);
    if (cc1101_tx_packet(payload, (uint8_t)PAYLOAD_LEN) == CC1101_STATUS_TIMEOUT)
    {
        printf("TX start TIMEOUT\n");
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
    marc &= CC1101_MARC_MASK;

    printf("TX #%u done MARCSTATE=%02X TXBYTES=%02X\n", s_tx_seq, marc, txbytes);

    if (!(marc == CC1101_MARC_IDLE && txbytes == 0u))
    {
        (void)cc1101_strobe(CC1101_SIDLE);
        (void)cc1101_strobe(CC1101_SFTX);
    }
    s_tx_busy = false;
    s_tx_seq++;
}

/* ---- RX role -------------------------------------------------------- */

static uint16_t s_rx_last_seq;
static bool     s_rx_have_seq;
static unsigned s_rx_quiet_ticks;

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
        printf("RX #%u len=%u RSSI=%d dBm LQI=%u %s %s", seq, info.len,
               cc1101_rssi_dbm(info.rssi_raw), info.lqi,
               info.crc_ok ? "CRC=OK" : "CRC=BAD", gdo2 ? "GDO2=1" : "GDO2=0");
        if (info.crc_ok)
        {
            printf(" lost=%u", s_rx_have_seq ? (uint16_t)(seq - s_rx_last_seq - 1u) : 0u);
            s_rx_last_seq = seq;
            s_rx_have_seq = true;
        }
        putchar(' ');
        if (info.len > 2u) { print_text(&payload[2], (uint8_t)(info.len - 2u)); }
        putchar('\n');
        break;
    }
    case CC1101_RX_EMPTY:       break;  /* sync without a kept packet */
    case CC1101_RX_OVERFLOW:    printf("RX FIFO overflow, flushed\n"); break;
    case CC1101_RX_BAD_LENGTH:  printf("RX bad length, flushed\n");   break;
    default:                    printf("RX TIMEOUT\n");               break;
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

    printf("RX listening MARCSTATE=%02X RSSI=%d dBm\n", marc, cc1101_rssi_dbm(rssi));

    if (marc != CC1101_MARC_RX && !cc1101_gdo0())
    {
        printf("RX not in RX state, restarting\n");
        (void)cc1101_rx_start();
    }
}

/* ---- Radio task ----------------------------------------------------- */

static void radio_task(void *arg)
{
    (void)arg;
    role_t role = ROLE_RX;
    bool need_setup = true;
    bool boot_was_down = false;
    int64_t next_tick = esp_timer_get_time();

    printf("\nesp8684-cc1101: 433.92 MHz GFSK 1.2 kBaud, role RX (BOOT switches)\n");

    for (;;)
    {
        /* Sleep until GDO0 falls or the next poll slice */
        bool gdo0_fell = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(POLL_MS)) != 0u;

        if (gdo0_fell && !need_setup)
        {
            if (role == ROLE_TX) { if (s_tx_busy) { tx_done(); } }
            else                 { rx_packet(); }
        }

        /* BOOT pressed: switch role and start over */
        bool boot_down = gpio_get_level(PIN_BOOT) == 0;
        if (boot_down && !boot_was_down)
        {
            role = (role == ROLE_RX) ? ROLE_TX : ROLE_RX;
            printf("role %s\n", role == ROLE_RX ? "RX" : "TX");
            s_tx_busy = false;
            s_rx_have_seq = false;
            need_setup = true;
            next_tick = esp_timer_get_time();
        }
        boot_was_down = boot_down;

        int64_t now = esp_timer_get_time();
        if (now < next_tick) { continue; }
        next_tick += (int64_t)TICK_MS * 1000;
        if (next_tick < now) { next_tick = now + (int64_t)TICK_MS * 1000; }

        if (need_setup)
        {
            if (!cc1101_bring_up()) { continue; }   /* retry on the next tick */
            need_setup = false;
            if (role == ROLE_RX)
            {
                if (cc1101_rx_start() == CC1101_STATUS_TIMEOUT) { need_setup = true; }
                else { printf("RX started\n"); }
                s_rx_quiet_ticks = 0u;
                continue;
            }
        }

        if (role == ROLE_TX)
        {
            if (s_tx_busy)
            {
                /* ~150 ms on air; a whole tick without GDO0 falling is a fault */
                printf("TX #%u TIMEOUT (no GDO0 falling edge)\n", s_tx_seq);
                s_tx_busy = false;
                s_tx_seq++;
                need_setup = true;      /* reset and reconfigure */
                continue;
            }
            if (!tx_send()) { need_setup = true; }
        }
        else
        {
            rx_tick();
        }
    }
}

void app_main(void)
{
    /* SmartRF image + this project's packet mode */
    for (unsigned a = 0u; a < CC1101_CONFIG_SIZE; a++) { s_cfg[a] = cc1101_cfg_433_gfsk_1k2[a]; }
    cc1101_cfg_packet_mode(s_cfg);

    const gpio_config_t boot = {
        .pin_bit_mask = 1ull << PIN_BOOT,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    (void)gpio_config(&boot);

    if (!cc1101_pins_init())
    {
        printf("SPI init failed\n");
        return;
    }

    /* ISR in place (disabled) before the task can enable it */
    (void)gpio_install_isr_service(0);
    (void)gpio_isr_handler_add(CC1101_PIN_GDO0, on_gdo0, NULL);
    gpio_intr_disable(CC1101_PIN_GDO0);     /* enabled after bring-up */

    xTaskCreate(radio_task, "radio", 4096, NULL, 5, &s_radio_task);
}
