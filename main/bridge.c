/*
 * bridge.c - line router between USB serial (UART0) and BLE (NUS)
 *
 * Tasks:
 *   usb_rx   reads UART0 (the console UART, 115200 8N1 through the CH340C)
 *   router   assembles lines per source, runs `!` commands, echoes
 * plus the NimBLE host task, which calls on_ble_rx() for each write.
 * Raw bytes from both sources go through one queue, so the router is the
 * only place that touches the line buffers.
 *
 * Output to the PC uses stdout (printf), which the UART driver serialises
 * through the VFS; output to the phone is serialised by s_out_lock, so a
 * line longer than one notification is never interleaved with another.
 */
#include "bridge.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_vfs_dev.h"
#include "sdkconfig.h"
#include "ble_nus.h"

#define CONSOLE_UART    ((uart_port_t)CONFIG_ESP_CONSOLE_UART_NUM)

#define LINE_LEN_MAX        (120u)      /* characters, without the line ending */
#define IDLE_FLUSH_US   (300000)    /* a line without CR/LF ends after this */
#define CHUNK_MAX       (64u)
#define QUEUE_LEN       (16u)

typedef enum { SRC_USB = 0, SRC_BLE, SRC_COUNT } src_t;

typedef struct
{
    uint8_t src;
    uint8_t len;
    uint8_t data[CHUNK_MAX];
} chunk_t;

typedef struct
{
    char    buf[LINE_LEN_MAX + 1u];
    size_t  len;
    bool    overflow;
    int64_t last_us;
} line_t;

typedef struct
{
    bool     radio_up;
    uint32_t rx_ok;
    uint32_t rx_bad;
    int16_t  rssi;
    uint8_t  lqi;
    int64_t  last_ok_us;            /* 0 = never */
} radio_stats_t;

static QueueHandle_t     s_q;
static SemaphoreHandle_t s_out_lock;
static line_t            s_line[SRC_COUNT];

static portMUX_TYPE  s_stats_mux = portMUX_INITIALIZER_UNLOCKED;
static radio_stats_t s_stats;

/* ---- Input ----------------------------------------------------------- */

static void push(uint8_t src, const uint8_t *data, size_t len, TickType_t wait)
{
    while (len != 0u)
    {
        chunk_t c;
        c.src = src;
        c.len = (uint8_t)((len < CHUNK_MAX) ? len : CHUNK_MAX);
        memcpy(c.data, data, c.len);
        if (xQueueSend(s_q, &c, wait) != pdTRUE) { return; }   /* full: drop */
        data += c.len;
        len  -= c.len;
    }
}

/* NimBLE host task: never block it */
static void on_ble_rx(const uint8_t *data, uint16_t len)
{
    push(SRC_BLE, data, len, 0);
}

static void usb_rx_task(void *arg)
{
    (void)arg;
    uint8_t buf[CHUNK_MAX];
    for (;;)
    {
        int n = uart_read_bytes(CONSOLE_UART, buf, sizeof buf, pdMS_TO_TICKS(20));
        if (n > 0) { push(SRC_USB, buf, (size_t)n, portMAX_DELAY); }
    }
}

/* ---- Output ---------------------------------------------------------- */

static void out_usb(const char *line)
{
    printf("%s\n", line);           /* VFS turns \n into CR LF */
}

static void out_ble(const char *line)
{
    if (!ble_nus_ready()) { return; }
    (void)ble_nus_send((const uint8_t *)line, strlen(line));
    (void)ble_nus_send((const uint8_t *)"\r\n", 2u);
}

static void reply(src_t src, const char *line)
{
    xSemaphoreTake(s_out_lock, portMAX_DELAY);
    if (src == SRC_BLE) { out_ble(line); }
    else                { out_usb(line); }
    xSemaphoreGive(s_out_lock);
}

void bridge_out(const char *line)
{
    if (!s_out_lock) { out_usb(line); return; }     /* before bridge_init */
    xSemaphoreTake(s_out_lock, portMAX_DELAY);
    out_usb(line);
    out_ble(line);
    xSemaphoreGive(s_out_lock);
}

/* ---- Radio state ----------------------------------------------------- */

void bridge_note_radio(bool up)
{
    portENTER_CRITICAL(&s_stats_mux);
    s_stats.radio_up = up;
    portEXIT_CRITICAL(&s_stats_mux);
}

void bridge_note_rx(bool crc_ok, int16_t rssi_dbm, uint8_t lqi)
{
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&s_stats_mux);
    if (crc_ok)
    {
        s_stats.rx_ok++;
        s_stats.last_ok_us = now;
    }
    else
    {
        s_stats.rx_bad++;
    }
    s_stats.rssi = rssi_dbm;
    s_stats.lqi  = lqi;
    portEXIT_CRITICAL(&s_stats_mux);
}

/* ---- Lines ----------------------------------------------------------- */

static void cmd_local(src_t src, const char *cmd)
{
    char out[160];

    if (strcmp(cmd, "link") == 0)
    {
        radio_stats_t st;
        portENTER_CRITICAL(&s_stats_mux);
        st = s_stats;
        portEXIT_CRITICAL(&s_stats_mux);

        long age = (st.last_ok_us == 0)
                 ? -1L : (long)((esp_timer_get_time() - st.last_ok_us) / 1000000);
        snprintf(out, sizeof out,
                 "!link,ble=%d,mtu=%u,radio=%s,rx=%" PRIu32 ",bad=%" PRIu32
                 ",rssi=%d,lqi=%u,age=%ld",
                 ble_nus_ready() ? 1 : 0, (unsigned)ble_nus_mtu(),
                 st.radio_up ? "ok" : "down", st.rx_ok, st.rx_bad,
                 (int)st.rssi, (unsigned)st.lqi, age);
        reply(src, out);
    }
    else if (strcmp(cmd, "help") == 0)
    {
        reply(src, "!help,link,help");
    }
    else
    {
        reply(src, "!err,unknown");
    }
}

static void handle_line(src_t src, const char *line)
{
    if (src == SRC_BLE)
    {
        /* Show phone traffic on the PC too (not an '@' line, so a host
         * following the logger protocol ignores it) */
        char note[LINE_LEN_MAX + 8u];
        snprintf(note, sizeof note, "ble> %s", line);
        xSemaphoreTake(s_out_lock, portMAX_DELAY);
        out_usb(note);
        xSemaphoreGive(s_out_lock);
    }

    if (line[0] == '!')
    {
        cmd_local(src, &line[1]);
        return;
    }

    /* Step 1: no radio link to the logger yet, so echo */
    char out[LINE_LEN_MAX + 8u];
    snprintf(out, sizeof out, "echo: %s", line);
    reply(src, out);
}

static void finish_line(src_t src)
{
    line_t *l = &s_line[src];
    if (l->overflow)
    {
        reply(src, "!err,toolong");
    }
    else if (l->len != 0u)
    {
        l->buf[l->len] = '\0';
        handle_line(src, l->buf);
    }
    l->len = 0u;
    l->overflow = false;
}

static void feed(src_t src, const uint8_t *data, size_t len)
{
    line_t *l = &s_line[src];
    for (size_t i = 0u; i < len; i++)
    {
        uint8_t c = data[i];
        if (c == '\r' || c == '\n')
        {
            finish_line(src);               /* CR LF: the LF finds len 0 */
        }
        else if (l->len < LINE_LEN_MAX)
        {
            l->buf[l->len++] = (char)c;
        }
        else
        {
            l->overflow = true;
        }
    }
    l->last_us = esp_timer_get_time();
}

static void router_task(void *arg)
{
    (void)arg;
    for (;;)
    {
        chunk_t c;
        if (xQueueReceive(s_q, &c, pdMS_TO_TICKS(50)) == pdTRUE && c.src < SRC_COUNT)
        {
            feed((src_t)c.src, c.data, c.len);
        }

        /* A line without CR/LF counts as done after a quiet gap */
        int64_t now = esp_timer_get_time();
        for (unsigned s = 0u; s < SRC_COUNT; s++)
        {
            line_t *l = &s_line[s];
            if ((l->len != 0u || l->overflow) && now - l->last_us > IDLE_FLUSH_US)
            {
                finish_line((src_t)s);
            }
        }
    }
}

/* ---- Init ------------------------------------------------------------ */

void bridge_init(const char *ble_name)
{
    s_q = xQueueCreate(QUEUE_LEN, sizeof(chunk_t));
    s_out_lock = xSemaphoreCreateMutex();

    /* UART0 RX through the driver; stdout goes through it as well, so
     * printf and uart_read_bytes do not fight over the FIFO */
    if (uart_driver_install(CONSOLE_UART, 512, 0, 0, NULL, 0) == ESP_OK)
    {
        esp_vfs_dev_uart_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
        xTaskCreate(usb_rx_task, "usb_rx", 2560, NULL, 4, NULL);
    }
    else
    {
        printf("bridge: UART driver install failed, no USB input\n");
    }

    if (!ble_nus_init(ble_name, on_ble_rx))
    {
        printf("bridge: BLE init failed\n");
    }

    xTaskCreate(router_task, "router", 3584, NULL, 4, NULL);

    printf("bridge: USB + BLE \"%s\" ready; type a line, or !help\n", ble_name);
}
