/*
 * cc1101.c - TI CC1101 sub-GHz transceiver, pin level and SPI access layer
 *            ESP8684 (ESP32-C2) / ESP-IDF v5.2 port of the MSP430 driver
 *
 * Differences from the MSP430 version:
 *  - SPI2 through the IDF spi_master driver, polled transactions. The
 *    header byte and the data go out as one full-duplex transaction;
 *    the chip status byte is the first byte received.
 *  - CSn is a plain GPIO (spics_io_num = -1) so the driver can pull it
 *    low and wait for CHIP_RDYn on SO before the first SCLK edge, which
 *    the hardware CS of the SPI peripheral cannot do.
 *  - Timeouts are measured with esp_timer (microseconds since boot), so
 *    they are real times, not loop-count lower bounds.
 *  - For the reset strobe SCLK/SI are taken back from SPI2 by freeing the
 *    bus, then the bus is set up again (public IDF API only).
 */
#include "cc1101.h"
#include <stddef.h>
#include <string.h>
#include "esp_rom_sys.h"
#include "esp_timer.h"

/* Tries for cc1101_read_status_stable(); two agreeing reads normally
 * come in the first two or three. */
#define STABLE_TRIES    (8u)

/* Header + the longest burst (64-byte FIFO) */
#define XFER_MAX        (1u + CC1101_FIFO_SIZE)

static spi_device_handle_t s_dev;
static bool s_bus_up;

static inline void delay_us(uint32_t us) { esp_rom_delay_us(us); }

/* ---- Pins and bus --------------------------------------------------- */

static bool bus_up(void)
{
    if (s_bus_up) { return true; }

    const spi_bus_config_t bus = {
        .mosi_io_num     = CC1101_PIN_SI,
        .miso_io_num     = CC1101_PIN_SO,
        .sclk_io_num     = CC1101_PIN_SCLK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = XFER_MAX,
    };
    if (spi_bus_initialize(CC1101_SPI_HOST, &bus, SPI_DMA_DISABLED) != ESP_OK)
    {
        return false;
    }

    const spi_device_interface_config_t dev = {
        .mode           = 0,                /* CPOL 0, CPHA 0 */
        .clock_speed_hz = CC1101_SPI_HZ,
        .spics_io_num   = -1,               /* CSn driven by this driver */
        .queue_size     = 1,
    };
    if (spi_bus_add_device(CC1101_SPI_HOST, &dev, &s_dev) != ESP_OK)
    {
        (void)spi_bus_free(CC1101_SPI_HOST);
        return false;
    }

    s_bus_up = true;
    return true;
}

static void bus_down(void)
{
    if (!s_bus_up) { return; }
    (void)spi_bus_remove_device(s_dev);
    (void)spi_bus_free(CC1101_SPI_HOST);
    s_dev = NULL;
    s_bus_up = false;
}

bool cc1101_pins_init(void)
{
    /* CSn high before it becomes an output, so it never glitches low */
    gpio_set_level(CC1101_PIN_CSN, 1);
    const gpio_config_t cs = {
        .pin_bit_mask = 1ull << CC1101_PIN_CSN,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&cs) != ESP_OK) { return false; }
    gpio_set_level(CC1101_PIN_CSN, 1);

    const gpio_config_t gdo = {
        .pin_bit_mask = (1ull << CC1101_PIN_GDO0) | (1ull << CC1101_PIN_GDO2),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&gdo) != ESP_OK) { return false; }

    return bus_up();
}

/* ---- Chip select ---------------------------------------------------- */

static inline void cs_high(void) { gpio_set_level(CC1101_PIN_CSN, 1); }
static inline void cs_low(void)  { gpio_set_level(CC1101_PIN_CSN, 0); }

/* Wait for SO (CHIP_RDYn) to go low. true = ready, false = timed out. */
static bool wait_so_low(void)
{
    const int64_t t0 = esp_timer_get_time();
    while (cc1101_so_high())
    {
        if (esp_timer_get_time() - t0 > (int64_t)CC1101_SO_TIMEOUT_US) { return false; }
    }
    return true;
}

/*
 * Pull CSn low and wait for the chip. On timeout CSn goes back high, so
 * the caller only has to return CC1101_STATUS_TIMEOUT. Per SWRS061 10.1
 * the first SCLK edge must not come before SO is low: from SLEEP/XOFF
 * the crystal has to start first, in IDLE/RX/TX SO is low at once.
 */
static bool chip_select(void)
{
    cs_low();
    if (!wait_so_low())
    {
        cs_high();
        return false;
    }
    return true;
}

/* n bytes full duplex, CSn already low. false if the IDF call failed. */
static bool spi_xfer(const uint8_t *tx, uint8_t *rx, size_t n)
{
    spi_transaction_t t = {
        .length    = n * 8u,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    return spi_device_polling_transmit(s_dev, &t) == ESP_OK;
}

/* ---- Reset ---------------------------------------------------------- */

uint8_t cc1101_reset(void)
{
    /*
     * SWRS061 19.1.2. SCLK=1 and SI=0 during the CSn strobe avoid the
     * pin-control mode (SWRS061 11.3); SPI mode 0 would hold SCLK low, so
     * free the bus and drive SI and SCLK as plain GPIO for this part.
     */
    bus_down();
    const gpio_config_t io = {
        .pin_bit_mask = (1ull << CC1101_PIN_SCLK) | (1ull << CC1101_PIN_SI),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&io) != ESP_OK) { return CC1101_STATUS_TIMEOUT; }
    gpio_set_level(CC1101_PIN_SI, 0);
    gpio_set_level(CC1101_PIN_SCLK, 1);

    cs_high();
    delay_us(5u);
    cs_low();                           /* strobe CSn low ... */
    delay_us(5u);
    cs_high();                          /* ... and high */
    delay_us(45u);                      /* >= 40 us high (datasheet) */

    /* Back to SPI2 with CSn still high; SCLK drops to idle low before
     * the chip is selected again. */
    if (!bus_up()) { return CC1101_STATUS_TIMEOUT; }

    if (!chip_select()) { return CC1101_STATUS_TIMEOUT; }

    uint8_t tx = CC1101_SRES;
    uint8_t status = CC1101_STATUS_TIMEOUT;
    if (!spi_xfer(&tx, &status, 1u))
    {
        cs_high();
        return CC1101_STATUS_TIMEOUT;
    }

    /* SO goes high while the reset runs and low again when it is done
     * and the chip sits in IDLE. CSn has to stay low until then. If SO
     * has not risen yet when this first looks, the wait ends early; that
     * is harmless, since every later access waits for CHIP_RDYn again
     * before the first SCLK edge. */
    bool done = wait_so_low();
    cs_high();
    return done ? status : CC1101_STATUS_TIMEOUT;
}

bool cc1101_init(void)
{
    if (!cc1101_pins_init()) { return false; }
    return cc1101_reset() != CC1101_STATUS_TIMEOUT;
}

/* ---- Register access ------------------------------------------------ */

/* One header byte, then len data bytes written from tx or read into rx
 * (one of them NULL; both NULL means header only). */
static uint8_t transaction(uint8_t header, const uint8_t *tx, uint8_t *rx, uint8_t len)
{
    if (!s_bus_up || len > CC1101_FIFO_SIZE) { return CC1101_STATUS_TIMEOUT; }

    uint8_t out[XFER_MAX];
    uint8_t in[XFER_MAX];
    const size_t n = 1u + len;

    out[0] = header;
    if (tx && len != 0u) { memcpy(&out[1], tx, len); }
    else if (len != 0u)  { memset(&out[1], 0x00, len); }

    if (!chip_select()) { return CC1101_STATUS_TIMEOUT; }
    bool ok = spi_xfer(out, in, n);
    cs_high();
    if (!ok) { return CC1101_STATUS_TIMEOUT; }

    if (rx && len != 0u) { memcpy(rx, &in[1], len); }
    return in[0];
}

uint8_t cc1101_strobe(uint8_t strobe)
{
    return transaction((uint8_t)(CC1101_WRITE_SINGLE | (strobe & 0x3Fu)), NULL, NULL, 0u);
}

uint8_t cc1101_strobe_rx_status(uint8_t strobe)
{
    return transaction((uint8_t)(CC1101_READ_SINGLE | (strobe & 0x3Fu)), NULL, NULL, 0u);
}

uint8_t cc1101_write_reg(uint8_t addr, uint8_t value)
{
    return transaction((uint8_t)(CC1101_WRITE_SINGLE | (addr & 0x3Fu)), &value, NULL, 1u);
}

uint8_t cc1101_read_reg(uint8_t addr, uint8_t *value)
{
    if (!value) { return CC1101_STATUS_TIMEOUT; }
    return transaction((uint8_t)(CC1101_READ_SINGLE | (addr & 0x3Fu)), NULL, value, 1u);
}

/*
 * Burst: the address auto-increments through the config registers. The
 * FIFO address does not move, and the PATABLE has its own internal index
 * that steps per byte and resets to 0 when CSn goes high.
 */
uint8_t cc1101_write_burst(uint8_t addr, const uint8_t *data, uint8_t len)
{
    if (!data && len != 0u) { return CC1101_STATUS_TIMEOUT; }
    return transaction((uint8_t)(CC1101_WRITE_BURST | (addr & 0x3Fu)), data, NULL, len);
}

uint8_t cc1101_read_burst(uint8_t addr, uint8_t *data, uint8_t len)
{
    if (!data && len != 0u) { return CC1101_STATUS_TIMEOUT; }
    return transaction((uint8_t)(CC1101_READ_BURST | (addr & 0x3Fu)), NULL, data, len);
}

uint8_t cc1101_read_status(uint8_t addr, uint8_t *value)
{
    if (!value) { return CC1101_STATUS_TIMEOUT; }
    /* Single byte, but with the burst bit: that is what turns 0x30-0x3D
     * from strobes into status registers. */
    return transaction((uint8_t)(CC1101_READ_BURST | (addr & 0x3Fu)), NULL, value, 1u);
}

uint8_t cc1101_read_status_stable(uint8_t addr, uint8_t *value)
{
    if (!value) { return CC1101_STATUS_TIMEOUT; }

    uint8_t prev;
    uint8_t status = cc1101_read_status(addr, &prev);
    if (status == CC1101_STATUS_TIMEOUT) { return status; }

    for (uint8_t i = STABLE_TRIES; i != 0u; i--)
    {
        uint8_t cur;
        status = cc1101_read_status(addr, &cur);
        if (status == CC1101_STATUS_TIMEOUT) { return status; }
        if (cur == prev)
        {
            *value = cur;
            return status;
        }
        prev = cur;
    }

    *value = prev;          /* still changing: best effort, latest value */
    return status;
}

/* ---- Register configuration ----------------------------------------- */

/* First and last address of the two bursts; 0x29-0x2B are skipped */
#define CFG_A_FIRST     CC1101_IOCFG2
#define CFG_A_LAST      CC1101_RCCTRL0
#define CFG_B_FIRST     CC1101_TEST2
#define CFG_B_LAST      CC1101_TEST0

uint8_t cc1101_write_config(const uint8_t *cfg)
{
    if (!cfg) { return CC1101_STATUS_TIMEOUT; }

    uint8_t status = cc1101_write_burst(CFG_A_FIRST, &cfg[CFG_A_FIRST],
                                        (uint8_t)(CFG_A_LAST - CFG_A_FIRST + 1u));
    if (status == CC1101_STATUS_TIMEOUT) { return status; }

    return cc1101_write_burst(CFG_B_FIRST, &cfg[CFG_B_FIRST],
                              (uint8_t)(CFG_B_LAST - CFG_B_FIRST + 1u));
}

uint8_t cc1101_read_config(uint8_t *out)
{
    return cc1101_read_burst(CC1101_IOCFG2, out, (uint8_t)CC1101_CONFIG_SIZE);
}

uint8_t cc1101_config_mask(uint8_t addr)
{
    switch (addr)
    {
    case CC1101_FSTEST:
    case CC1101_PTEST:
    case CC1101_AGCTEST: return 0x00u;     /* never written */
    case CC1101_FSCAL3:  return 0xF0u;     /* [3:0] calibration result */
    case CC1101_FSCAL2:  return 0xE0u;     /* [4:0] VCO current result */
    case CC1101_FSCAL1:  return 0x00u;     /* capacitor array result */
    default:             return (addr <= CC1101_CONFIG_LAST) ? 0xFFu : 0x00u;
    }
}

bool cc1101_wait_marcstate(uint8_t want, uint16_t timeout_us, uint8_t *last)
{
    const int64_t t0 = esp_timer_get_time();
    uint8_t marc = 0xFFu;
    for (;;)
    {
        if (cc1101_read_status_stable(CC1101_MARCSTATE, &marc) == CC1101_STATUS_TIMEOUT)
        {
            if (last) { *last = 0xFFu; }
            return false;
        }
        marc &= CC1101_MARC_MASK;
        if (marc == want) { break; }
        if (esp_timer_get_time() - t0 > (int64_t)timeout_us)
        {
            if (last) { *last = marc; }
            return false;
        }
        delay_us(10u);
    }
    if (last) { *last = marc; }
    return true;
}

bool cc1101_calibrate(void)
{
    if (cc1101_strobe(CC1101_SCAL) == CC1101_STATUS_TIMEOUT) { return false; }

    /*
     * The first MARCSTATE read can still show IDLE if the state machine
     * has not moved yet, and waiting for IDLE would then return at once.
     * So first see it leave IDLE (calibration lasts ~0.7 ms, far longer
     * than this window), then wait for it to come back.
     */
    const int64_t t0 = esp_timer_get_time();
    uint8_t marc = CC1101_MARC_IDLE;
    for (;;)
    {
        if (cc1101_read_status_stable(CC1101_MARCSTATE, &marc) == CC1101_STATUS_TIMEOUT)
        {
            return false;
        }
        if ((marc & CC1101_MARC_MASK) != CC1101_MARC_IDLE) { break; }
        if (esp_timer_get_time() - t0 > 200) { return false; }    /* SCAL had no effect */
        delay_us(5u);
    }

    /* ~0.7 ms nominal at 26 MHz */
    return cc1101_wait_marcstate(CC1101_MARC_IDLE, 5000u, NULL);
}

/* ---- Packets -------------------------------------------------------- */

uint8_t cc1101_tx_packet(const uint8_t *payload, uint8_t len)
{
    if (!payload || len == 0u || len > CC1101_MAX_PAYLOAD) { return CC1101_STATUS_TIMEOUT; }

    /* SFTX is only accepted in IDLE or TXFIFO_UNDERFLOW */
    if (cc1101_strobe(CC1101_SIDLE) == CC1101_STATUS_TIMEOUT) { return CC1101_STATUS_TIMEOUT; }
    if (!cc1101_wait_marcstate(CC1101_MARC_IDLE, 1000u, NULL)) { return CC1101_STATUS_TIMEOUT; }
    if (cc1101_strobe(CC1101_SFTX) == CC1101_STATUS_TIMEOUT)   { return CC1101_STATUS_TIMEOUT; }

    /* A single write to 0x3F goes to the TX FIFO: length byte first */
    if (cc1101_write_reg(CC1101_FIFO, len) == CC1101_STATUS_TIMEOUT)            { return CC1101_STATUS_TIMEOUT; }
    if (cc1101_write_burst(CC1101_FIFO, payload, len) == CC1101_STATUS_TIMEOUT) { return CC1101_STATUS_TIMEOUT; }

    /* From IDLE, STX calibrates (MCSM0.FS_AUTOCAL) and transmits */
    return cc1101_strobe(CC1101_STX);
}

uint8_t cc1101_rx_start(void)
{
    if (cc1101_strobe(CC1101_SIDLE) == CC1101_STATUS_TIMEOUT) { return CC1101_STATUS_TIMEOUT; }
    if (!cc1101_wait_marcstate(CC1101_MARC_IDLE, 1000u, NULL)) { return CC1101_STATUS_TIMEOUT; }
    if (cc1101_strobe(CC1101_SFRX) == CC1101_STATUS_TIMEOUT)   { return CC1101_STATUS_TIMEOUT; }
    return cc1101_strobe(CC1101_SRX);
}

/* SIDLE + SFRX; SFRX is only accepted in IDLE or RXFIFO_OVERFLOW */
static void rx_flush(void)
{
    (void)cc1101_strobe(CC1101_SIDLE);
    (void)cc1101_wait_marcstate(CC1101_MARC_IDLE, 1000u, NULL);
    (void)cc1101_strobe(CC1101_SFRX);
}

cc1101_rx_result_t cc1101_rx_read_packet(uint8_t *payload, uint8_t max,
                                         cc1101_rx_info_t *info)
{
    if (!payload || !info) { return CC1101_RX_TIMEOUT; }
    info->len = 0u;
    info->rssi_raw = 0u;
    info->lqi = 0x7Fu;
    info->crc_ok = false;

    uint8_t rxbytes;
    if (cc1101_read_status_stable(CC1101_RXBYTES, &rxbytes) == CC1101_STATUS_TIMEOUT)
    {
        return CC1101_RX_TIMEOUT;
    }
    if (rxbytes & CC1101_FIFO_OVF)
    {
        rx_flush();
        return CC1101_RX_OVERFLOW;
    }

    uint8_t count = (uint8_t)(rxbytes & CC1101_FIFO_COUNT);
    if (count == 0u) { return CC1101_RX_EMPTY; }

    uint8_t len;
    if (cc1101_read_reg(CC1101_FIFO, &len) == CC1101_STATUS_TIMEOUT) { return CC1101_RX_TIMEOUT; }

    /* length + payload + RSSI + LQI must all be in the FIFO already */
    if (len == 0u || len > max || (uint16_t)len + 3u > count)
    {
        rx_flush();
        return CC1101_RX_BAD_LENGTH;
    }

    uint8_t st[2];
    if (cc1101_read_burst(CC1101_FIFO, payload, len) == CC1101_STATUS_TIMEOUT ||
        cc1101_read_burst(CC1101_FIFO, st, 2u) == CC1101_STATUS_TIMEOUT)
    {
        return CC1101_RX_TIMEOUT;
    }

    info->len      = len;
    info->rssi_raw = st[0];
    info->lqi      = (uint8_t)(st[1] & 0x7Fu);
    info->crc_ok   = (st[1] & 0x80u) != 0u;

    return info->crc_ok ? CC1101_RX_OK : CC1101_RX_CRC_ERROR;
}
