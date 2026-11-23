# CLAUDE.md - esp8684-cc1101

Context for Claude sessions continuing this project (handed over from the
desktop PC session of 2026-11-23).

## Goal

Drivers and firmware for an **Espressif ESP8684 (ESP32-C2 die)** driving a
**TI CC1101** sub-GHz transceiver over SPI, so the ESP8684 can talk by radio
to the user's **MSP430FR5994 supercapacitor data logger** ("msp430logger",
project `msp430drivers` / folder `driver_development`).

## Status (2026-11-23, laptop session)

- Branch `esp-cc1101-port`: ESP-IDF project with the CC1101 driver ported
  from the MSP430 (`components/cc1101/`) and the RX/TX link test app
  (`main/main.c`, same packets and output lines as the MSP430 test app).
- **Builds clean** with ESP-IDF v5.2.1 (no warnings under IDF's -Werror
  flags; app 0x26140 bytes, 85% of the 1 MB app partition free).
  **Not run on hardware yet.**
- Next: build, flash, run the RX test against the MSP430 in TX; then the
  BLE step (below); then the radio framing layer.

## Decisions (answered by the user 2026-11-23)

1. **Board: "NodeMCU ESP32-C2"** devkit (sold by UsinaInfo, Brazil):
   ESP8684 module (Ai-Thinker-style "ESP-C2-12", FCC ID 2AL3B-ESPC2-12),
   **CH340C** USB-serial on USB-C, BOOT button = IO9, EN reset button,
   **no user LED** (power LED only). Headers expose IO0-IO7, IO9, IO10,
   IO18, IO19 (RXD0), IO20 (TXD0). IO8 is not on the headers.
   Flash size and crystal (26 vs 40 MHz) still to confirm from esptool's
   output on the first flash; `sdkconfig.defaults` assumes 2 MB / 26 MHz.
2. **CC1101 module**: the same "CC1101 433 MHz V2.0" board as on the
   MSP430 (26 MHz crystal, **3.3 V only**).
3. **Wiring** (chosen by Claude; SPI pins are the SPI2 IO MUX pins printed
   on the board's pinout):

   | CC1101 | ESP8684 | note |
   |---|---|---|
   | VCC | 3V3 | never 5 V |
   | GND | GND | |
   | SCLK | IO6 | FSPICLK |
   | SI (MOSI) | IO7 | FSPID |
   | SO (MISO/GDO1) | IO2 | FSPIQ; polled as GPIO for CHIP_RDYn |
   | CSn | IO10 | plain GPIO, driven by the driver |
   | GDO0 | IO3 | falling-edge IRQ = end of packet |
   | GDO2 | IO1 | CRC OK |

   Left free: IO0, IO4, IO5, IO18. Not used: IO8/IO9 (strapping, IO9 =
   BOOT), IO19/IO20 (console).
4. **Over the air: transparent bridge** for the logger's `@` machine-mode
   lines (needs the framing layer: seq, fragment, ack/retry; the MSP430
   side of it is done in the MSP430 projects).
5. **ESP8684 -> phone over BLE.** Plan: NimBLE peripheral with the
   Nordic UART Service (NUS, 6E400001-B5A3-F393-E0A9-E50E24DCCA9E; RX =
   write, TX = notify), MTU raised to 247 so one `@` line fits one or a few
   notifications. First test with a generic phone app (nRF Connect, or
   "Serial Bluetooth Terminal" on Android / Bluefruit Connect on iOS); a
   custom phone UI later (Web Bluetooth page if the phone is Android +
   Chrome; native app otherwise). Still to ask: phone OS, and what
   msp430_dashboard is built with.

## Radio settings that MUST match the MSP430 side

From `reference/msp430/cc1101_radio/cc1101_settings.c` (verified on
hardware on the MSP430 2026-11-22). Reuse the register image byte for byte.

- SmartRF Studio 7 "typical settings" preset 1, recomputed for
  **433.92 MHz** (FREQ = 0x10B071) with the 26 MHz crystal
- **GFSK, 1.2 kBaud**, deviation 5.2 kHz, RX BW 58 kHz
- sync word **0xD391** (30/32), 4 preamble bytes, channel 0
- PATABLE[0] = **0x60 = 0 dBm**
- packet mode (`cc1101_cfg_packet_mode()`): PKTCTRL0 = 0x05 (variable length,
  CRC on, no whitening), PKTLEN = 61, PKTCTRL1 append status (RSSI, LQI/CRC_OK),
  MCSM1 RX/TX -> IDLE, IOCFG0 = 0x06 (GDO0 high sync->end of packet; its falling
  edge = packet done), IOCFG2 = 0x07 (CRC OK packet in RX FIFO)
- max payload 61 bytes; RSSI offset 74 dB
- The MSP430 test app (`reference/msp430/cc1101_radio/main.c`) sends one
  packet/s: 16-bit sequence number + "hello 433". Good first interop test:
  ESP8684 in RX receiving that.

User preference: register config starts from **SmartRF Studio defaults**,
not the old EasyLink values (those were 868 MHz on a 433 MHz module).

## Reference material in this repo

- `reference/msp430/cc1101_radio/` - working MSP430 CC1101 driver (SPI access
  layer, strobes, burst, status-register errata read, reset, config +
  readback check, SCAL, packet TX/RX) and its test app. Port its structure
  and behaviour; replace the MSP430 eUSCI/GPIO calls with ESP-IDF
  `spi_master` / `gpio`.
- `reference/msp430/logger/console_protocol.md` - the logger's console and
  machine-mode protocol (firmware 2.0), used by msp430_dashboard.

Copies (snapshot 2026-11-23); originals live on the desktop PC in
`C:\Users\joaog\workspace_v12\{cc1101_radio,driver_development,msp430_dashboard}`.

## Documents to use

- ESP8684 datasheet, ESP8684 Technical Reference Manual (GPIO, IO MUX /
  GPIO matrix, GP-SPI2)
- ESP-IDF Programming Guide **v5.2, ESP32-C2** (spi_master, gpio, ISR)
- TI CC1101 datasheet SWRS061 (sections 10 SPI, 17.3 RSSI, 19.1 reset,
  26 GDO, 29 registers), errata SWRZ020

## Toolchain

- ESP-IDF **v5.2.1** (the version installed on the desktop PC; keep the
  laptop on the same version), `idf.py set-target esp32c2`
- Build / flash / monitor: `idf.py build`, `idf.py -p COMx flash monitor`
- **Laptop install**: IDF in `C:\Espressif\esp-idf-v5.2.1`, tools in
  `C:\Espressif\tools` (Python 3.14 venv). Per shell:
  `$env:IDF_TOOLS_PATH='C:\Espressif\tools'; $env:CCACHE_DIR='C:\Espressif\ccache'; . C:\Espressif\esp-idf-v5.2.1\export.ps1`
- **IDF 5.2.1 cannot build from a path with non-ASCII characters** (the
  laptop user folder is `C:\Users\João Lopes`): kconfgen, ccache and ldgen
  each fail on the "ã". subst drives and junctions do not help (idf.py
  resolves them back). Keep the project at an ASCII-only path.
- No native USB on the ESP8684: flashing goes through the board's USB-UART
  (CH340C; Windows may need the WCH CH340 driver).
- Layout: `components/cc1101/` (driver + register images, port of
  `reference/msp430/cc1101_radio/`), `main/` (test app).

## How the user works (from the MSP430 projects)

- Register-level, well-commented C drivers: file header with STATUS, wiring
  table, datasheet section references; every chip wait has a timeout
  (a missing module shows up as a timeout, never a hang).
- Build in small steps, each one testable on its own (SPI access + reset ->
  config + readback -> packet TX/RX -> link test), with clear UART output
  describing what success looks like.
- **The user runs the hardware tests.** Only results they report count as
  verified; say what to look for and ask for the log.
- Keep the code compiling at each step; commit per step.
