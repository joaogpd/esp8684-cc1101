# CLAUDE.md - esp8684-cc1101

Context for Claude sessions continuing this project (handed over from the
desktop PC session of 2026-11-23).

## Goal

Drivers and firmware for an **Espressif ESP8684 (ESP32-C2 die)** driving a
**TI CC1101** sub-GHz transceiver over SPI, so the ESP8684 can talk by radio
to the user's **MSP430FR5994 supercapacitor data logger** ("msp430logger",
project `msp430drivers` / folder `driver_development`).

## Status (2026-11-23)

- Repository skeleton only: README, .gitignore, this file, `reference/`.
- No ESP-IDF project, no driver code yet.
- Next step: confirm the hardware (below), then create the ESP-IDF project
  (`idf.py create-project`, target `esp32c2`), then port the CC1101 SPI
  access layer, then config + packet TX/RX matching the MSP430 side.

## Open questions - ask the user before writing code

1. **Which ESP8684 board?** (e.g. ESP8684-DevKitM-1, DevKitC-02, a bare
   ESP8684-MINI-1 / WROOM-02C module). Needed: free GPIOs, flash size
   (ESP8684H2 = 2 MB, H4 = 4 MB), USB-serial chip (CP210x / CH340), LED and
   BOOT button pins. Ask for the board schematic or a photo of the markings.
2. **Which CC1101 module?** Probably the same generic "CC1101 433 MHz V2.0"
   8-pin board used on the MSP430 (26 MHz crystal, 387-464 MHz matching,
   **3.3 V only, never 5 V**).
3. **Wiring** ESP8684 <-> CC1101 (SCLK, MOSI, MISO/GDO1, CSn, GDO0, GDO2).
   Avoid strapping pins GPIO8 / GPIO9 and the flash pins. Note on the
   C2: CHIP_RDYn polling needs MISO readable as a GPIO while it is the SPI
   input (the MSP430 driver polls SO low after CSn falls).
4. **What goes over the air** - not decided. The logger currently has NO
   radio code; it only has a UART console (9600 8N1, eUSCI_A0). Options:
   - (suggested) ESP8684 = transparent radio <-> serial bridge carrying the
     logger's `@` machine-mode lines, so the msp430_dashboard works over
     radio unchanged. Lines can exceed one packet (61 B payload), so a small
     framing layer (seq, fragment, ack/retry) is needed.
   - a compact binary packet format (samples, history records, commands).
   Either way the MSP430 logger needs a CC1101 link added - that work
   belongs in the MSP430 projects (on the desktop PC), not in this repo.
5. Where the ESP8684 forwards data (USB serial to a PC, Wi-Fi, BLE...).

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
- No native USB on the ESP8684: flashing goes through the board's USB-UART.

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
