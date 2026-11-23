# esp8684-cc1101

Drivers for a TI CC1101 sub-GHz transceiver on an Espressif ESP8684
(ESP32-C2), to talk to the MSP430FR5994 logger (msp430drivers /
cc1101_radio projects).

## Toolchain

- ESP-IDF **v5.2.1**, target `esp32c2`
  (Windows: Espressif IDF installer, then the "ESP-IDF 5.2" terminal)
- USB-serial driver for the board's bridge chip (CP210x or CH340)

```
idf.py set-target esp32c2
idf.py build
idf.py -p COMx flash monitor
```

## Radio settings (must match the MSP430 side)

433.92 MHz, GFSK 1.2 kBaud, sync word 0xD391, 0 dBm - the register image
from `cc1101_radio/drivers/devices/cc1101_settings.c`.

## For Claude sessions

See `CLAUDE.md` (project context, open questions, radio settings) and
`reference/` (MSP430-side CC1101 driver and logger protocol).

## Reference documents

- ESP8684 datasheet and Technical Reference Manual (GPIO, IO MUX, GP-SPI)
- ESP-IDF Programming Guide for ESP32-C2, v5.2 (spi_master, gpio)
- TI CC1101 datasheet (SWRS061)
