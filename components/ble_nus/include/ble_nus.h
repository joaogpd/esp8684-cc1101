/*
 * ble_nus.h - BLE peripheral with the Nordic UART Service (NUS), NimBLE
 *
 * STATUS: written 2026-11-23, builds; not yet run on hardware.
 *
 * A "serial port over BLE" that phone apps already understand (nRF
 * Connect, Bluefruit Connect UART mode, Serial Bluetooth Terminal):
 *
 *   service  6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 *   RX char  6E400002-...  write / write without response   phone -> ESP
 *   TX char  6E400003-...  notify                            ESP -> phone
 *
 * One connection at a time; advertising restarts after a disconnect.
 * No pairing yet (anyone in range can connect) - add before real use.
 *
 * Reference: ESP-IDF v5.2 NimBLE examples (bleprph), Bluetooth Core spec
 * vol 3 part F (ATT MTU, notifications).
 */
#ifndef BLE_NUS_H_
#define BLE_NUS_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Called from the NimBLE host task for every write to the RX
 * characteristic, with the raw bytes (no line framing: one write may hold
 * part of a line, a whole line or several). Must not block for long.
 */
typedef void (*ble_nus_rx_cb_t)(const uint8_t *data, uint16_t len);

/*
 * Start NimBLE, register the service and advertise as `name` (at most
 * 20 characters, so it fits the advertising packet). Needs nvs_flash_init()
 * first (the BLE controller keeps PHY calibration data in NVS).
 * Returns false if NimBLE or the service could not be set up.
 */
bool ble_nus_init(const char *name, ble_nus_rx_cb_t on_rx);

/* A phone is connected and has enabled notifications on the TX char. */
bool ble_nus_ready(void);

/* A phone is connected (notifications may still be off). */
bool ble_nus_connected(void);

/* Negotiated ATT MTU (23 until the phone asks for more; iPhones ask for
 * 185 or more right after connecting). Payload per notification = MTU-3. */
uint16_t ble_nus_mtu(void);

/*
 * Send bytes as TX notifications, split at MTU-3. Waits up to ~0.5 s per
 * piece when the host is out of buffers. Returns false if no phone is
 * ready or a notification failed. Not thread-safe between callers that
 * need their bytes kept together: serialise them (bridge.c does).
 */
bool ble_nus_send(const uint8_t *data, size_t len);

#endif /* BLE_NUS_H_ */
