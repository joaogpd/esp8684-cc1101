/*
 * bridge.h - line router between USB serial (UART0 / CH340C) and BLE (NUS)
 *
 * STATUS: step 1 of the bridge (2026-11-23): lines from the phone or the
 * PC are echoed back, `!` lines are answered by the ESP8684 itself, and
 * received radio packets are shown on both. The radio side of the bridge
 * (commands to the logger, `@` lines back, framing) comes next.
 *
 * Lines end with CR, LF or CR LF. A write that ends without one (phone
 * apps often send "status" with no line ending) counts as a whole line
 * after 300 ms with no more bytes. Lines longer than 120 characters are
 * dropped with "!err,toolong" (logger commands are at most 48).
 *
 * Local commands (never sent to the logger):
 *   !help   list of local commands
 *   !link   BLE and radio state, one line:
 *           !link,ble=<0|1>,mtu=<n>,radio=<ok|down>,rx=<n>,bad=<n>,
 *                 rssi=<dBm>,lqi=<n>,age=<s since last good packet, -1 none>
 */
#ifndef BRIDGE_H_
#define BRIDGE_H_

#include <stdbool.h>
#include <stdint.h>

/* UART0 input driver, BLE (advertising name `ble_name`), router task.
 * Needs nvs_flash_init() first. */
void bridge_init(const char *ble_name);

/* One line to the PC and, if a phone is listening, to the phone.
 * No line ending in `line`; CR LF is added. Any task. */
void bridge_out(const char *line);

/* Radio state for !link, set by the radio task */
void bridge_note_radio(bool up);
void bridge_note_rx(bool crc_ok, int16_t rssi_dbm, uint8_t lqi);

#endif /* BRIDGE_H_ */
