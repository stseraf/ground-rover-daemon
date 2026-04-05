#pragma once

#include <cstdint>

struct LteStatus {
    bool     present   = false;  // usb0 interface is up
    bool     connected = false;  // modem status daemon reports connected
    uint8_t  rssi      = 0;      // 0–31 CSQ scale (AT+CSQ style)
    char     netmode[8]{};       // "LTE", "UMTS", "EDGE", …
    char     oper[24]{};         // operator name e.g. "KYIVSTAR"
    uint64_t rx_bytes  = 0;      // cumulative RX bytes on usb0 since Pi boot
    uint64_t tx_bytes  = 0;      // cumulative TX bytes on usb0 since Pi boot
};
