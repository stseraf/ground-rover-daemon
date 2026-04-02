#pragma once

#include <cstdint>

struct LteStatus {
    bool    present   = false;  // usb0 interface is up
    bool    connected = false;  // modem API reports "Connected"
    uint8_t rssi      = 0;      // 0–31 CSQ scale (AT+CSQ style)
    char    netmode[8]{};       // "LTE", "3G", "2G", …
    char    oper[24]{};         // operator name e.g. "KYIVSTAR"
};
