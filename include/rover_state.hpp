#pragma once

#include <cstdint>
#include <arpa/inet.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include "common/mavlink.h"
#pragma GCC diagnostic pop

struct RoverState {
    bool        armed           = false;
    uint8_t     base_mode       = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    uint32_t    custom_mode     = 0;
    sockaddr_in qgc_addr        = {};
    socklen_t   qgc_addr_len    = sizeof(sockaddr_in);
    bool        qgc_known       = false;
};
