#pragma once

#include <cstdint>
#include <arpa/inet.h>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include "common/mavlink.h"
#pragma GCC diagnostic pop

#include "gps_fix.hpp"
#include "lte_status.hpp"
#include "camera_info.hpp"

struct RoverState {
    bool        armed           = false;
    uint8_t     base_mode       = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    uint32_t    custom_mode     = 0;  // 0=MANUAL (ArduRover custom_mode value)
    sockaddr_in qgc_addr        = {};
    socklen_t   qgc_addr_len    = sizeof(sockaddr_in);
    bool        qgc_known       = false;

    GpsFix  current_fix{};
    int32_t home_alt_mm = 0;    // altitude at first valid fix, mm MSL
    bool    home_set    = false;

    LteStatus lte{};
    bool      lte_failsafe_active = false;
    bool      lte_was_connected   = false;  // edge detection for drop event

    // Camera catalog — populated by discover_cameras() at startup and used
    // to answer QGC's CAMERA_INFORMATION / VIDEO_STREAM_INFORMATION probes.
    // Actual video delivery is via the embedded RTSP server (pull model),
    // not tied to this state.
    std::vector<CameraInfo> cameras;
    uint32_t video_bitrate_bps = 5000000;    // mirrors VIDEO_BITRATE param
    uint32_t video_fps         = 30;         // mirrors VIDEO_FPS param
};
