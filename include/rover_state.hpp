#pragma once

#include <cstdint>
#include <arpa/inet.h>
#include <sys/types.h>
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

    // Camera / video streaming state
    std::vector<CameraInfo> cameras;         // populated at startup by discover_cameras()
    int     active_cam_idx  = -1;            // -1 = no stream running
    int     active_mode_idx = -1;
    pid_t   active_gst_pid  = -1;
    char    qgc_ip[INET6_ADDRSTRLEN]{};      // extracted from qgc_addr on first packet
    bool    qgc_ip_known    = false;
    uint32_t video_bitrate_bps = 5000000;    // mirrors VIDEO_BITRATE param
    uint32_t video_fps         = 30;         // mirrors VIDEO_FPS param

    // Stream health monitor / auto-retry state
    int  gst_retry_count = 0;    // consecutive restart attempts on current mode
    bool gst_gave_up     = false; // true after exhausting all retries; cleared on next deliberate start
};
