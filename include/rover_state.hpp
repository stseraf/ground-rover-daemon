#pragma once

#include <array>
#include <cstdint>
#include <arpa/inet.h>
#include <vector>

#include "config.hpp"

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
    // Per-mode H.264 target bitrate (bps), index = sensor mode index.
    // Mirrors the VIDEO_BITRATE_1..N params; refreshed at startup and on
    // every PARAM_SET that changes a video-bitrate param.
    std::array<uint32_t, Config::MAX_VIDEO_BITRATE_PARAMS> per_mode_bitrate_bps{};
    uint32_t video_fps         = 30;         // mirrors VIDEO_FPS param

    // Video transport selected at runtime via VIDEO_TRANSPORT param:
    //   0 = RTSP server (clients pull from rtsp://<rover>:8554/stream-N)
    //   1 = UDP push    (rover sends RTP/H.264 to udp://<qgc>:5600)
    // Mirrored from the param so MAVLink advertisement (URI/type) and
    // stream-start commands can branch without re-reading the store.
    uint8_t  video_transport   = 0;
};
