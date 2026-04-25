#pragma once

#include <cstdint>
#include <string>
#include <vector>

// Common stream descriptor used by both transport backends.
struct VideoStream {
    std::string mount;        // RTSP only — UDP backend ignores this
    uint16_t    width;
    uint16_t    height;
    int         fps;
    uint32_t    bitrate_bps;
};

// Two transport backends:
//  - RTSP (pull): gst-rtsp-server, clients connect to rtsp://<rover>:8554/...
//                 lower latency but smaller QGC-side jitter buffer → motion
//                 artifacts surface more easily on lossy links.
//  - UDP push:    gst-launch sends RTP/H.264 to udp://<qgc>:5600.
//                 higher latency (QGC's raw-UDP renderer buffers ~200 ms),
//                 hides packet loss better — historically more stable picture.
//
// Selected at runtime via the VIDEO_TRANSPORT MAVLink param. Switching
// stops the current backend before constructing the new one.
class VideoBackend {
public:
    virtual ~VideoBackend() = default;

    // Stand the backend up. RTSP: register all mounts. UDP: cache stream
    // catalog and wait for start_stream() (autostart on first QGC contact
    // is driven by main.cpp).
    virtual bool start(const std::vector<VideoStream>& streams) = 0;

    // Tear everything down. Idempotent.
    virtual void stop() = 0;

    // Activate stream_id (1-indexed). RTSP: no-op (the client picks the
    // mount via URL). UDP: kill any running pipeline, spawn a new one for
    // the matching sensor mode.
    virtual void start_stream(uint8_t stream_id) = 0;

    // Stop the active stream. RTSP: no-op. UDP: kill pipeline.
    virtual void stop_stream() = 0;

    // Per-loop tick. RTSP: no-op. UDP: watchdog (respawns gst-launch if
    // it exited unexpectedly).
    virtual void tick() = 0;
};
