#pragma once

#include "video_backend.hpp"
#include "rtsp_server.hpp"

class RtspBackend : public VideoBackend {
public:
    explicit RtspBackend(uint16_t port) : port_(port) {}

    bool start(const std::vector<VideoStream>& streams) override;
    void stop() override;

    // RTSP is pull-driven; QGC's gear dropdown switches mounts by URL.
    // start_stream is a no-op — the pipeline lifecycle is bound to
    // client connect/disconnect inside the server. stop_stream
    // force-disconnects all clients to release the camera (used by
    // RESET_CAMERA_SETTINGS to actually halt streaming).
    void start_stream(uint8_t /*stream_id*/) override {}
    void stop_stream() override { server_.disconnect_all_clients(); }
    void tick() override {}

private:
    uint16_t   port_;
    RtspServer server_;
};
