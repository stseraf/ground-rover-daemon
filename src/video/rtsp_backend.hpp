#pragma once

#include "video_backend.hpp"
#include "rtsp_server.hpp"

class RtspBackend : public VideoBackend {
public:
    explicit RtspBackend(uint16_t port) : port_(port) {}

    bool start(const std::vector<VideoStream>& streams) override;
    void stop() override;

    // RTSP is pull-driven; QGC's gear dropdown switches between mounts
    // by URL. start_stream / stop_stream are no-ops here — the pipeline
    // lifecycle is bound to client connect/disconnect inside the server.
    void start_stream(uint8_t /*stream_id*/) override {}
    void stop_stream() override {}
    void tick() override {}

private:
    uint16_t   port_;
    RtspServer server_;
};
