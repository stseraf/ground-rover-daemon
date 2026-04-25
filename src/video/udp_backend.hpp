#pragma once

#include <sys/types.h>
#include <cstdint>

#include "video_backend.hpp"

struct RoverState;

// UDP-push transport: spawns a gst-launch subprocess that pushes RTP/H.264
// to udp://<learned-qgc-ip>:5600. One pipeline runs at a time; switching
// streams = kill + respawn for the new sensor mode (~1-2 s for libcamera
// release/reopen).
//
// QGC IP is read from state.qgc_addr each time a pipeline is spawned,
// so the IP-learning happens transparently in the main MAVLink recv loop
// (state.qgc_addr is filled by recvfrom).
class UdpBackend : public VideoBackend {
public:
    UdpBackend(const RoverState& state, uint16_t udp_port)
        : state_(state), port_(udp_port) {}

    ~UdpBackend() override { stop(); }

    bool start(const std::vector<VideoStream>& streams) override;
    void stop() override;

    void start_stream(uint8_t stream_id) override;
    void stop_stream() override;
    void tick() override;

private:
    void   spawn_pipeline(const VideoStream& s);
    void   kill_pipeline();
    static void resolve_qgc_ip(const RoverState& state, char* out, size_t out_len);

    const RoverState&       state_;
    uint16_t                port_;
    std::vector<VideoStream> streams_;
    pid_t                   pid_         = -1;
    uint8_t                 active_id_   = 0;  // 1-indexed; 0 = no active stream
};
