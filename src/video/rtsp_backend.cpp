#include "rtsp_backend.hpp"

bool RtspBackend::start(const std::vector<VideoStream>& streams)
{
    std::vector<RtspServer::Stream> rs;
    rs.reserve(streams.size());
    for (const auto& s : streams) {
        rs.push_back({s.mount, s.width, s.height, s.fps, s.bitrate_bps});
    }
    return server_.start(port_, rs);
}

void RtspBackend::stop()
{
    server_.stop();
}
