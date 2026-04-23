// Stub build — compiled when VIDEO=none, avoids the gstreamer-rtsp-server
// dependency for quick local sanity-check builds on hosts without the
// cross-arch libgstrtspserver dev package installed. Daemon still starts,
// MAVLink works, video simply isn't served.
#include "video/rtsp_server.hpp"

#include "logger.hpp"

RtspServer::RtspServer() {}
RtspServer::~RtspServer() {}

bool RtspServer::start(uint16_t port, const std::vector<Stream>& streams)
{
    logger::line("[rtsp] stub build (VIDEO=none) — %zu stream(s) ignored on :%u",
                 streams.size(), port);
    return false;
}

void RtspServer::stop() {}

void* RtspServer::thread_main(void*) { return nullptr; }
