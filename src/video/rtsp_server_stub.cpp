// Stub build — compiled when VIDEO=none, avoids the gstreamer-rtsp-server
// dependency for quick local sanity-check builds on hosts without the
// cross-arch libgstrtspserver dev package installed. Daemon still starts,
// MAVLink works, video simply isn't served.
#include "video/rtsp_server.hpp"

#include "logger.hpp"

RtspServer::RtspServer() {}
RtspServer::~RtspServer() {}

bool RtspServer::start(uint16_t port, const char* mount,
                       uint16_t /*width*/, uint16_t /*height*/,
                       int /*fps*/, uint32_t /*bitrate_bps*/)
{
    logger::line("[rtsp] stub build (VIDEO=none) — no video served on :%u%s",
                 port, mount);
    return false;
}

void RtspServer::stop() {}

void* RtspServer::thread_main(void*) { return nullptr; }
