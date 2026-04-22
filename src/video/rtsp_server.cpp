#include "video/rtsp_server.hpp"

#include <cstdio>
#include <cstring>

#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>

#include "logger.hpp"

RtspServer::RtspServer() {}

RtspServer::~RtspServer()
{
    stop();
}

bool RtspServer::start(uint16_t port, const char* mount,
                       uint16_t width, uint16_t height,
                       int fps, uint32_t bitrate_bps)
{
    if (running_) return false;

    // gst_init is idempotent — safe to call on every start(); cheap
    // after the first invocation (scans the plugin registry once).
    gst_init(nullptr, nullptr);

    // Pipeline: libcamerasrc → v4l2h264enc → rtph264pay(name=pay0).
    // "pay0" is how gst-rtsp-server locates the RTP payload output.
    char launch[640];
    std::snprintf(launch, sizeof(launch),
        "( libcamerasrc ! "
        "video/x-raw,width=%u,height=%u,framerate=%d/1 ! "
        "v4l2h264enc extra-controls=\"controls,repeat_sequence_header=1,"
        "video_bitrate=%u\" ! "
        "video/x-h264,level=(string)4 ! "
        "h264parse config-interval=-1 ! "
        "rtph264pay config-interval=1 pt=96 mtu=1400 name=pay0 )",
        width, height, fps, bitrate_bps);

    server_ = gst_rtsp_server_new();
    char port_str[8];
    std::snprintf(port_str, sizeof(port_str), "%u", port);
    gst_rtsp_server_set_service(server_, port_str);

    GstRTSPMountPoints*   mounts  = gst_rtsp_server_get_mount_points(server_);
    GstRTSPMediaFactory*  factory = gst_rtsp_media_factory_new();
    gst_rtsp_media_factory_set_launch(factory, launch);
    // Shared: one encoder feeds all clients. Required because libcamerasrc
    // is exclusive-access.
    gst_rtsp_media_factory_set_shared(factory, TRUE);
    // No server-side jitter buffering — we want the lowest possible latency
    // and the client can do its own jitter handling if it wants.
    gst_rtsp_media_factory_set_latency(factory, 0);
    gst_rtsp_mount_points_add_factory(mounts, mount, factory);
    g_object_unref(mounts);

    if (gst_rtsp_server_attach(server_, nullptr) == 0) {
        logger::line("[rtsp] failed to attach server on port %u", port);
        g_object_unref(server_);
        server_ = nullptr;
        return false;
    }

    loop_ = g_main_loop_new(nullptr, FALSE);
    if (pthread_create(&thread_, nullptr, &RtspServer::thread_main, this) != 0) {
        logger::line("[rtsp] pthread_create failed");
        g_main_loop_unref(loop_);
        loop_ = nullptr;
        g_object_unref(server_);
        server_ = nullptr;
        return false;
    }

    running_ = true;
    logger::line("[rtsp] listening on rtsp://<rover>:%u%s (%ux%u @%dfps %ukbps)",
                 port, mount, width, height, fps, bitrate_bps / 1000);
    return true;
}

void RtspServer::stop()
{
    if (!running_) return;
    running_ = false;

    if (loop_) g_main_loop_quit(loop_);
    pthread_join(thread_, nullptr);

    if (loop_)   { g_main_loop_unref(loop_);       loop_   = nullptr; }
    if (server_) { g_object_unref(server_);        server_ = nullptr; }
    logger::line("[rtsp] stopped");
}

void* RtspServer::thread_main(void* self_v)
{
    auto* self = static_cast<RtspServer*>(self_v);
    g_main_loop_run(self->loop_);
    return nullptr;
}
