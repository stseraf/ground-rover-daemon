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

bool RtspServer::start(uint16_t port, const std::vector<Stream>& streams)
{
    if (running_) return false;
    if (streams.empty()) {
        logger::line("[rtsp] no streams configured — server not started");
        return false;
    }

    // gst_init is idempotent — safe to call on every start(); cheap
    // after the first invocation (scans the plugin registry once).
    gst_init(nullptr, nullptr);

    // Dedicated context so stop() can fully destroy it and release the
    // listen socket immediately — avoids EADDRINUSE on restart.
    ctx_ = g_main_context_new();

    server_ = gst_rtsp_server_new();
    char port_str[8];
    std::snprintf(port_str, sizeof(port_str), "%u", port);
    gst_rtsp_server_set_service(server_, port_str);

    GstRTSPMountPoints* mounts = gst_rtsp_server_get_mount_points(server_);
    for (const auto& s : streams) {
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
            "rtph264pay config-interval=1 pt=96 mtu=1200 name=pay0 )",
            s.width, s.height, s.fps, s.bitrate_bps);

        GstRTSPMediaFactory* factory = gst_rtsp_media_factory_new();
        gst_rtsp_media_factory_set_launch(factory, launch);
        // Shared: one encoder feeds all clients OF THIS MOUNT. Required
        // because libcamerasrc is exclusive-access — cross-mount switching
        // is serialised (old mount releases camera before new one opens).
        gst_rtsp_media_factory_set_shared(factory, TRUE);
        // No server-side jitter buffering — we want the lowest possible
        // latency and the client can do its own jitter handling if it wants.
        gst_rtsp_media_factory_set_latency(factory, 0);
        gst_rtsp_mount_points_add_factory(mounts, s.mount.c_str(), factory);

        logger::line("[rtsp]   mount %s → %ux%u @%dfps %ukbps",
                     s.mount.c_str(), s.width, s.height, s.fps, s.bitrate_bps / 1000);
    }
    g_object_unref(mounts);

    if (gst_rtsp_server_attach(server_, ctx_) == 0) {
        logger::line("[rtsp] failed to attach server on port %u", port);
        g_object_unref(server_);
        server_ = nullptr;
        g_main_context_unref(ctx_);
        ctx_ = nullptr;
        return false;
    }

    loop_ = g_main_loop_new(ctx_, FALSE);
    if (pthread_create(&thread_, nullptr, &RtspServer::thread_main, this) != 0) {
        logger::line("[rtsp] pthread_create failed");
        g_main_loop_unref(loop_);   loop_   = nullptr;
        g_object_unref(server_);    server_ = nullptr;
        g_main_context_unref(ctx_); ctx_    = nullptr;
        return false;
    }

    running_ = true;
    logger::line("[rtsp] listening on port %u (%zu stream(s))", port, streams.size());
    return true;
}

void RtspServer::stop()
{
    if (!running_) return;
    running_ = false;

    if (loop_) g_main_loop_quit(loop_);
    pthread_join(thread_, nullptr);

    if (loop_)   { g_main_loop_unref(loop_);   loop_   = nullptr; }
    if (server_) { g_object_unref(server_);    server_ = nullptr; }
    if (ctx_)    { g_main_context_unref(ctx_); ctx_    = nullptr; }
    logger::line("[rtsp] stopped");
}

void* RtspServer::thread_main(void* self_v)
{
    auto* self = static_cast<RtspServer*>(self_v);
    g_main_loop_run(self->loop_);
    return nullptr;
}

static GstRTSPFilterResult drop_client(GstRTSPServer*, GstRTSPClient*, gpointer)
{
    return GST_RTSP_FILTER_REMOVE;
}

void RtspServer::disconnect_all_clients()
{
    if (!server_) return;
    gst_rtsp_server_client_filter(server_, drop_client, nullptr);
    logger::line("[rtsp] disconnected all clients");
}
