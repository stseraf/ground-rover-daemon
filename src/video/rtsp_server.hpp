#pragma once

#include <cstdint>
#include <pthread.h>

struct _GMainLoop;
struct _GstRTSPServer;

// Embedded RTSP server wrapping gst-rtsp-server. Runs its own GMainLoop on
// a pthread; the daemon's MAVLink loop is untouched.
//
// Pull model: the capture pipeline is constructed on first client connect
// and torn down when the last client disconnects. libcamerasrc is therefore
// only held while someone is actually watching — no LTE burn when idle.
//
// Exclusive-use caveat: libcamerasrc can't be opened twice, so set_shared(TRUE)
// gives every connected client frames from the same encoder.
class RtspServer {
public:
    RtspServer();
    ~RtspServer();

    RtspServer(const RtspServer&)            = delete;
    RtspServer& operator=(const RtspServer&) = delete;

    // Starts the server listening on port:mount. Pipeline params are fixed
    // for the lifetime of the server — changes to VIDEO_BITRATE / VIDEO_FPS
    // take effect on daemon restart. Returns true on success.
    bool start(uint16_t port, const char* mount,
               uint16_t width, uint16_t height,
               int fps, uint32_t bitrate_bps);

    // Signals the GMainLoop to quit and joins the thread. Safe to call from
    // a signal handler context — uses g_main_loop_quit() which is thread-safe.
    void stop();

private:
    static void* thread_main(void* self);

    _GMainLoop*     loop_   = nullptr;
    _GstRTSPServer* server_ = nullptr;
    pthread_t       thread_{};
    bool            running_ = false;
};
