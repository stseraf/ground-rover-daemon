#pragma once

#include <cstdint>
#include <pthread.h>
#include <string>
#include <vector>

struct _GMainContext;
struct _GMainLoop;
struct _GstRTSPServer;

// Embedded RTSP server wrapping gst-rtsp-server. Runs its own GMainLoop on
// a pthread; the daemon's MAVLink loop is untouched.
//
// Pull model: a capture pipeline is constructed on first client connect to
// a given mount and torn down when the last client of that mount disconnects.
// libcamerasrc is therefore only held while someone is actually watching —
// no LTE burn when idle.
//
// Exclusive-use caveat: libcamerasrc can't be opened twice. Each mount has
// its own factory and pipeline, so only one mount can be actively streaming
// at a time. Switching streams = disconnect old, connect new, ~1-2 s for
// the camera to release and reopen.
class RtspServer {
public:
    struct Stream {
        std::string mount;
        uint16_t    width;
        uint16_t    height;
        int         fps;
        uint32_t    bitrate_bps;
    };

    RtspServer();
    ~RtspServer();

    RtspServer(const RtspServer&)            = delete;
    RtspServer& operator=(const RtspServer&) = delete;

    // Starts the server listening on `port`. One factory is registered per
    // stream — each with its own capture+encode pipeline at the requested
    // resolution/fps/bitrate.
    bool start(uint16_t port, const std::vector<Stream>& streams);

    // Signals the GMainLoop to quit and joins the thread. Safe to call from
    // a signal-handler context — uses g_main_loop_quit() which is thread-safe.
    void stop();

private:
    static void* thread_main(void* self);

    _GMainContext*  ctx_    = nullptr;
    _GMainLoop*     loop_   = nullptr;
    _GstRTSPServer* server_ = nullptr;
    pthread_t       thread_{};
    bool            running_ = false;
};
