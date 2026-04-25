#include "udp_backend.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>

#include "rover_state.hpp"
#include "logger.hpp"

void UdpBackend::resolve_qgc_ip(const RoverState& state, char* out, size_t out_len)
{
    out[0] = '\0';
    if (!state.qgc_known) return;
    inet_ntop(AF_INET, &state.qgc_addr.sin_addr, out, static_cast<socklen_t>(out_len));
}

bool UdpBackend::start(const std::vector<VideoStream>& streams)
{
    streams_   = streams;
    pid_       = -1;
    active_id_ = 0;
    logger::line("[udp] backend ready (%zu stream(s)) — awaiting QGC start",
                 streams_.size());
    return true;
}

void UdpBackend::stop()
{
    kill_pipeline();
    streams_.clear();
}

void UdpBackend::start_stream(uint8_t stream_id)
{
    if (stream_id < 1 || stream_id > streams_.size()) {
        logger::line("[udp] start_stream: invalid stream_id %u (have %zu)",
                     stream_id, streams_.size());
        return;
    }
    if (pid_ > 0 && active_id_ == stream_id) return;     // already running
    if (pid_ > 0) kill_pipeline();
    spawn_pipeline(streams_[stream_id - 1]);
    if (pid_ > 0) active_id_ = stream_id;
}

void UdpBackend::stop_stream()
{
    if (pid_ > 0) kill_pipeline();
    active_id_ = 0;
}

void UdpBackend::tick()
{
    if (pid_ <= 0) return;
    int status = 0;
    pid_t r = ::waitpid(pid_, &status, WNOHANG);
    if (r == 0) return;                  // still running
    // Exited unexpectedly — respawn the same stream.
    logger::line("[udp] gst-launch (pid %d) exited — respawning", pid_);
    pid_t old_pid = pid_;
    (void)old_pid;
    pid_ = -1;
    if (active_id_ >= 1 && active_id_ <= streams_.size())
        spawn_pipeline(streams_[active_id_ - 1]);
}

void UdpBackend::spawn_pipeline(const VideoStream& s)
{
    char qgc_ip[INET_ADDRSTRLEN];
    resolve_qgc_ip(state_, qgc_ip, sizeof(qgc_ip));
    if (qgc_ip[0] == '\0') {
        logger::line("[udp] QGC IP not yet learned — pipeline deferred");
        return;
    }

    // rtph264pay config-interval=1: SPS/PPS roughly every 1 s — same value
    // the original UDP-push impl shipped with; gives QGC's mid-stream join
    // a recovery window without bloating every IDR.
    char cmd[640];
    std::snprintf(cmd, sizeof(cmd),
        "gst-launch-1.0 libcamerasrc ! "
        "video/x-raw,width=%u,height=%u,framerate=%d/1 ! "
        "v4l2h264enc extra-controls=\"controls,repeat_sequence_header=1,"
        "video_bitrate=%u\" ! "
        "'video/x-h264,level=(string)4' ! "
        "h264parse config-interval=-1 ! "
        "rtph264pay config-interval=1 pt=96 mtu=1400 ! "
        "udpsink host=%s port=%u sync=false",
        s.width, s.height, s.fps, s.bitrate_bps, qgc_ip, port_);

    logger::line("[udp] spawn: %s", cmd);

    pid_t pid = ::fork();
    if (pid < 0) {
        logger::line("[udp] fork failed: %s", std::strerror(errno));
        return;
    }
    if (pid == 0) {
        // New process group so killpg reaches gst-launch under sh.
        ::setpgrp();
        int devnull = ::open("/dev/null", O_WRONLY);
        if (devnull >= 0) {
            ::dup2(devnull, STDOUT_FILENO);
            ::dup2(devnull, STDERR_FILENO);
            ::close(devnull);
        }
        ::execl("/bin/sh", "sh", "-c", cmd, nullptr);
        ::_exit(127);
    }
    pid_ = pid;
    logger::line("[udp] pid %d started → %s:%u %ux%u@%dfps %ukbps",
                 pid_, qgc_ip, port_, s.width, s.height, s.fps,
                 s.bitrate_bps / 1000);
}

void UdpBackend::kill_pipeline()
{
    if (pid_ <= 0) { pid_ = -1; return; }

    ::killpg(pid_, SIGTERM);
    ::usleep(500'000);

    int status = 0;
    if (::waitpid(pid_, &status, WNOHANG) == pid_) {
        logger::line("[udp] pid %d exited cleanly", pid_);
        pid_ = -1;
        return;
    }

    logger::line("[udp] pid %d still alive — SIGKILL", pid_);
    ::killpg(pid_, SIGKILL);
    ::waitpid(pid_, nullptr, 0);
    pid_ = -1;
}
