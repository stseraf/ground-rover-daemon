#include "camera/gst_pipeline.hpp"

#include <cstdio>
#include <cstring>
#include <cerrno>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/wait.h>

#include "logger.hpp"

pid_t gst_spawn(uint16_t width, uint16_t height, int fps,
                uint32_t bitrate_bps, const char* qgc_ip)
{
    char cmd[640];
    std::snprintf(cmd, sizeof(cmd),
        "gst-launch-1.0 libcamerasrc ! "
        "video/x-raw,width=%u,height=%u,framerate=%d/1 ! "
        "v4l2h264enc extra-controls=\"controls,repeat_sequence_header=1,"
        "video_bitrate=%u\" ! "
        "'video/x-h264,level=(string)4' ! "
        "h264parse config-interval=-1 ! "
        "rtph264pay config-interval=1 pt=96 mtu=1400 ! "
        "udpsink host=%s port=5600 sync=false",
        width, height, fps, bitrate_bps, qgc_ip);

    logger::line("[gst] spawn: %s", cmd);

    pid_t pid = fork();
    if (pid < 0) {
        logger::line("[gst] fork failed: %s", std::strerror(errno));
        return -1;
    }

    if (pid == 0) {
        // Child: new process group so killpg reaches gst-launch too
        setpgrp();

        // Redirect stdout/stderr to /dev/null — gst-launch is verbose
        int devnull = open("/dev/null", O_WRONLY);
        if (devnull >= 0) {
            dup2(devnull, STDOUT_FILENO);
            dup2(devnull, STDERR_FILENO);
            close(devnull);
        }

        execl("/bin/sh", "sh", "-c", cmd, nullptr);
        _exit(127);
    }

    logger::line("[gst] PID %d started (%ux%u %dfps %ubps → %s:5600)",
                 pid, width, height, fps, bitrate_bps, qgc_ip);
    return pid;
}

void gst_kill(pid_t pid)
{
    if (pid <= 0) return;

    // Kill the entire process group (sh + gst-launch child)
    killpg(pid, SIGTERM);

    // Give gst-launch up to 500ms to flush and exit cleanly
    usleep(500'000);

    int status;
    if (waitpid(pid, &status, WNOHANG) == pid) {
        logger::line("[gst] PID %d exited cleanly", pid);
        return;
    }

    logger::line("[gst] PID %d still alive — sending SIGKILL", pid);
    killpg(pid, SIGKILL);
    waitpid(pid, nullptr, 0);
    logger::line("[gst] PID %d killed", pid);
}
