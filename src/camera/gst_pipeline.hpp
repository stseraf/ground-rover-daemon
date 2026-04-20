#pragma once

#include <cstdint>
#include <sys/types.h>

// Spawn a GStreamer RTP/H.264 UDP pipeline.
// Returns the PID of the spawned sh process, or -1 on failure.
pid_t gst_spawn(uint16_t width, uint16_t height, int fps,
                uint32_t bitrate_bps, const char* qgc_ip);

// Send SIGTERM to the pipeline's process group; SIGKILL after 500ms if still alive.
void gst_kill(pid_t pid);
