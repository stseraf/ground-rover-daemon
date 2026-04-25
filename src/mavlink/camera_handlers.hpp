#pragma once

#include <cstdint>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include "common/mavlink.h"
#pragma GCC diagnostic pop

class MavSender;
class VideoBackend;
struct RoverState;

// Handle camera REQUEST_MESSAGE sub-commands (msg IDs 259, 269, 270, ...)
// targeted at camera component cam_idx.
void handle_camera_request_message(MavSender& mav, const RoverState& state,
                                   const mavlink_command_long_t* cmd,
                                   int cam_idx);

// Handle COMMAND_LONGs targeted at camera component cam_idx.
// Includes: MAV_CMD_REQUEST_MESSAGE, start/stop streaming, deprecated camera cmds.
// VIDEO_START/STOP_STREAMING are forwarded to the active VideoBackend —
// RTSP backend treats them as no-ops (pipeline is client-driven), UDP
// backend uses them to spawn/kill the gst-launch subprocess for the
// requested stream_id.
void handle_camera_command_long(MavSender& mav, RoverState& state,
                                VideoBackend* backend,
                                const mavlink_command_long_t* cmd,
                                int cam_idx);
