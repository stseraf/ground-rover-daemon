#pragma once

#include <cstdint>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include "common/mavlink.h"
#pragma GCC diagnostic pop

class MavSender;
struct RoverState;

// Handle camera REQUEST_MESSAGE sub-commands (msg IDs 259, 269, 270, ...)
// targeted at camera component cam_idx.
void handle_camera_request_message(MavSender& mav, const RoverState& state,
                                   const mavlink_command_long_t* cmd,
                                   int cam_idx);

// Handle COMMAND_LONGs targeted at camera component cam_idx.
// Includes: MAV_CMD_REQUEST_MESSAGE, start/stop streaming, deprecated camera cmds.
// Video delivery is via the embedded RTSP server (pull model), so start/stop
// are accepted but have no side effects on the pipeline.
void handle_camera_command_long(MavSender& mav, const RoverState& state,
                                const mavlink_command_long_t* cmd,
                                int cam_idx);
