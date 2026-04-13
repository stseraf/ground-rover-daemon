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
// state is non-const because start/stop streaming mutates active stream fields.
void handle_camera_command_long(MavSender& mav, RoverState& state,
                                const mavlink_command_long_t* cmd,
                                int cam_idx);

// Periodic liveness check — call every Config::GST_MONITOR_INTERVAL_US from the main loop.
// Checks whether active_gst_pid is still alive and applies the retry/fallback policy on death.
void gst_monitor_tick(MavSender& mav, RoverState& state, uint64_t now);
