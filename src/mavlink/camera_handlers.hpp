#pragma once

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include "common/mavlink.h"
#pragma GCC diagnostic pop

class MavSender;
struct RoverState;

// Handle camera-related REQUEST_MESSAGE sub-commands (msg IDs 259–262)
void handle_camera_request_message(MavSender& mav, const RoverState& state,
                                   const mavlink_command_long_t* cmd);

// Handle camera-related direct COMMAND_LONGs (521, 522, 525)
void handle_camera_command_long(MavSender& mav, const RoverState& state,
                                const mavlink_command_long_t* cmd);
