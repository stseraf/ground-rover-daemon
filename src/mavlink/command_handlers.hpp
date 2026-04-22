#pragma once

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include "common/mavlink.h"
#pragma GCC diagnostic pop

class MavSender;
class ParamStore;
struct RoverState;

void handle_command_long(MavSender& mav, RoverState& state, ParamStore& params,
                         const mavlink_command_long_t* cmd);
