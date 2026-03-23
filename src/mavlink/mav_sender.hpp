#pragma once

#include <cstdint>

#include "udp_socket.hpp"
#include "rover_state.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include "common/mavlink.h"
#pragma GCC diagnostic pop

class MavSender {
public:
    MavSender(UdpSocket& sock, uint8_t sys_id, uint8_t comp_id);

    void send_heartbeat(const RoverState& state);
    void send_sys_status(const RoverState& state);
    void send_autopilot_version(const RoverState& state);
    void send_available_modes(const RoverState& state, uint32_t mode);
    void send_command_ack(const RoverState& state, uint16_t command, uint8_t result,
                          uint8_t target_system, uint8_t target_component);
    void send_mission_count(const RoverState& state,
                            uint8_t target_system, uint8_t target_component);
    void send_servo_output_raw(const RoverState& state, int16_t left, int16_t right);
    void send_param(const RoverState& state, const char* name, float value,
                    uint16_t index, uint16_t total);

private:
    void send(mavlink_message_t& msg, const RoverState& state);

    UdpSocket& sock_;
    uint8_t    sys_id_;
    uint8_t    comp_id_;
};
