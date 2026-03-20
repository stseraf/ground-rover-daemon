#pragma once

#include <cstdint>
#include <cstring>
#include <array>

#include "udp_socket.hpp"
#include "rover_state.hpp"
#include "logger.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include "common/mavlink.h"
#pragma GCC diagnostic pop

#include "config.hpp"

class MavSender {
public:
    MavSender(UdpSocket& sock, uint8_t sys_id, uint8_t comp_id)
        : sock_{sock}, sys_id_{sys_id}, comp_id_{comp_id} {}

    void send_heartbeat(const RoverState& state)
    {
        mavlink_message_t msg;
        uint8_t base_mode = static_cast<uint8_t>(state.base_mode | (state.armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0));
        mavlink_msg_heartbeat_pack(sys_id_, comp_id_, &msg,
            MAV_TYPE_GROUND_ROVER, MAV_AUTOPILOT_GENERIC,
            base_mode, state.custom_mode, MAV_STATE_ACTIVE);
        send(msg, state);
    }

    void send_sys_status(const RoverState& state)
    {
        mavlink_message_t msg;
        mavlink_sys_status_t sys{};
        sys.load              = Config::DUMMY_LOAD_PERMILLE;
        sys.voltage_battery   = Config::DUMMY_BATTERY_MV;
        sys.current_battery   = Config::DUMMY_CURRENT_CA;
        sys.battery_remaining = Config::DUMMY_BATTERY_PCT;
        mavlink_msg_sys_status_encode(sys_id_, comp_id_, &msg, &sys);
        send(msg, state);
    }

    void send_autopilot_version(const RoverState& state)
    {
        mavlink_message_t msg;
        mavlink_autopilot_version_t av{};
        av.capabilities =
            MAV_PROTOCOL_CAPABILITY_MISSION_INT
          | MAV_PROTOCOL_CAPABILITY_COMMAND_INT
          | MAV_PROTOCOL_CAPABILITY_MAVLINK2;
        mavlink_msg_autopilot_version_encode(sys_id_, comp_id_, &msg, &av);
        logger::line("tx: MAVLINK_MSG_ID_AUTOPILOT_VERSION(148): capabilities=0x%016llX",
                     (unsigned long long)av.capabilities);
        send(msg, state);
    }

    void send_available_modes(const RoverState& state, uint32_t mode)
    {
        struct ModeInfo {
            const char *name;
            uint8_t     standard_mode;
            uint32_t    custom_mode;
            uint32_t    properties;
        };
        constexpr std::array<ModeInfo, 2> modes{{
            { "HOLD",   MAV_STANDARD_MODE_POSITION_HOLD, 0, 0 },
            { "MANUAL", MAV_STANDARD_MODE_NON_STANDARD,  0, MAV_MODE_PROPERTY_ADVANCED }
        }};
        constexpr uint8_t number_modes = static_cast<uint8_t>(modes.size());

        mavlink_message_t msg;
        mavlink_available_modes_t am{};
        am.number_modes  = number_modes;
        am.mode_index    = mode;
        am.standard_mode = modes[mode-1].standard_mode;
        am.custom_mode   = modes[mode-1].custom_mode;
        am.properties    = modes[mode-1].properties;
        std::strncpy(am.mode_name, modes[mode-1].name, sizeof(am.mode_name) - 1);
        mavlink_msg_available_modes_encode(sys_id_, comp_id_, &msg, &am);
        logger::line("tx: MAVLINK_MSG_ID_AVAILABLE_MODES(435) mode %u/%u: %s",
                     mode, number_modes, modes[mode-1].name);
        send(msg, state);
    }

    void send_command_ack(const RoverState& state, uint16_t command, uint8_t result,
                          uint8_t target_system, uint8_t target_component)
    {
        mavlink_message_t msg;
        mavlink_msg_command_ack_pack(sys_id_, comp_id_, &msg,
            command, result,
            0, /* progress */
            0, /* result_param2 */
            target_system, target_component);
        logger::line("tx: MAVLINK_MSG_ID_COMMAND_ACK(77): command=%u result=%u", command, result);
        send(msg, state);
    }

    void send_mission_count(const RoverState& state,
                            uint8_t target_system, uint8_t target_component)
    {
        mavlink_message_t msg;
        mavlink_msg_mission_count_pack(sys_id_, comp_id_, &msg,
            target_system, target_component, 0, MAV_MISSION_TYPE_MISSION, 0);
        logger::line("tx: MAVLINK_MSG_ID_MISSION_COUNT(44): %u", 0);
        send(msg, state);
    }

    // left/right in [-1000, 1000]; converted to PWM µs [1000, 2000]
    void send_servo_output_raw(const RoverState& state, int16_t left, int16_t right)
    {
        auto to_pwm = [](int16_t v) -> uint16_t {
            return static_cast<uint16_t>(Config::DRIVE_PWM_CENTER +
                   (static_cast<int32_t>(v) * Config::DRIVE_PWM_HALF_RANGE) / Config::DRIVE_AXIS_MAX);
        };
        const uint16_t pwm_left  = to_pwm(left);
        const uint16_t pwm_right = to_pwm(right);
        const uint16_t neutral   = Config::DRIVE_PWM_CENTER;
        mavlink_message_t msg;
        mavlink_msg_servo_output_raw_pack(sys_id_, comp_id_, &msg,
            0,           /* time_usec */
            0,           /* port: MAIN */
            pwm_left, pwm_right,
            neutral, neutral, neutral, neutral, neutral, neutral,
            neutral, neutral, neutral, neutral, neutral, neutral, neutral, neutral);
        send(msg, state);
    }

    void send_param(const RoverState& state, const char *name, float value,
                    uint16_t index, uint16_t total)
    {
        mavlink_message_t msg;
        mavlink_msg_param_value_pack(sys_id_, comp_id_, &msg,
            name, value, MAV_PARAM_TYPE_REAL32, total, index);
        logger::line("tx: MAVLINK_MSG_ID_PARAM_VALUE(22): %s=%.2f (%u/%u)",
                     name, value, index + 1, total);
        send(msg, state);
    }

private:
    void send(mavlink_message_t& msg, const RoverState& state)
    {
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        int len = mavlink_msg_to_send_buffer(buf, &msg);
        sock_.send_to(buf, static_cast<size_t>(len), state.qgc_addr, state.qgc_addr_len);
    }

    UdpSocket& sock_;
    uint8_t    sys_id_;
    uint8_t    comp_id_;
};
