#include "mav_sender.hpp"

#include <cstring>
#include <cmath>
#include <array>
#include <climits>

#include "config.hpp"
#include "logger.hpp"

MavSender::MavSender(UdpSocket& sock, uint8_t sys_id, uint8_t comp_id)
    : sock_{sock}, sys_id_{sys_id}, comp_id_{comp_id} {}

void MavSender::send_heartbeat(const RoverState& state)
{
    mavlink_message_t msg;
    uint8_t base_mode = static_cast<uint8_t>(state.base_mode | (state.armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0));
    mavlink_msg_heartbeat_pack(sys_id_, comp_id_, &msg,
        MAV_TYPE_GROUND_ROVER, MAV_AUTOPILOT_GENERIC,
        base_mode, state.custom_mode, MAV_STATE_ACTIVE);
    send(msg, state);
}

void MavSender::send_sys_status(const RoverState& state)
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

void MavSender::send_autopilot_version(const RoverState& state)
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

void MavSender::send_protocol_version(const RoverState& state)
{
    mavlink_message_t msg;
    mavlink_protocol_version_t pv{};
    pv.version     = 200;  // MAVLink 2
    pv.min_version = 100;
    pv.max_version = 200;
    mavlink_msg_protocol_version_encode(sys_id_, comp_id_, &msg, &pv);
    logger::line("tx: MAVLINK_MSG_ID_PROTOCOL_VERSION(300): version=%u", pv.version);
    send(msg, state);
}

void MavSender::send_current_mode(const RoverState& state)
{
    mavlink_current_mode_t cm{};
    cm.custom_mode          = state.custom_mode;
    cm.intended_custom_mode = state.custom_mode;
    cm.standard_mode        = (state.custom_mode == 4)
                              ? MAV_STANDARD_MODE_POSITION_HOLD
                              : MAV_STANDARD_MODE_NON_STANDARD;
    mavlink_message_t msg;
    mavlink_msg_current_mode_encode(sys_id_, comp_id_, &msg, &cm);
    send(msg, state);
}

void MavSender::send_available_modes(const RoverState& state, uint32_t mode)
{
    struct ModeInfo {
        const char* name;
        uint8_t     standard_mode;
        uint32_t    custom_mode;
        uint32_t    properties;
    };
    constexpr std::array<ModeInfo, 2> modes{{
        { "HOLD",   MAV_STANDARD_MODE_POSITION_HOLD, 4, 0 },
        { "MANUAL", MAV_STANDARD_MODE_NON_STANDARD,  0, 0 }
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

void MavSender::send_command_ack(const RoverState& state, uint16_t command, uint8_t result,
                                  uint8_t target_system, uint8_t target_component)
{
    mavlink_message_t msg;
    mavlink_msg_command_ack_pack(sys_id_, comp_id_, &msg,
        command, result,
        0, /* progress */
        0, /* result_param2 */
        target_system, target_component);
    // logger::line("tx: MAVLINK_MSG_ID_COMMAND_ACK(77): command=%u result=%u", command, result);
    send(msg, state);
}

void MavSender::send_mission_count(const RoverState& state,
                                    uint8_t target_system, uint8_t target_component)
{
    mavlink_message_t msg;
    mavlink_msg_mission_count_pack(sys_id_, comp_id_, &msg,
        target_system, target_component, 0, MAV_MISSION_TYPE_MISSION, 0);
    logger::line("tx: MAVLINK_MSG_ID_MISSION_COUNT(44): %u", 0);
    send(msg, state);
}

void MavSender::send_servo_output_raw(const RoverState& state, int16_t left, int16_t right)
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
        0, 0,
        pwm_left, pwm_right,
        neutral, neutral, neutral, neutral, neutral, neutral,
        neutral, neutral, neutral, neutral, neutral, neutral, neutral, neutral);
    send(msg, state);
}

void MavSender::send_camera_information(const RoverState& state)
{
    mavlink_camera_information_t info = {};
    // flags=0 → no capture capabilities; QGC stops polling on receipt
    mavlink_message_t msg;
    mavlink_msg_camera_information_encode(sys_id_, comp_id_, &msg, &info);
    send(msg, state);
}

void MavSender::send_camera_settings(const RoverState& state)
{
    mavlink_camera_settings_t s = {};
    mavlink_message_t msg;
    mavlink_msg_camera_settings_encode(sys_id_, comp_id_, &msg, &s);
    send(msg, state);
}

void MavSender::send_storage_information(const RoverState& state)
{
    mavlink_storage_information_t s = {};
    mavlink_message_t msg;
    mavlink_msg_storage_information_encode(sys_id_, comp_id_, &msg, &s);
    send(msg, state);
}

void MavSender::send_camera_capture_status(const RoverState& state)
{
    mavlink_camera_capture_status_t s = {};
    mavlink_message_t msg;
    mavlink_msg_camera_capture_status_encode(sys_id_, comp_id_, &msg, &s);
    send(msg, state);
}

void MavSender::send_param(const RoverState& state, const char* name, float value,
                            uint16_t index, uint16_t total)
{
    mavlink_message_t msg;
    mavlink_msg_param_value_pack(sys_id_, comp_id_, &msg,
        name, value, MAV_PARAM_TYPE_REAL32, total, index);
    logger::line("tx: MAVLINK_MSG_ID_PARAM_VALUE(22): %s=%.2f (%u/%u)",
                 name, value, index + 1, total);
    send(msg, state);
}

void MavSender::send_gps_raw_int(const RoverState& state)
{
    const GpsFix& f = state.current_fix;
    mavlink_message_t msg;
    mavlink_gps_raw_int_t g{};
    g.time_usec          = f.time_usec;
    g.fix_type           = f.fix_type;
    g.lat                = f.lat_degE7;
    g.lon                = f.lon_degE7;
    g.alt                = f.alt_mm;
    g.eph                = f.hdop_100;
    g.epv                = UINT16_MAX;   // not available from NMEA
    g.vel                = f.vel_cm_s;
    g.cog                = f.cog_cdeg;
    g.satellites_visible = f.satellites;
    g.alt_ellipsoid      = 0;
    g.h_acc              = UINT32_MAX;   // not available
    g.v_acc              = UINT32_MAX;
    g.vel_acc            = UINT32_MAX;
    g.hdg_acc            = UINT32_MAX;
    g.yaw                = 0;
    mavlink_msg_gps_raw_int_encode(sys_id_, comp_id_, &msg, &g);
    send(msg, state);
}

void MavSender::send_global_position_int(const RoverState& state)
{
    const GpsFix& f = state.current_fix;
    mavlink_message_t msg;

    // Decompose ground speed into north/east components (cm/s)
    float cog_rad = static_cast<float>(f.cog_cdeg) * (float)M_PI / 18000.0f;
    int16_t vx = static_cast<int16_t>(static_cast<float>(f.vel_cm_s) *  cosf(cog_rad));
    int16_t vy = static_cast<int16_t>(static_cast<float>(f.vel_cm_s) *  sinf(cog_rad));

    mavlink_msg_global_position_int_pack(sys_id_, comp_id_, &msg,
        0,                                    // time_boot_ms (unused — no boot time counter)
        f.lat_degE7,
        f.lon_degE7,
        f.alt_mm,
        f.alt_mm - state.home_alt_mm,         // relative_alt
        vx, vy,
        0,                                    // vz = 0 (ground rover)
        f.cog_cdeg);                          // hdg ≈ course over ground
    send(msg, state);
}

void MavSender::send(mavlink_message_t& msg, const RoverState& state)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    int len = mavlink_msg_to_send_buffer(buf, &msg);
    sock_.send_to(buf, static_cast<size_t>(len), state.qgc_addr, state.qgc_addr_len);
}
