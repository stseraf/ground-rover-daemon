#include "mav_sender.hpp"

#include <cstring>
#include <cmath>
#include <ctime>
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
        MAV_TYPE_GROUND_ROVER, MAV_AUTOPILOT_ARDUPILOTMEGA,
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
        MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT
      | MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT
      | MAV_PROTOCOL_CAPABILITY_MISSION_INT
      | MAV_PROTOCOL_CAPABILITY_COMMAND_INT
      | MAV_PROTOCOL_CAPABILITY_MISSION_FENCE
      | MAV_PROTOCOL_CAPABILITY_MISSION_RALLY
      | MAV_PROTOCOL_CAPABILITY_MAVLINK2;
    // ArduRover 4.6.3 OFFICIAL — encoded per AP_FWVersion convention.
    // Tracks the "latest stable" baked into QGC so it doesn't pop up the
    // "Vehicle is not running latest stable firmware" warning on connect.
    av.flight_sw_version = (4u << 24) | (6u << 16) | (3u << 8) | FIRMWARE_VERSION_TYPE_OFFICIAL;
    av.vendor_id         = 0x4141;  // 'AA' — ArduPilot convention
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
    // Both modes report NON_STANDARD so QGC renders our mode_name ("HOLD" /
    // "MANUAL") instead of substituting its own "Position" label for the
    // POSITION_HOLD standard-mode code.
    cm.standard_mode        = MAV_STANDARD_MODE_NON_STANDARD;
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
        { "HOLD",   MAV_STANDARD_MODE_NON_STANDARD, 4, 0 },
        { "MANUAL", MAV_STANDARD_MODE_NON_STANDARD, 0, 0 }
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
                                    uint8_t target_system, uint8_t target_component,
                                    uint8_t mission_type)
{
    mavlink_message_t msg;
    mavlink_msg_mission_count_pack(sys_id_, comp_id_, &msg,
        target_system, target_component, 0, mission_type, 0);
    logger::line("tx: MAVLINK_MSG_ID_MISSION_COUNT(44): count=0 type=%u", mission_type);
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

// ── Camera component sends ─────────────────────────────────────────────────

void MavSender::send_camera_heartbeat(uint8_t cam_comp_id, const RoverState& state)
{
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(sys_id_, cam_comp_id, &msg,
        MAV_TYPE_CAMERA, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
    send(msg, state);
}

void MavSender::send_camera_information(uint8_t cam_comp_id, const RoverState& state,
                                         const CameraInfo& cam)
{
    struct timespec ts;
    clock_gettime(CLOCK_BOOTTIME, &ts);
    mavlink_camera_information_t info{};
    info.time_boot_ms = static_cast<uint32_t>(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
    info.firmware_version = 1;
    info.resolution_h = cam.max_w;
    info.resolution_v = cam.max_h;
    info.flags        = CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM |
                        CAMERA_CAP_FLAGS_CAPTURE_VIDEO;
    std::strncpy(reinterpret_cast<char*>(info.vendor_name), "Rover",    32);
    std::strncpy(reinterpret_cast<char*>(info.model_name),  cam.model,  32);
    mavlink_message_t msg;
    mavlink_msg_camera_information_encode(sys_id_, cam_comp_id, &msg, &info);
    logger::line("tx: CAMERA_INFORMATION(259) cam_comp=%u model=%s", cam_comp_id, cam.model);
    send(msg, state);
}

void MavSender::send_video_stream_information(uint8_t cam_comp_id, const RoverState& state,
                                               const CameraInfo& cam, int cam_idx, int mode_idx)
{
    const SensorMode& mode    = cam.modes[mode_idx];
    bool              running = (state.active_cam_idx == cam_idx &&
                                 state.active_mode_idx == mode_idx);

    mavlink_video_stream_information_t info{};
    info.stream_id        = static_cast<uint8_t>(mode_idx + 1);
    info.count            = static_cast<uint8_t>(cam.modes.size());
    info.type             = VIDEO_STREAM_TYPE_RTPUDP;
    info.flags            = running ? VIDEO_STREAM_STATUS_FLAGS_RUNNING : 0;
    info.framerate        = std::min(static_cast<float>(state.video_fps), mode.fps);
    info.resolution_h     = mode.width;
    info.resolution_v     = mode.height;
    info.bitrate          = state.video_bitrate_bps;
    info.rotation         = 0;
    info.hfov             = 0;
    info.encoding         = VIDEO_STREAM_ENCODING_H264;
    info.camera_device_id = 0;  // 0 = MAVLink camera with its own comp_id
    std::memcpy(info.name, mode.name, sizeof(info.name));  // both are char[32]
    std::strncpy(info.uri,  "udp://0.0.0.0:5600", sizeof(info.uri)  - 1);

    mavlink_message_t msg;
    mavlink_msg_video_stream_information_encode(sys_id_, cam_comp_id, &msg, &info);
    send(msg, state);
}

void MavSender::send_video_stream_status(uint8_t cam_comp_id, const RoverState& state,
                                          const CameraInfo& cam, int cam_idx, uint8_t stream_id)
{
    int mode_idx = stream_id - 1;
    if (mode_idx < 0 || mode_idx >= static_cast<int>(cam.modes.size())) return;

    const SensorMode& mode    = cam.modes[mode_idx];
    bool              running = (state.active_cam_idx == cam_idx &&
                                 state.active_mode_idx == mode_idx);

    mavlink_video_stream_status_t status{};
    status.stream_id    = stream_id;
    status.flags        = running ? VIDEO_STREAM_STATUS_FLAGS_RUNNING : 0;
    status.framerate    = std::min(static_cast<float>(state.video_fps), mode.fps);
    status.resolution_h = mode.width;
    status.resolution_v = mode.height;
    status.bitrate      = state.video_bitrate_bps;
    status.rotation     = 0;
    status.hfov         = 0;

    mavlink_message_t msg;
    mavlink_msg_video_stream_status_encode(sys_id_, cam_comp_id, &msg, &status);
    send(msg, state);
}

void MavSender::send_command_ack_cam(uint8_t cam_comp_id, const RoverState& state,
                                      uint16_t command, uint8_t result,
                                      uint8_t target_system, uint8_t target_component)
{
    mavlink_message_t msg;
    mavlink_msg_command_ack_pack(sys_id_, cam_comp_id, &msg,
        command, result, 0, 0, target_system, target_component);
    send(msg, state);
}

// ── Legacy stub responses ──────────────────────────────────────────────────

void MavSender::send_camera_settings(uint8_t cam_comp_id, const RoverState& state)
{
    struct timespec ts;
    clock_gettime(CLOCK_BOOTTIME, &ts);
    mavlink_camera_settings_t s = {};
    s.time_boot_ms = static_cast<uint32_t>(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
    s.mode_id      = CAMERA_MODE_VIDEO;
    mavlink_message_t msg;
    mavlink_msg_camera_settings_encode(sys_id_, cam_comp_id, &msg, &s);
    send(msg, state);
}

void MavSender::send_storage_information(uint8_t cam_comp_id, const RoverState& state)
{
    mavlink_storage_information_t s = {};
    s.storage_id    = 1;
    s.storage_count = 0;  // no removable storage
    s.status        = STORAGE_STATUS_NOT_SUPPORTED;
    mavlink_message_t msg;
    mavlink_msg_storage_information_encode(sys_id_, cam_comp_id, &msg, &s);
    send(msg, state);
}

void MavSender::send_camera_capture_status(uint8_t cam_comp_id, const RoverState& state)
{
    struct timespec ts;
    clock_gettime(CLOCK_BOOTTIME, &ts);
    mavlink_camera_capture_status_t s = {};
    s.time_boot_ms = static_cast<uint32_t>(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
    mavlink_message_t msg;
    mavlink_msg_camera_capture_status_encode(sys_id_, cam_comp_id, &msg, &s);
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

// CSQ 0-31 (AT+CSQ scale) → RADIO_STATUS rssi byte.
// QGC interprets the rssi field as int8_t and displays it directly as dBm.
// AT+CSQ dBm formula: dBm = -113 + 2*CSQ
//   CSQ 0  → -113 dBm, CSQ 27 → -59 dBm, CSQ 31 → -51 dBm
// CSQ > 31 (e.g. 99 = "unknown") → -127 dBm (conventional "no signal").
static uint8_t csq_to_radio_rssi(uint8_t csq)
{
    int8_t dbm = (csq > 31) ? INT8_MIN : static_cast<int8_t>(-113 + 2 * csq);
    return static_cast<uint8_t>(dbm);
}

void MavSender::send_radio_status(const RoverState& state)
{
    if (!state.qgc_known)
        return;

    const LteStatus& lte = state.lte;

    // rssi  = primary link signal (int8_t dBm encoded as uint8_t).
    // remrssi = secondary link signal (same encoding).
    // When on WiFi: rssi = WiFi AP dBm, remrssi = LTE tower dBm (background).
    // When on LTE:  rssi = LTE tower dBm, remrssi = 0 (no WiFi).
    // Disconnected: rssi = -127 dBm (INT8_MIN → raw 0x80).
    uint8_t rssi    = static_cast<uint8_t>(INT8_MIN);  // default: no signal
    uint8_t remrssi = 0;

    if (lte.connected) {
        bool wifi_active = ::strncmp(lte.uplink, "wifi", 4) == 0;
        if (wifi_active && lte.wifi_rssi_dbm != 0) {
            rssi    = static_cast<uint8_t>(static_cast<int8_t>(lte.wifi_rssi_dbm));
            remrssi = csq_to_radio_rssi(lte.rssi);
        } else {
            rssi    = csq_to_radio_rssi(lte.rssi);
            remrssi = 0;
        }
    }

    // txbuf: repurposed as link-type sentinel visible in MAVLink Inspector.
    // 100 = LTE active, 50 = WiFi active, 0 = no link.
    uint8_t txbuf = 0;
    if (lte.connected)
        txbuf = (::strncmp(lte.uplink, "wifi", 4) == 0) ? 50 : 100;

    mavlink_message_t msg;
    mavlink_msg_radio_status_pack(sys_id_, comp_id_, &msg,
        rssi, remrssi, txbuf, 0, 0, 0, 0);
    send(msg, state);
}

void MavSender::send_gimbal_manager_information(const RoverState& state,
                                                  uint8_t from_comp_id)
{
    mavlink_gimbal_manager_information_t gmi{};
    struct timespec ts;
    clock_gettime(CLOCK_BOOTTIME, &ts);
    gmi.time_boot_ms     = static_cast<uint32_t>(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
    gmi.cap_flags        = 0;  // no axis control, no lock, no retract — nothing
    gmi.gimbal_device_id = 0;  // no associated device → "standalone, empty" manager
    // All angle ranges left at 0 (default) so QGC shows no control affordance.
    mavlink_message_t msg;
    mavlink_msg_gimbal_manager_information_encode(sys_id_, from_comp_id, &msg, &gmi);
    send(msg, state);
}

void MavSender::send_statustext(const RoverState& state, uint8_t severity, const char* text)
{
    mavlink_message_t msg;
    char buf[50]{};
    std::strncpy(buf, text, sizeof(buf) - 1);
    mavlink_msg_statustext_pack(sys_id_, comp_id_, &msg, severity, buf, 0, 0);
    send(msg, state);
}

void MavSender::send(mavlink_message_t& msg, const RoverState& state)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    int len = mavlink_msg_to_send_buffer(buf, &msg);
    sock_.send_to(buf, static_cast<size_t>(len), state.qgc_addr, state.qgc_addr_len);
}
