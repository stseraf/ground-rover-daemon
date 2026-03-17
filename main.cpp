#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cerrno>
#include <unistd.h>
#include <ctime>

#include <array>

#include "udp_socket.hpp"
#include "rover_state.hpp"
#include "logger.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include "external/mavlink/common/mavlink.h"
#pragma GCC diagnostic pop

namespace Config {
    constexpr uint8_t MAV_SYS_ID = 1;
    constexpr uint8_t MAV_COMP_ID = 1;
    constexpr uint16_t UDP_BIND_PORT = 14550;
    constexpr uint64_t HEARTBEAT_INTERVAL_US = 1'000'000; // 1 Hz
    constexpr unsigned LOOP_SLEEP_US = 1'000;      // 1 ms

    // Stub telemetry values
    constexpr uint16_t DUMMY_LOAD_PERMILLE = 500; // stub: 50.0% CPU load
    constexpr uint16_t DUMMY_BATTERY_MV = 12000; // stub: 12 V
    constexpr int16_t DUMMY_CURRENT_CA = 0; // stub: 0 A
    constexpr int8_t DUMMY_BATTERY_PCT = 80; // stub: 80%
}

/* -------------------- utils -------------------- */

static uint64_t micros()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000ULL + static_cast<uint64_t>(ts.tv_nsec) / 1000;
}

/* -------------------- MAVLink TX -------------------- */

static void mav_send(UdpSocket& sock, RoverState& state, mavlink_message_t *msg)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    int len = mavlink_msg_to_send_buffer(buf, msg);
    sock.send_to(buf, static_cast<size_t>(len), state.qgc_addr, state.qgc_addr_len);
}

static void send_heartbeat(UdpSocket& sock, RoverState& state)
{
    mavlink_message_t msg;

    uint8_t base_mode = static_cast<uint8_t>(state.base_mode | (state.armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0));

    mavlink_msg_heartbeat_pack(
        Config::MAV_SYS_ID,
        Config::MAV_COMP_ID,
        &msg,
        MAV_TYPE_GROUND_ROVER,
        MAV_AUTOPILOT_GENERIC,
        base_mode,
        state.custom_mode,
        MAV_STATE_ACTIVE
    );
    mav_send(sock, state, &msg);
}

static void send_sys_status(UdpSocket& sock, RoverState& state)
{
    mavlink_message_t msg;

    mavlink_sys_status_t sys{};
    sys.load              = Config::DUMMY_LOAD_PERMILLE;
    sys.voltage_battery   = Config::DUMMY_BATTERY_MV;
    sys.current_battery   = Config::DUMMY_CURRENT_CA;
    sys.battery_remaining = Config::DUMMY_BATTERY_PCT;

    mavlink_msg_sys_status_encode(Config::MAV_SYS_ID, Config::MAV_COMP_ID, &msg, &sys);
    mav_send(sock, state, &msg);
}

static void send_autopilot_version(UdpSocket& sock, RoverState& state)
{
    mavlink_message_t msg;

    mavlink_autopilot_version_t av{};
    av.capabilities =
      MAV_PROTOCOL_CAPABILITY_MISSION_INT
    | MAV_PROTOCOL_CAPABILITY_COMMAND_INT
    | MAV_PROTOCOL_CAPABILITY_MAVLINK2;

    mavlink_msg_autopilot_version_encode(Config::MAV_SYS_ID, Config::MAV_COMP_ID, &msg, &av);
    logger::line("tx: MAVLINK_MSG_ID_AUTOPILOT_VERSION(148): capabilities=0x%016llX", (unsigned long long)av.capabilities);
    mav_send(sock, state, &msg);
}

static void send_available_modes(UdpSocket& sock, RoverState& state, uint32_t mode)
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
    am.mode_index    = mode; /* indexed from 1 */
    am.standard_mode = modes[mode-1].standard_mode;
    am.custom_mode   = modes[mode-1].custom_mode;
    am.properties    = modes[mode-1].properties;
    std::strncpy(am.mode_name, modes[mode-1].name, sizeof(am.mode_name) - 1);

    mavlink_msg_available_modes_encode(Config::MAV_SYS_ID, Config::MAV_COMP_ID, &msg, &am);
    logger::line("tx: MAVLINK_MSG_ID_AVAILABLE_MODES(435) mode %u/%u: %s", mode, number_modes, modes[mode-1].name);
    mav_send(sock, state, &msg);
}

static void send_command_ack(UdpSocket& sock, RoverState& state, uint16_t command, uint8_t result,
                             uint8_t target_system, uint8_t target_component)
{
    mavlink_message_t ack;
    mavlink_msg_command_ack_pack(
        Config::MAV_SYS_ID,
        Config::MAV_COMP_ID,
        &ack,
        command,
        result,
        0, /* progress */
        0, /* result_param2 */
        target_system,
        target_component
    );
    logger::line("tx: MAVLINK_MSG_ID_COMMAND_ACK(77): command=%u result=%u", command, result);
    mav_send(sock, state, &ack);
}

/* -------------------- COMMAND HANDLING -------------------- */

static void handle_command_long(UdpSocket& sock, RoverState& state,
                                const mavlink_command_long_t *cmd)
{
    if (cmd->command == MAV_CMD_COMPONENT_ARM_DISARM) {
        state.armed = (cmd->param1 > 0.5f);
        logger::line("rx: MAVLINK_MSG_ID_COMMAND_LONG(76): MAV_CMD_COMPONENT_ARM_DISARM(400): Arm=%d", static_cast<uint32_t>(cmd->param1));
        send_command_ack(sock, state, cmd->command, MAV_RESULT_ACCEPTED, cmd->target_system, cmd->target_component);
    }
    else if (cmd->command == MAV_CMD_REQUEST_MESSAGE) {
        logger::line("rx: MAVLINK_MSG_ID_COMMAND_LONG(76): MAV_CMD_REQUEST_MESSAGE(512): ");

        if (static_cast<uint32_t>(cmd->param1) == MAVLINK_MSG_ID_SYS_STATUS) {
            std::printf("MAVLINK_MSG_ID_SYS_STATUS(1) handle\n");
            send_command_ack(sock, state, cmd->command, MAV_RESULT_ACCEPTED, cmd->target_system, cmd->target_component);
            send_sys_status(sock, state);
        }
        else if (static_cast<uint32_t>(cmd->param1) == MAVLINK_MSG_ID_AUTOPILOT_VERSION) {
            std::printf("MAVLINK_MSG_ID_AUTOPILOT_VERSION(148) handle\n");
            send_command_ack(sock, state, cmd->command, MAV_RESULT_ACCEPTED, cmd->target_system, cmd->target_component);
            send_autopilot_version(sock, state);
        }
        else if (static_cast<uint32_t>(cmd->param1) == MAVLINK_MSG_ID_CAMERA_INFORMATION) {
            std::printf("MAVLINK_MSG_ID_CAMERA_INFORMATION(259): MAV_RESULT_UNSUPPORTED\n");
            send_command_ack(sock, state, cmd->command, MAV_RESULT_UNSUPPORTED, cmd->target_system, cmd->target_component);
        }
        else if (static_cast<uint32_t>(cmd->param1) == MAVLINK_MSG_ID_AVAILABLE_MODES) {
            std::printf("MAVLINK_MSG_ID_AVAILABLE_MODES(435) handle requested mode=%u\n", static_cast<uint32_t>(cmd->param2));
            send_command_ack(sock, state, cmd->command, MAV_RESULT_ACCEPTED, cmd->target_system, cmd->target_component);
            send_available_modes(sock, state, static_cast<uint32_t>(cmd->param2));
        }
        else if (static_cast<uint32_t>(cmd->param1) == MAVLINK_MSG_ID_COMPONENT_METADATA) {
            std::printf("MAVLINK_MSG_ID_COMPONENT_METADATA(397): MAV_RESULT_UNSUPPORTED\n");
            send_command_ack(sock, state, cmd->command, MAV_RESULT_UNSUPPORTED, cmd->target_system, cmd->target_component);
        }
        else if (static_cast<uint32_t>(cmd->param1) == MAVLINK_MSG_ID_COMPONENT_INFORMATION) {
            std::printf("MAVLINK_MSG_ID_COMPONENT_INFORMATION(395) MAV_RESULT_UNSUPPORTED\n");
            send_command_ack(sock, state, cmd->command, MAV_RESULT_UNSUPPORTED, cmd->target_system, cmd->target_component);
        }
        else if (static_cast<uint32_t>(cmd->param1) == MAVLINK_MSG_ID_PROTOCOL_VERSION) {
            std::printf("MAVLINK_MSG_ID_PROTOCOL_VERSION(300): MAV_RESULT_UNSUPPORTED - Deprecated\n");
            send_command_ack(sock, state, cmd->command, MAV_RESULT_UNSUPPORTED, cmd->target_system, cmd->target_component);
        }
        else if (static_cast<uint32_t>(cmd->param1) == MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION) {
            std::printf("MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION(280): MAV_RESULT_UNSUPPORTED\n");
            send_command_ack(sock, state, cmd->command, MAV_RESULT_UNSUPPORTED, cmd->target_system, cmd->target_component);
        }
        else {
            std::printf("Unknown REQUEST_MESSAGE(%u): MAV_RESULT_UNSUPPORTED\n", static_cast<uint32_t>(cmd->param1));
            send_command_ack(sock, state, cmd->command, MAV_RESULT_UNSUPPORTED, cmd->target_system, cmd->target_component);
        }
    }
    else if (cmd->command == MAV_CMD_DO_SET_MODE) {
        logger::line("rx: MAVLINK_MSG_ID_COMMAND_LONG(76): MAV_CMD_DO_SET_MODE(176): base_mode=0x%02X custom_mode=%u", static_cast<uint32_t>(cmd->param1), static_cast<uint32_t>(cmd->param2));
        state.base_mode   = static_cast<uint8_t>(cmd->param1);
        state.custom_mode = static_cast<uint32_t>(cmd->param2);
        send_command_ack(sock, state, cmd->command, MAV_RESULT_ACCEPTED, cmd->target_system, cmd->target_component);
    }
    else if (cmd->command == MAV_CMD_NAV_TAKEOFF) {
        logger::line("rx: MAVLINK_MSG_ID_COMMAND_LONG(76): MAV_CMD_NAV_TAKEOFF(22): MAV_RESULT_UNSUPPORTED");
        send_command_ack(sock, state, cmd->command, MAV_RESULT_UNSUPPORTED, cmd->target_system, cmd->target_component);
    }
    else if (cmd->command == MAV_CMD_REQUEST_CAMERA_INFORMATION) {
        logger::line("rx: MAVLINK_MSG_ID_COMMAND_LONG(76): MAV_CMD_REQUEST_CAMERA_INFORMATION(521): MAV_RESULT_UNSUPPORTED");
        send_command_ack(sock, state, cmd->command, MAV_RESULT_UNSUPPORTED, cmd->target_system, cmd->target_component);
    }
    else if (cmd->command == MAV_CMD_MISSION_START) {
        logger::line("rx: MAVLINK_MSG_ID_COMMAND_LONG(76): MAV_CMD_MISSION_START(300): MAV_RESULT_UNSUPPORTED");
        send_command_ack(sock, state, cmd->command, MAV_RESULT_UNSUPPORTED, cmd->target_system, cmd->target_component);
    }
    else {
        logger::line("rx: MAVLINK_MSG_ID_COMMAND_LONG(76): Unknown COMMAND_LONG(%u): MAV_RESULT_UNSUPPORTED", static_cast<uint32_t>(cmd->command));
        send_command_ack(sock, state, cmd->command, MAV_RESULT_UNSUPPORTED, cmd->target_system, cmd->target_component);
    }
}

/* -------------------- MAIN -------------------- */

int main()
{
    UdpSocket sock{Config::UDP_BIND_PORT};
    RoverState state{};

    logger::line("MAVLink rover daemon started");

    uint64_t last_hb = 0;
    mavlink_message_t msg;
    mavlink_status_t  status;

    while (true) {
        uint8_t receive_buffer[2048];
        ssize_t n = sock.recv_nonblocking(receive_buffer, sizeof(receive_buffer),
                                          state.qgc_addr, state.qgc_addr_len);
        if (n > 0) {
            state.qgc_known = true;
            for (ssize_t i = 0; i < n; i++) {
                if (mavlink_parse_char(MAVLINK_COMM_0, receive_buffer[i], &msg, &status)) {
                    switch (msg.msgid) {
                        case MAVLINK_MSG_ID_HEARTBEAT: {
                            mavlink_heartbeat_t hb;
                            mavlink_msg_heartbeat_decode(&msg, &hb);
                            //logger::line("rx: MAVLINK_MSG_ID_HEARTBEAT(0): type=%u autopilot=%u base_mode=0x%02X", hb.type, hb.autopilot, hb.base_mode);
                            break;
                        }
                        case MAVLINK_MSG_ID_SET_MODE: {
                            mavlink_set_mode_t sm;
                            mavlink_msg_set_mode_decode(&msg, &sm);
                            logger::line("rx: MAVLINK_MSG_ID_SET_MODE(11): base=0x%02X custom=%u", sm.base_mode, sm.custom_mode);
                            state.base_mode   = sm.base_mode;
                            state.custom_mode = sm.custom_mode;
                            break;
                        }
                        case MAVLINK_MSG_ID_MANUAL_CONTROL: {
                            mavlink_manual_control_t mc;
                            mavlink_msg_manual_control_decode(&msg, &mc);
                            logger::same_line("rx: MAVLINK_MSG_ID_MANUAL_CONTROL(69) x=%d y=%d z=%d r=%d buttons=0x%02X           ", mc.x, mc.y, mc.z, mc.r, mc.buttons);
                            break;
                        }
                        case MAVLINK_MSG_ID_COMMAND_LONG: {
                            mavlink_command_long_t cmd;
                            mavlink_msg_command_long_decode(&msg, &cmd);
                            handle_command_long(sock, state, &cmd);
                            break;
                        }
                        case MAVLINK_MSG_ID_REQUEST_DATA_STREAM: {
                            mavlink_request_data_stream_t rds;
                            mavlink_msg_request_data_stream_decode(&msg, &rds);
                            logger::line("rx: MAVLINK_MSG_ID_REQUEST_DATA_STREAM(66): req_stream_id=%u req_message_rate=%u start_stop=%u",
                                rds.req_stream_id, rds.req_message_rate, rds.start_stop);
                            break;
                        }
                        case MAVLINK_MSG_ID_DATA_STREAM: {
                            mavlink_data_stream_t ds;
                            mavlink_msg_data_stream_decode(&msg, &ds);
                            logger::line("rx: MAVLINK_MSG_ID_DATA_STREAM(67): stream_id=%u message_rate=%u", ds.stream_id, ds.message_rate);
                            break;
                        }
                        case MAVLINK_MSG_ID_SYSTEM_TIME: {
                            mavlink_system_time_t st;
                            mavlink_msg_system_time_decode(&msg, &st);
                            logger::line("rx: MAVLINK_MSG_ID_SYSTEM_TIME(2): time_unix_usec=%llu time_boot_ms=%u", (unsigned long long)st.time_unix_usec, st.time_boot_ms);
                            break;
                        }
                        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
                            mavlink_mission_request_list_t mrl;
                            mavlink_msg_mission_request_list_decode(&msg, &mrl);
                            logger::line("rx: MAVLINK_MSG_ID_MISSION_REQUEST_LIST(43): target=%u:%u", mrl.target_system, mrl.target_component);
                            {
                                std::printf("MAVLINK_MSG_ID_MISSION_COUNT(44) handle\n");
                                mavlink_message_t mc;
                                mavlink_msg_mission_count_pack(
                                    Config::MAV_SYS_ID,
                                    Config::MAV_COMP_ID,
                                    &mc,
                                    mrl.target_system,
                                    mrl.target_component,
                                    0,
                                    MAV_MISSION_TYPE_MISSION,
                                    0
                                );
                                logger::line("tx: MAVLINK_MSG_ID_MISSION_COUNT(44): %u", 0);
                                mav_send(sock, state, &mc);
                            }
                            break;
                        }
                        case MAVLINK_MSG_ID_MISSION_ACK: {
                            mavlink_mission_ack_t ma;
                            mavlink_msg_mission_ack_decode(&msg, &ma);
                            logger::line("rx: MAVLINK_MSG_ID_MISSION_ACK(47): target=%u:%u type=%u", ma.target_system, ma.target_component, ma.type);
                            break;
                        }
                        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
                            mavlink_param_request_list_t prl;
                            mavlink_msg_param_request_list_decode(&msg, &prl);
                            logger::line("rx: MAVLINK_MSG_ID_PARAM_REQUEST_LIST(21) received target=%u:%u", prl.target_system, prl.target_component);
                            {
                                std::printf("MAVLINK_MSG_ID_PARAM_VALUE(22) handle\n");
                                constexpr uint16_t param_count = 3;
                                const char *names[param_count] = { "DUMMY_P1", "DUMMY_P2", "DUMMY_P3" };
                                const float values[param_count] = { 0.0f, 0.0f, 0.0f };

                                for (uint16_t i = 0; i < param_count; i++) {
                                    mavlink_message_t pm;
                                    mavlink_msg_param_value_pack(
                                        Config::MAV_SYS_ID,
                                        Config::MAV_COMP_ID,
                                        &pm,
                                        names[i],
                                        values[i],
                                        MAV_PARAM_TYPE_REAL32,
                                        param_count,
                                        i
                                    );
                                    logger::line("tx: MAVLINK_MSG_ID_PARAM_VALUE(22): %s=%.2f (%u/%u)", names[i], values[i], i+1, param_count);
                                    mav_send(sock, state, &pm);
                                }
                            }
                            break;
                        }
                        default:
                            logger::line("rx: Unknown message ID: %u", msg.msgid);
                            break;
                    }
                }
            }
        }

        uint64_t now = micros();

        if (now - last_hb > Config::HEARTBEAT_INTERVAL_US) {
            send_heartbeat(sock, state);
            send_sys_status(sock, state);
            last_hb = now;
        }
        usleep(Config::LOOP_SLEEP_US);
    }
}
