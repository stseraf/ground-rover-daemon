#include <cstdio>
#include <cstdlib>
#include <cerrno>
#include <unistd.h>
#include <ctime>

#include "config.hpp"
#include "udp_socket.hpp"
#include "rover_state.hpp"
#include "logger.hpp"
#include "mav_sender.hpp"

/* -------------------- utils -------------------- */

static uint64_t micros()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000ULL + static_cast<uint64_t>(ts.tv_nsec) / 1000;
}

/* -------------------- COMMAND HANDLING -------------------- */

static void handle_command_long(MavSender& mav, RoverState& state,
                                const mavlink_command_long_t *cmd)
{
    if (cmd->command == MAV_CMD_COMPONENT_ARM_DISARM) {
        state.armed = (cmd->param1 > 0.5f);
        logger::line("rx: MAVLINK_MSG_ID_COMMAND_LONG(76): MAV_CMD_COMPONENT_ARM_DISARM(400): Arm=%d", static_cast<uint32_t>(cmd->param1));
        mav.send_command_ack(state, cmd->command, MAV_RESULT_ACCEPTED, cmd->target_system, cmd->target_component);
    }
    else if (cmd->command == MAV_CMD_REQUEST_MESSAGE) {
        logger::line("rx: MAVLINK_MSG_ID_COMMAND_LONG(76): MAV_CMD_REQUEST_MESSAGE(512): ");

        if (static_cast<uint32_t>(cmd->param1) == MAVLINK_MSG_ID_SYS_STATUS) {
            std::printf("MAVLINK_MSG_ID_SYS_STATUS(1) handle\n");
            mav.send_command_ack(state, cmd->command, MAV_RESULT_ACCEPTED, cmd->target_system, cmd->target_component);
            mav.send_sys_status(state);
        }
        else if (static_cast<uint32_t>(cmd->param1) == MAVLINK_MSG_ID_AUTOPILOT_VERSION) {
            std::printf("MAVLINK_MSG_ID_AUTOPILOT_VERSION(148) handle\n");
            mav.send_command_ack(state, cmd->command, MAV_RESULT_ACCEPTED, cmd->target_system, cmd->target_component);
            mav.send_autopilot_version(state);
        }
        else if (static_cast<uint32_t>(cmd->param1) == MAVLINK_MSG_ID_CAMERA_INFORMATION) {
            std::printf("MAVLINK_MSG_ID_CAMERA_INFORMATION(259): MAV_RESULT_UNSUPPORTED\n");
            mav.send_command_ack(state, cmd->command, MAV_RESULT_UNSUPPORTED, cmd->target_system, cmd->target_component);
        }
        else if (static_cast<uint32_t>(cmd->param1) == MAVLINK_MSG_ID_AVAILABLE_MODES) {
            std::printf("MAVLINK_MSG_ID_AVAILABLE_MODES(435) handle requested mode=%u\n", static_cast<uint32_t>(cmd->param2));
            mav.send_command_ack(state, cmd->command, MAV_RESULT_ACCEPTED, cmd->target_system, cmd->target_component);
            mav.send_available_modes(state, static_cast<uint32_t>(cmd->param2));
        }
        else if (static_cast<uint32_t>(cmd->param1) == MAVLINK_MSG_ID_COMPONENT_METADATA) {
            std::printf("MAVLINK_MSG_ID_COMPONENT_METADATA(397): MAV_RESULT_UNSUPPORTED\n");
            mav.send_command_ack(state, cmd->command, MAV_RESULT_UNSUPPORTED, cmd->target_system, cmd->target_component);
        }
        else if (static_cast<uint32_t>(cmd->param1) == MAVLINK_MSG_ID_COMPONENT_INFORMATION) {
            std::printf("MAVLINK_MSG_ID_COMPONENT_INFORMATION(395) MAV_RESULT_UNSUPPORTED\n");
            mav.send_command_ack(state, cmd->command, MAV_RESULT_UNSUPPORTED, cmd->target_system, cmd->target_component);
        }
        else if (static_cast<uint32_t>(cmd->param1) == MAVLINK_MSG_ID_PROTOCOL_VERSION) {
            std::printf("MAVLINK_MSG_ID_PROTOCOL_VERSION(300): MAV_RESULT_UNSUPPORTED - Deprecated\n");
            mav.send_command_ack(state, cmd->command, MAV_RESULT_UNSUPPORTED, cmd->target_system, cmd->target_component);
        }
        else if (static_cast<uint32_t>(cmd->param1) == MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION) {
            std::printf("MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION(280): MAV_RESULT_UNSUPPORTED\n");
            mav.send_command_ack(state, cmd->command, MAV_RESULT_UNSUPPORTED, cmd->target_system, cmd->target_component);
        }
        else {
            std::printf("Unknown REQUEST_MESSAGE(%u): MAV_RESULT_UNSUPPORTED\n", static_cast<uint32_t>(cmd->param1));
            mav.send_command_ack(state, cmd->command, MAV_RESULT_UNSUPPORTED, cmd->target_system, cmd->target_component);
        }
    }
    else if (cmd->command == MAV_CMD_DO_SET_MODE) {
        logger::line("rx: MAVLINK_MSG_ID_COMMAND_LONG(76): MAV_CMD_DO_SET_MODE(176): base_mode=0x%02X custom_mode=%u", static_cast<uint32_t>(cmd->param1), static_cast<uint32_t>(cmd->param2));
        state.base_mode   = static_cast<uint8_t>(cmd->param1);
        state.custom_mode = static_cast<uint32_t>(cmd->param2);
        mav.send_command_ack(state, cmd->command, MAV_RESULT_ACCEPTED, cmd->target_system, cmd->target_component);
    }
    else if (cmd->command == MAV_CMD_NAV_TAKEOFF) {
        logger::line("rx: MAVLINK_MSG_ID_COMMAND_LONG(76): MAV_CMD_NAV_TAKEOFF(22): MAV_RESULT_UNSUPPORTED");
        mav.send_command_ack(state, cmd->command, MAV_RESULT_UNSUPPORTED, cmd->target_system, cmd->target_component);
    }
    else if (cmd->command == MAV_CMD_REQUEST_CAMERA_INFORMATION) {
        logger::line("rx: MAVLINK_MSG_ID_COMMAND_LONG(76): MAV_CMD_REQUEST_CAMERA_INFORMATION(521): MAV_RESULT_UNSUPPORTED");
        mav.send_command_ack(state, cmd->command, MAV_RESULT_UNSUPPORTED, cmd->target_system, cmd->target_component);
    }
    else if (cmd->command == MAV_CMD_MISSION_START) {
        logger::line("rx: MAVLINK_MSG_ID_COMMAND_LONG(76): MAV_CMD_MISSION_START(300): MAV_RESULT_UNSUPPORTED");
        mav.send_command_ack(state, cmd->command, MAV_RESULT_UNSUPPORTED, cmd->target_system, cmd->target_component);
    }
    else {
        logger::line("rx: MAVLINK_MSG_ID_COMMAND_LONG(76): Unknown COMMAND_LONG(%u): MAV_RESULT_UNSUPPORTED", static_cast<uint32_t>(cmd->command));
        mav.send_command_ack(state, cmd->command, MAV_RESULT_UNSUPPORTED, cmd->target_system, cmd->target_component);
    }
}

/* -------------------- MAIN -------------------- */

int main()
{
    UdpSocket  sock{Config::UDP_BIND_PORT};
    RoverState state{};
    MavSender  mav{sock, Config::MAV_SYS_ID, Config::MAV_COMP_ID};

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
                            handle_command_long(mav, state, &cmd);
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
                            std::printf("MAVLINK_MSG_ID_MISSION_COUNT(44) handle\n");
                            mav.send_mission_count(state, mrl.target_system, mrl.target_component);
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
                            std::printf("MAVLINK_MSG_ID_PARAM_VALUE(22) handle\n");
                            constexpr uint16_t param_count = 3;
                            const char  *names[param_count]  = { "DUMMY_P1", "DUMMY_P2", "DUMMY_P3" };
                            const float  values[param_count] = { 0.0f, 0.0f, 0.0f };
                            for (uint16_t i = 0; i < param_count; i++)
                                mav.send_param(state, names[i], values[i], i, param_count);
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
            mav.send_heartbeat(state);
            mav.send_sys_status(state);
            last_hb = now;
        }
        usleep(Config::LOOP_SLEEP_US);
    }
}
