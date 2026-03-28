#include <cstdio>
#include <cstdlib>
#include <cerrno>
#include <unistd.h>
#include <ctime>
#include <csignal>
#include <atomic>

#include "config.hpp"
#include "udp_socket.hpp"
#include "rover_state.hpp"
#include "logger.hpp"
#include "mavlink/mav_sender.hpp"
#include "mavlink/command_handlers.hpp"
#include "drive/diff_drive.hpp"
#include "motor/uart_motor_driver.hpp"
#ifdef DRIVER_TB6612
#include "motor/tb6612_driver.hpp"
#endif

static std::atomic<bool> running{true};

static void handle_signal(int sig)
{
    const char *msg_int  = "\n[signal] SIGINT received — shutting down\n";
    const char *msg_term = "\n[signal] SIGTERM received — shutting down\n";
    const char *msg = (sig == SIGINT) ? msg_int : msg_term;
    ssize_t unused __attribute__((unused)) = write(STDOUT_FILENO, msg, __builtin_strlen(msg));
    running = false;
}

static uint64_t micros()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000ULL + static_cast<uint64_t>(ts.tv_nsec) / 1000;
}

int main()
{
    std::signal(SIGINT,  handle_signal);
    std::signal(SIGTERM, handle_signal);
    UdpSocket  sock{Config::UDP_BIND_PORT};
    RoverState state{};
    MavSender  mav{sock, Config::MAV_SYS_ID, Config::MAV_COMP_ID};

    logger::line("MAVLink rover daemon started");

#ifdef DRIVER_TB6612
    Tb6612Driver    motors{};
#else
    UartMotorDriver motors{};
#endif
    DriveSlew slew{};
    uint64_t last_mc_us        = 0;
    uint64_t last_hb           = 0;
    bool     mc_timeout_active = false;
    uint64_t last_slew_tick_us = 0;
    bool     prev_armed        = false;
    mavlink_message_t msg;
    mavlink_status_t  status;

    while (running) {
        uint8_t receive_buffer[2048];
        ssize_t n = sock.recv(receive_buffer, sizeof(receive_buffer),
                              state.qgc_addr, state.qgc_addr_len);
        if (n > 0) {
            state.qgc_known = true;
            for (ssize_t i = 0; i < n; i++) {
                if (mavlink_parse_char(MAVLINK_COMM_0, receive_buffer[i], &msg, &status)) {
                    switch (msg.msgid) {
                        case MAVLINK_MSG_ID_HEARTBEAT: {
                            mavlink_heartbeat_t hb;
                            mavlink_msg_heartbeat_decode(&msg, &hb);
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
                            uint64_t now_mc = micros();
                            if (state.armed) {
                                uint32_t elapsed_ms = static_cast<uint32_t>(
                                    std::min<uint64_t>((now_mc - last_mc_us) / 1000, 500));
                                DriveOutput raw      = compute_diff_drive(mc.x, mc.y);
                                DriveOutput smoothed = slew.step(raw, elapsed_ms);
                                int ain1 = smoothed.left  > 0 ? 1 : 0;
                                int ain2 = smoothed.left  < 0 ? 1 : 0;
                                int bin1 = smoothed.right > 0 ? 1 : 0;
                                int bin2 = smoothed.right < 0 ? 1 : 0;
                                int pwma = (smoothed.left  < 0 ? -smoothed.left  : smoothed.left)  / 10;
                                int pwmb = (smoothed.right < 0 ? -smoothed.right : smoothed.right) / 10;
                                logger::same_line(
                                    "rx: MC(69) x=%5d y=%5d z=%5d r=%5d btn=0x%02X"
                                    " | A:IN=%d%d PWM=%3d%% | B:IN=%d%d PWM=%3d%% | STBY=H",
                                    mc.x, mc.y, mc.z, mc.r, mc.buttons,
                                    ain1, ain2, pwma, bin1, bin2, pwmb);
                                motors.set(smoothed.left, smoothed.right);
                                mav.send_servo_output_raw(state, smoothed.left, smoothed.right);
                            } else {
                                slew.reset();
                                logger::same_line(
                                    "rx: MC(69) x=%5d y=%5d z=%5d r=%5d btn=0x%02X"
                                    " | A:IN=00 PWM=  0%% | B:IN=00 PWM=  0%% | STBY=L",
                                    mc.x, mc.y, mc.z, mc.r, mc.buttons);
                                motors.stop();
                                mav.send_servo_output_raw(state, 0, 0);
                            }
                            last_mc_us = now_mc;
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
                            //std::printf("MAVLINK_MSG_ID_MISSION_COUNT(44) handle\n");
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
                            //std::printf("MAVLINK_MSG_ID_PARAM_VALUE(22) handle\n");
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

        if (state.armed != prev_armed) {
            if (state.armed) {
                motors.engage();
                logger::line("[motors] armed — driver enabled (STBY HIGH)");
            } else {
                motors.release();
                slew.reset();
                last_mc_us = 0;
                logger::line("[motors] disarmed — driver released (STBY LOW)");
            }
            prev_armed = state.armed;
        }

        if (state.armed && last_mc_us > 0
                && (now - last_mc_us) > Config::MC_TIMEOUT_US) {
            if (!mc_timeout_active) {
                mc_timeout_active = true;
                last_slew_tick_us = now;
                logger::line("[failsafe] MANUAL_CONTROL timeout — slewing to stop");
            }
            uint32_t elapsed_ms = static_cast<uint32_t>(
                std::min<uint64_t>((now - last_slew_tick_us) / 1000, 500));
            last_slew_tick_us = now;
            DriveOutput smoothed = slew.step({0, 0}, elapsed_ms);
            motors.set(smoothed.left, smoothed.right);
        } else {
            mc_timeout_active = false;
        }

        if (now - last_hb > Config::HEARTBEAT_INTERVAL_US) {
            mav.send_heartbeat(state);
            mav.send_sys_status(state);
            mav.send_current_mode(state);
            last_hb = now;
        }
    }
}
