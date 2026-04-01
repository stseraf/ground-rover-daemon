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
#include "mavlink/param_store.hpp"
#include "mavlink/command_handlers.hpp"
#include "drive/diff_drive.hpp"
#include "motor/uart_motor_driver.hpp"
#ifdef DRIVER_TB6612
#include "motor/tb6612_driver.hpp"
#endif
#include "gimbal/stub_gimbal_controller.hpp"
#ifdef GIMBAL_I2C
#include "gimbal/i2c_gimbal_controller.hpp"
#endif
#include "gps/stub_gps_provider.hpp"
#ifdef GPS_NMEA
#include "gps/nmea_gps_provider.hpp"
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
    ParamStore params{};
    MavSender  mav{sock, Config::MAV_SYS_ID, Config::MAV_COMP_ID};

    {
        struct timespec ts;
        clock_gettime(CLOCK_BOOTTIME, &ts);
        logger::line("MAVLink rover daemon started at %lds%03ldms since boot",
                     ts.tv_sec, ts.tv_nsec / 1000000);
    }

#ifdef DRIVER_TB6612
    Tb6612Driver    motors{};
#else
    UartMotorDriver motors{};
#endif
#ifdef GIMBAL_I2C
    I2cGimbalController gimbal{};
#else
    StubGimbalController gimbal{};
#endif
#ifdef GPS_NMEA
    NmeaGpsProvider gps{Config::Gps::UART_DEV, Config::Gps::BAUD_RATE};
#else
    StubGpsProvider gps{};
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
                                int16_t  dead_zone = static_cast<int16_t>(params.get(0));
                                uint32_t slew_ms   = static_cast<uint32_t>(params.get(1));
                                DriveOutput raw      = compute_diff_drive(mc.x, mc.y, dead_zone);
                                int16_t out_left, out_right;
                                if (raw.left == 0 && raw.right == 0) {
                                    slew.reset();
                                    out_left = out_right = 0;
                                } else {
                                    DriveOutput smoothed = slew.step(raw, elapsed_ms, slew_ms);
                                    int16_t trim = static_cast<int16_t>(params.get(2));
                                    out_left  = clamp_axis(static_cast<int32_t>(smoothed.left)  + trim);
                                    out_right = clamp_axis(static_cast<int32_t>(smoothed.right) - trim);
                                }
                                // int ain1 = out_left  > 0 ? 1 : 0;
                                // int ain2 = out_left  < 0 ? 1 : 0;
                                // int bin1 = out_right > 0 ? 1 : 0;
                                // int bin2 = out_right < 0 ? 1 : 0;
                                // int pwma = (out_left  < 0 ? -out_left  : out_left)  / 10;
                                // int pwmb = (out_right < 0 ? -out_right : out_right) / 10;
                                // logger::same_line(
                                //     "rx: MC(69) x=%5d y=%5d z=%5d r=%5d btn=0x%02X"
                                //     " | A:IN=%d%d PWM=%3d%% | B:IN=%d%d PWM=%3d%% | STBY=H",
                                //     mc.x, mc.y, mc.z, mc.r, mc.buttons,
                                //     ain1, ain2, pwma, bin1, bin2, pwmb);
                                motors.set(out_left, out_right);
                                mav.send_servo_output_raw(state, out_left, out_right);
                            } else {
                                slew.reset();
                                // logger::same_line(
                                //     "rx: MC(69) x=%5d y=%5d z=%5d r=%5d btn=0x%02X"
                                //     " | A:IN=00 PWM=  0%% | B:IN=00 PWM=  0%% | STBY=L",
                                //     mc.x, mc.y, mc.z, mc.r, mc.buttons);
                                motors.stop();
                                mav.send_servo_output_raw(state, 0, 0);
                            }
                            // Gimbal: always active regardless of arm state
                            {
                                int16_t gdz_val = static_cast<int16_t>(Config::Gimbal::DEAD_ZONE);
                                auto gdz = [gdz_val](int16_t v) -> int16_t {
                                    return (v > -gdz_val && v < gdz_val) ? 0 : v;
                                };
                                gimbal.set(gdz(mc.x), gdz(mc.r));
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
                            logger::line("rx: MAVLINK_MSG_ID_PARAM_REQUEST_LIST(21) target=%u:%u", prl.target_system, prl.target_component);
                            for (uint16_t i = 0; i < ParamStore::COUNT; ++i)
                                mav.send_param(state, params.name(i), params.get(i), i, ParamStore::COUNT);
                            break;
                        }
                        case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
                            mavlink_param_request_read_t prr;
                            mavlink_msg_param_request_read_decode(&msg, &prr);
                            int idx = (prr.param_index >= 0)
                                      ? prr.param_index
                                      : params.find_by_name(prr.param_id);
                            if (idx >= 0 && idx < ParamStore::COUNT)
                                mav.send_param(state, params.name(idx), params.get(idx),
                                               static_cast<uint16_t>(idx), ParamStore::COUNT);
                            else
                                logger::line("rx: PARAM_REQUEST_READ(20): unknown param");
                            break;
                        }
                        case MAVLINK_MSG_ID_PARAM_SET: {
                            mavlink_param_set_t ps;
                            mavlink_msg_param_set_decode(&msg, &ps);
                            int idx = params.find_by_name(ps.param_id);
                            if (idx >= 0) {
                                params.set(static_cast<uint16_t>(idx), ps.param_value);
                                params.save(Config::PARAM_FILE);
                                mav.send_param(state, params.name(idx), params.get(idx),
                                               static_cast<uint16_t>(idx), ParamStore::COUNT);
                                logger::line("rx: PARAM_SET(23): %s=%.4f saved", params.name(idx), ps.param_value);
                            } else {
                                logger::line("rx: PARAM_SET(23): unknown param %.16s", ps.param_id);
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

        gps.update();
#ifdef GPS_NMEA
        gps.set_raw_log(params.get(4) != 0.0f);
#endif
        state.current_fix = gps.fix();
        if (!state.home_set && state.current_fix.valid) {
            state.home_alt_mm = state.current_fix.alt_mm;
            state.home_set    = true;
            logger::line("[gps] home altitude set: %.1f m MSL",
                         state.home_alt_mm / 1000.0);
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

        uint64_t mc_timeout_us = static_cast<uint64_t>(params.get(3)) * 1000;
        if (state.armed && last_mc_us > 0
                && (now - last_mc_us) > mc_timeout_us) {
            if (!mc_timeout_active) {
                mc_timeout_active = true;
                last_slew_tick_us = now;
                logger::line("[failsafe] MANUAL_CONTROL timeout — slewing to stop");
            }
            uint32_t elapsed_ms = static_cast<uint32_t>(
                std::min<uint64_t>((now - last_slew_tick_us) / 1000, 500));
            last_slew_tick_us = now;
            uint32_t slew_ms = static_cast<uint32_t>(params.get(1));
            DriveOutput smoothed = slew.step({0, 0}, elapsed_ms, slew_ms);
            int16_t trim = static_cast<int16_t>(params.get(2));
            motors.set(clamp_axis(static_cast<int32_t>(smoothed.left)  + trim),
                       clamp_axis(static_cast<int32_t>(smoothed.right) - trim));
        } else {
            mc_timeout_active = false;
        }

        if (now - last_hb > Config::HEARTBEAT_INTERVAL_US) {
            mav.send_heartbeat(state);
            mav.send_sys_status(state);
            mav.send_current_mode(state);
            mav.send_gps_raw_int(state);
            mav.send_global_position_int(state);
            last_hb = now;
        }
    }
}
