#include <cstdio>
#include <cstdlib>
#include <cerrno>
#include <cstring>
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
#include "mavlink/camera_handlers.hpp"
#include "camera/camera_discovery.hpp"
#include "video/rtsp_server.hpp"
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
#include "lte/stub_lte_monitor.hpp"
#ifdef LTE_USB
#include "lte/usb_lte_monitor.hpp"
#include "lte/link_switcher.hpp"
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
#ifdef __APPLE__
        clock_gettime(CLOCK_MONOTONIC, &ts);
#else
        clock_gettime(CLOCK_BOOTTIME, &ts);
#endif
        logger::line("MAVLink rover daemon started at %lds%03ldms since boot",
                     ts.tv_sec, ts.tv_nsec / 1000000);
        logger::line("[build] %s@%s", GIT_BRANCH, GIT_SHA);
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
#ifdef LTE_USB
    UsbLteMonitor lte{};
    LinkSwitcher  link_switcher{};
#else
    StubLteMonitor lte{};
#endif
    // Camera discovery — runs libcamera-hello --list-cameras at startup
    state.cameras = discover_cameras();
    state.video_bitrate_bps = static_cast<uint32_t>(params.get(5));
    state.video_fps         = static_cast<uint32_t>(params.get(6));

    // Embedded RTSP server — clients pull from rtsp://<rover>:8554/stream-N
    // (N = 1-indexed sensor mode). One mount per resolution gives QGC's
    // "gear" video-source dropdown a stream to switch between. libcamerasrc
    // is exclusive, so only one mount has an active pipeline at a time;
    // the switch costs ~1-2 s for camera release + reopen.
    RtspServer rtsp;
    if (!state.cameras.empty() && !state.cameras[0].modes.empty()) {
        const CameraInfo& cam = state.cameras[0];
        std::vector<RtspServer::Stream> streams;
        streams.reserve(cam.modes.size());
        for (size_t i = 0; i < cam.modes.size(); ++i) {
            const SensorMode& mode = cam.modes[i];
            int fps     = static_cast<int>(state.video_fps);
            int max_fps = static_cast<int>(mode.fps);
            if (fps > max_fps) fps = max_fps;
            char mount[32];
            std::snprintf(mount, sizeof(mount), "%s-%zu",
                          Config::RTSP_MOUNT_PREFIX, i + 1);
            streams.push_back({mount, mode.width, mode.height, fps,
                               state.video_bitrate_bps});
        }
        rtsp.start(Config::RTSP_PORT, streams);
    } else {
        logger::line("[rtsp] no camera detected — server not started");
    }

    DriveSlew slew{};
    uint64_t last_mc_us        = 0;
    uint64_t last_hb           = 0;
    uint64_t last_lte_poll     = 0;
    bool     mc_timeout_active = false;
    uint64_t last_slew_tick_us = 0;
    bool     prev_armed        = false;
    char     prev_uplink[8]{}; // edge-detect uplink type changes for STATUSTEXT
    int      wifi_drop_polls   = 0; // consecutive polls: NET_LINK_PREF=WiFi but rssi=0
    bool     banner_sent       = false; // one-shot per QGC session: STATUSTEXT banner
    bool     prev_gps_valid    = false; // edge-detect GPS fix acquired/lost
    uint64_t last_qgc_packet_us = 0;    // for reconnect detection (silence > QGC_RECONNECT_GAP_US)
    // LTE-only traffic accounting for the current daemon session.
    // usb0 /proc counters persist across daemon restarts and reset to 0 when
    // usb0 disappears, so we track deltas between polls and gate accumulation
    // on uplink == "lte" (WiFi uplink bytes are not billed).
    // Emission threshold (LTE_LOG_STEP_MB param) is applied to the current
    // session totals vs the totals at last emission, so changing the step
    // mid-session doesn't strand the counter on a stale boundary.
    uint64_t lte_session_rx_bytes = 0;
    uint64_t lte_session_tx_bytes = 0;
    uint64_t lte_last_rx_bytes    = 0;
    uint64_t lte_last_tx_bytes    = 0;
    uint64_t lte_rx_emitted_bytes = 0; // lte_session_rx_bytes at last emission
    uint64_t lte_tx_emitted_bytes = 0;
    bool     lte_baseline_set     = false;
    mavlink_message_t msg;
    mavlink_status_t  status;

    while (running) {
        // Detect QGC silence. state.qgc_known flips false once we cross the
        // threshold; next incoming packet treats the revival as a reconnect.
        if (state.qgc_known && last_qgc_packet_us != 0 &&
            (micros() - last_qgc_packet_us) > Config::QGC_RECONNECT_GAP_US) {
            logger::line("[qgc] losing qgc (no packets for %llu s)",
                         static_cast<unsigned long long>(
                             (micros() - last_qgc_packet_us) / 1'000'000ULL));
            state.qgc_known = false;
        }

        uint8_t receive_buffer[2048];
        ssize_t n = sock.recv(receive_buffer, sizeof(receive_buffer),
                              state.qgc_addr, state.qgc_addr_len);
        if (n > 0) {
            if (!state.qgc_known && last_qgc_packet_us != 0) {
                logger::line("[qgc] reconnected — re-arming status messages");
                banner_sent    = false;
                prev_gps_valid = false;
                prev_uplink[0] = '\0';
            }
            last_qgc_packet_us = micros();
            state.qgc_known = true;
            if (!banner_sent) {
                char banner[64];
                std::snprintf(banner, sizeof(banner), "Rover %s@%s", GIT_BRANCH, GIT_SHA);
                mav.send_statustext(state, MAV_SEVERITY_INFO, banner);
                if (!state.cameras.empty() && !state.cameras[0].modes.empty()) {
                    char stxt[64];
                    std::snprintf(stxt, sizeof(stxt),
                                  "RTSP :%u%s-{1..%zu}",
                                  Config::RTSP_PORT, Config::RTSP_MOUNT_PREFIX,
                                  state.cameras[0].modes.size());
                    mav.send_statustext(state, MAV_SEVERITY_INFO, stxt);
                }
                banner_sent = true;
            }
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
                            // Route camera commands to the camera handler;
                            // everything else goes to the autopilot handler.
                            int cam_idx = -1;
                            uint8_t tc = cmd.target_component;
                            if (!state.cameras.empty() &&
                                tc >= MAV_COMP_ID_CAMERA &&
                                tc < static_cast<uint8_t>(MAV_COMP_ID_CAMERA +
                                                           state.cameras.size())) {
                                cam_idx = tc - MAV_COMP_ID_CAMERA;
                            }
                            // QGC sometimes sends camera REQUEST_MESSAGEs to comp 1;
                            // intercept and route to first camera component.
                            if (cam_idx < 0 && !state.cameras.empty() &&
                                cmd.command == MAV_CMD_REQUEST_MESSAGE) {
                                uint32_t mid = static_cast<uint32_t>(cmd.param1);
                                if (mid == MAVLINK_MSG_ID_CAMERA_INFORMATION ||
                                    mid == MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION ||
                                    mid == MAVLINK_MSG_ID_VIDEO_STREAM_STATUS)
                                    cam_idx = 0;
                            }
                            if (cam_idx >= 0)
                                handle_camera_command_long(mav, state, &cmd, cam_idx);
                            else
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
                                if (std::strncmp(params.name(idx), "VIDEO_BITRATE", 16) == 0)
                                    state.video_bitrate_bps =
                                        static_cast<uint32_t>(params.get(idx));
                                if (std::strncmp(params.name(idx), "VIDEO_FPS", 16) == 0)
                                    state.video_fps =
                                        static_cast<uint32_t>(params.get(idx));
#ifdef LTE_USB
                                if (std::strncmp(params.name(idx), "NET_LINK_PREF", 16) == 0) {
                                    int pref = static_cast<int>(params.get(idx));
                                    wifi_drop_polls = 0; // reset fallback counter on manual change
                                    if (pref == 1)      link_switcher.send("wifi");
                                    else if (pref == 2) link_switcher.send("lte");
                                }
#endif
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
        if (state.qgc_known && state.current_fix.valid != prev_gps_valid) {
            if (state.current_fix.valid) {
                char stxt[64];
                std::snprintf(stxt, sizeof(stxt), "GPS fix: %u sats, HDOP %.1f",
                              state.current_fix.satellites,
                              state.current_fix.hdop_100 / 100.0);
                mav.send_statustext(state, MAV_SEVERITY_INFO, stxt);
            } else {
                mav.send_statustext(state, MAV_SEVERITY_WARNING, "GPS fix lost");
            }
            prev_gps_valid = state.current_fix.valid;
        }

        uint64_t now = micros();

        if (now - last_lte_poll > Config::Lte::POLL_INTERVAL_US) {
            lte.update();
            state.lte    = lte.status();
            last_lte_poll = now;

            // Accumulate usb0 bytes only while LTE is the active uplink.
            // Baseline is invalidated on usb0 disappearance so the kernel's
            // fresh counters on re-attach don't show up as a huge delta.
            bool lte_uplink_active = state.lte.connected &&
                                     ::strncmp(state.lte.uplink, "lte", 3) == 0;
            if (state.lte.present) {
                if (lte_baseline_set && lte_uplink_active) {
                    if (state.lte.rx_bytes >= lte_last_rx_bytes)
                        lte_session_rx_bytes += state.lte.rx_bytes - lte_last_rx_bytes;
                    if (state.lte.tx_bytes >= lte_last_tx_bytes)
                        lte_session_tx_bytes += state.lte.tx_bytes - lte_last_tx_bytes;
                }
                lte_last_rx_bytes = state.lte.rx_bytes;
                lte_last_tx_bytes = state.lte.tx_bytes;
                lte_baseline_set  = true;
            } else {
                lte_baseline_set = false;
            }

            int step_mb = static_cast<int>(params.get(8));
            if (step_mb > 0) {
                uint64_t step_bytes =
                    static_cast<uint64_t>(step_mb) * 1000ULL * 1000ULL;
                if (lte_session_rx_bytes / step_bytes >
                        lte_rx_emitted_bytes / step_bytes ||
                    lte_session_tx_bytes / step_bytes >
                        lte_tx_emitted_bytes / step_bytes) {
                    char msg[64];
                    std::snprintf(msg, sizeof(msg),
                                  "LTE session: rx %.1f MB tx %.1f MB",
                                  lte_session_rx_bytes / 1e6,
                                  lte_session_tx_bytes / 1e6);
                    logger::line("[lte] %s", msg);
                    if (state.qgc_known)
                        mav.send_statustext(state, MAV_SEVERITY_INFO, msg);
                    lte_rx_emitted_bytes = lte_session_rx_bytes;
                    lte_tx_emitted_bytes = lte_session_tx_bytes;
                }
            }

            // Skip transient "unknown" during a switch (routes/iptables mid-swap)
            // to avoid spurious duplicate LTE notifications.
            bool uplink_is_real = ::strncmp(state.lte.uplink, "wifi", 4) == 0 ||
                                  ::strncmp(state.lte.uplink, "lte", 3) == 0;

            // Notify operator when active uplink type changes (LTE ↔ WiFi ↔ disconnected)
            if (state.qgc_known && uplink_is_real &&
                ::strncmp(state.lte.uplink, prev_uplink, sizeof(prev_uplink)) != 0) {
                char stxt[50]{};
                if (!state.lte.connected) {
                    ::snprintf(stxt, sizeof(stxt), "Link: disconnected");
                    mav.send_statustext(state, MAV_SEVERITY_WARNING, stxt);
                } else if (::strncmp(state.lte.uplink, "wifi", 4) == 0) {
                    if (state.lte.wifi_rssi_dbm != 0)
                        ::snprintf(stxt, sizeof(stxt), "Link: WiFi %d dBm",
                                   state.lte.wifi_rssi_dbm);
                    else
                        ::snprintf(stxt, sizeof(stxt), "Link: WiFi");
                    mav.send_statustext(state, MAV_SEVERITY_INFO, stxt);
                } else {
                    ::snprintf(stxt, sizeof(stxt), "Link: LTE %.23s %.7s",
                               state.lte.oper, state.lte.netmode);
                    mav.send_statustext(state, MAV_SEVERITY_INFO, stxt);
                }
            }

            // Kernel WG session gets stuck when the NAT path changes mid-flight;
            // bouncing wg0 forces a fresh handshake on the new uplink.
            if (prev_uplink[0] != '\0' && state.lte.connected && uplink_is_real &&
                ::strncmp(state.lte.uplink, prev_uplink, sizeof(prev_uplink)) != 0) {
                logger::line("[wg] uplink %s→%s — restarting wg0",
                             prev_uplink, state.lte.uplink);
                int rc = std::system("(sudo wg-quick down wg0 && sudo wg-quick up wg0) "
                                     ">>/tmp/wg-restart.log 2>&1 &");
                (void)rc;
            }

            if (uplink_is_real) {
                ::strncpy(prev_uplink, state.lte.uplink, sizeof(prev_uplink) - 1);
                prev_uplink[sizeof(prev_uplink) - 1] = '\0';
            }

#ifdef LTE_USB
            // Auto-fallback: WiFi preferred but wpa_supplicant lost association
            // (wpa_cli signal_poll returns FAIL → wifi_rssi_dbm stays 0 in status server).
            // After 3 consecutive polls (~15 s) with no WiFi signal, switch to LTE.
            if (static_cast<int>(params.get(7)) == 1 &&
                ::strncmp(state.lte.uplink, "wifi", 4) == 0 &&
                state.lte.wifi_rssi_dbm == 0 &&
                state.lte.connected) {
                if (++wifi_drop_polls >= 3) {
                    logger::line("[lte] WiFi signal lost (%d polls) — fallback to LTE", wifi_drop_polls);
                    link_switcher.send("lte");
                    if (state.qgc_known)
                        mav.send_statustext(state, MAV_SEVERITY_WARNING,
                                            "Link: WiFi lost, LTE fallback");
                    params.set(7, 0.0f);
                    params.save(Config::PARAM_FILE);
                    wifi_drop_polls = 0;
                }
            } else {
                wifi_drop_polls = 0;
            }
#endif
        }

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

        // LTE failsafe: stop motors if connection drops while armed
        {
            bool lte_just_dropped = state.lte_was_connected && !state.lte.connected;
            if (lte_just_dropped && state.armed) {
                logger::line("[lte] connection lost — stopping motors");
                state.lte_failsafe_active = true;
            }
            if (state.lte.connected)
                state.lte_failsafe_active = false;

            if (state.lte_failsafe_active && state.armed) {
                uint32_t elapsed_ms = static_cast<uint32_t>(
                    std::min<uint64_t>((now - last_slew_tick_us) / 1000, 500));
                last_slew_tick_us = now;
                uint32_t slew_ms = static_cast<uint32_t>(params.get(1));
                DriveOutput smoothed = slew.step({0, 0}, elapsed_ms, slew_ms);
                int16_t trim = static_cast<int16_t>(params.get(2));
                motors.set(clamp_axis(static_cast<int32_t>(smoothed.left)  + trim),
                           clamp_axis(static_cast<int32_t>(smoothed.right) - trim));
            }
            state.lte_was_connected = state.lte.connected;
        }

        if (now - last_hb > Config::HEARTBEAT_INTERVAL_US) {
            mav.send_heartbeat(state);
            mav.send_sys_status(state);
            mav.send_current_mode(state);
            mav.send_gps_raw_int(state);
            mav.send_global_position_int(state);
            mav.send_radio_status(state);
            // One heartbeat per discovered camera component
            for (int i = 0; i < static_cast<int>(state.cameras.size()); ++i)
                mav.send_camera_heartbeat(
                    static_cast<uint8_t>(MAV_COMP_ID_CAMERA + i), state);
            last_hb = now;
        }
    }

    rtsp.stop();
}
