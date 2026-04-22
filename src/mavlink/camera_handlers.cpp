#include "camera_handlers.hpp"
#include "mav_sender.hpp"
#include "rover_state.hpp"
#include "logger.hpp"
#include "camera/gst_pipeline.hpp"

#include <cmath>
#include <sys/wait.h>

// ── Helper ─────────────────────────────────────────────────────────────────

static uint8_t cam_comp(int cam_idx)
{
    return static_cast<uint8_t>(MAV_COMP_ID_CAMERA + cam_idx);
}

// ── Request-message handler ────────────────────────────────────────────────

void handle_camera_request_message(MavSender& mav, const RoverState& state,
                                   const mavlink_command_long_t* cmd,
                                   int cam_idx)
{
    auto     msg_id      = static_cast<uint32_t>(cmd->param1);
    uint8_t  cc          = cam_comp(cam_idx);
    const CameraInfo& cam = state.cameras[cam_idx];

    // GIMBAL_MANAGER_INFORMATION: QGC probes every camera component for one.
    // Reply ACCEPTED + "no gimbal" stub so GimbalController stops its 6×
    // retry loop at connect.
    if (msg_id == MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION) {
        mav.send_command_ack_cam(cc, state, cmd->command, MAV_RESULT_ACCEPTED,
                                 cmd->target_system, cmd->target_component);
        mav.send_gimbal_manager_information(state, cc);
        return;
    }

    mav.send_command_ack_cam(cc, state, cmd->command, MAV_RESULT_ACCEPTED,
                             cmd->target_system, cmd->target_component);

    // Expected polls (CAMERA_INFORMATION, VIDEO_STREAM_INFORMATION, ...) are
    // sent back via mav_sender, which logs there. We only log at this layer
    // when the msg_id falls into the default / unsupported case.
    switch (msg_id) {
        case MAVLINK_MSG_ID_CAMERA_INFORMATION:
            mav.send_camera_information(cc, state, cam);
            break;

        case MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION:
            for (int i = 0; i < static_cast<int>(cam.modes.size()); ++i)
                mav.send_video_stream_information(cc, state, cam, cam_idx, i);
            break;

        case MAVLINK_MSG_ID_VIDEO_STREAM_STATUS: {
            auto stream_id = static_cast<uint8_t>(cmd->param2);
            if (stream_id == 0) {
                for (int i = 0; i < static_cast<int>(cam.modes.size()); ++i)
                    mav.send_video_stream_status(cc, state, cam, cam_idx,
                                                  static_cast<uint8_t>(i + 1));
            } else {
                mav.send_video_stream_status(cc, state, cam, cam_idx, stream_id);
            }
            break;
        }

        case MAVLINK_MSG_ID_CAMERA_SETTINGS:
            mav.send_camera_settings(cc, state);
            break;
        case MAVLINK_MSG_ID_STORAGE_INFORMATION:
            mav.send_storage_information(cc, state);
            break;
        case MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS:
            mav.send_camera_capture_status(cc, state);
            break;

        default:
            logger::line("rx: camera REQUEST_MESSAGE(%u) cam=%d — unsupported",
                         msg_id, cam_idx);
            break;
    }
}

// ── Command-long handler ───────────────────────────────────────────────────

void handle_camera_command_long(MavSender& mav, RoverState& state,
                                const mavlink_command_long_t* cmd,
                                int cam_idx)
{
    uint8_t          cc  = cam_comp(cam_idx);
    const CameraInfo& cam = state.cameras[cam_idx];

    switch (cmd->command) {

        // Modern: REQUEST_MESSAGE dispatches to the request handler above
        case MAV_CMD_REQUEST_MESSAGE:
            handle_camera_request_message(mav, state, cmd, cam_idx);
            return;  // ACK is sent inside handle_camera_request_message

        // Deprecated: QGC fallback if REQUEST_MESSAGE times out
        case MAV_CMD_REQUEST_CAMERA_INFORMATION:
            logger::line("rx: deprecated REQUEST_CAMERA_INFORMATION cam=%d", cam_idx);
            mav.send_command_ack_cam(cc, state, cmd->command, MAV_RESULT_ACCEPTED,
                                     cmd->target_system, cmd->target_component);
            mav.send_camera_information(cc, state, cam);
            break;

        case MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
            logger::line("rx: deprecated REQUEST_VIDEO_STREAM_INFORMATION cam=%d", cam_idx);
            mav.send_command_ack_cam(cc, state, cmd->command, MAV_RESULT_ACCEPTED,
                                     cmd->target_system, cmd->target_component);
            for (int i = 0; i < static_cast<int>(cam.modes.size()); ++i)
                mav.send_video_stream_information(cc, state, cam, cam_idx, i);
            break;

        case MAV_CMD_REQUEST_CAMERA_SETTINGS:
            logger::line("rx: REQUEST_CAMERA_SETTINGS cam=%d", cam_idx);
            mav.send_command_ack_cam(cc, state, cmd->command, MAV_RESULT_ACCEPTED,
                                     cmd->target_system, cmd->target_component);
            mav.send_camera_settings(cc, state);
            break;

        case MAV_CMD_REQUEST_STORAGE_INFORMATION:
            logger::line("rx: REQUEST_STORAGE_INFORMATION cam=%d", cam_idx);
            mav.send_command_ack_cam(cc, state, cmd->command, MAV_RESULT_ACCEPTED,
                                     cmd->target_system, cmd->target_component);
            mav.send_storage_information(cc, state);
            break;

        case MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
            logger::line("rx: REQUEST_CAMERA_CAPTURE_STATUS cam=%d", cam_idx);
            mav.send_command_ack_cam(cc, state, cmd->command, MAV_RESULT_ACCEPTED,
                                     cmd->target_system, cmd->target_component);
            mav.send_camera_capture_status(cc, state);
            break;

        case MAV_CMD_REQUEST_VIDEO_STREAM_STATUS: {
            auto stream_id = static_cast<uint8_t>(cmd->param1);
            logger::line("rx: deprecated REQUEST_VIDEO_STREAM_STATUS cam=%d stream=%u",
                         cam_idx, stream_id);
            mav.send_command_ack_cam(cc, state, cmd->command, MAV_RESULT_ACCEPTED,
                                     cmd->target_system, cmd->target_component);
            if (stream_id == 0) {
                for (int i = 0; i < static_cast<int>(cam.modes.size()); ++i)
                    mav.send_video_stream_status(cc, state, cam, cam_idx,
                                                  static_cast<uint8_t>(i + 1));
            } else {
                mav.send_video_stream_status(cc, state, cam, cam_idx, stream_id);
            }
            break;
        }

        case MAV_CMD_VIDEO_START_STREAMING: {
            auto stream_id = static_cast<uint8_t>(cmd->param1);
            int  mode_idx  = stream_id - 1;

            logger::line("rx: VIDEO_START_STREAMING cam=%d stream=%u", cam_idx, stream_id);

            if (mode_idx < 0 || mode_idx >= static_cast<int>(cam.modes.size())) {
                logger::line("[gst] invalid stream_id %u (cam has %zu modes)",
                             stream_id, cam.modes.size());
                mav.send_command_ack_cam(cc, state, cmd->command, MAV_RESULT_DENIED,
                                         cmd->target_system, cmd->target_component);
                break;
            }

            if (!state.qgc_ip_known) {
                logger::line("[gst] cannot start — QGC IP not yet known");
                mav.send_command_ack_cam(cc, state, cmd->command,
                                         MAV_RESULT_TEMPORARILY_REJECTED,
                                         cmd->target_system, cmd->target_component);
                break;
            }

            // Stop any running stream first
            if (state.active_gst_pid > 0) {
                logger::line("[gst] stopping previous stream (PID %d)", state.active_gst_pid);
                gst_kill(state.active_gst_pid);
                state.active_gst_pid  = -1;
                state.active_cam_idx  = -1;
                state.active_mode_idx = -1;
            }

            const SensorMode& mode = cam.modes[mode_idx];
            int fps = static_cast<int>(state.video_fps);
            int max_fps = static_cast<int>(std::floor(mode.fps));
            if (fps > max_fps) {
                logger::line("[gst] VIDEO_FPS %d exceeds mode max %.2f, clamping to %d",
                             fps, mode.fps, max_fps);
                fps = max_fps;
            }
            state.active_gst_pid = gst_spawn(mode.width, mode.height, fps,
                                              state.video_bitrate_bps, state.qgc_ip);
            if (state.active_gst_pid > 0) {
                state.active_cam_idx  = cam_idx;
                state.active_mode_idx = mode_idx;
                state.gst_retry_count = 0;
                state.gst_gave_up     = false;
                logger::line("[gst] started: %s", mode.name);
                char stxt[64];
                std::snprintf(stxt, sizeof(stxt), "Stream start: %ux%u %d/%dfps %ukbps",
                              mode.width, mode.height, fps, static_cast<int>(mode.fps),
                              state.video_bitrate_bps / 1000);
                mav.send_statustext(state, MAV_SEVERITY_INFO, stxt);
                mav.send_command_ack_cam(cc, state, cmd->command, MAV_RESULT_ACCEPTED,
                                         cmd->target_system, cmd->target_component);
            } else {
                mav.send_statustext(state, MAV_SEVERITY_ERROR, "Video stream failed to start");
                mav.send_command_ack_cam(cc, state, cmd->command, MAV_RESULT_FAILED,
                                         cmd->target_system, cmd->target_component);
            }
            break;
        }

        case MAV_CMD_RESET_CAMERA_SETTINGS: {
            logger::line("rx: RESET_CAMERA_SETTINGS(529) cam=%d — stopping stream", cam_idx);

            if (state.active_gst_pid > 0) {
                logger::line("[gst] stopping stream (PID %d)", state.active_gst_pid);
                gst_kill(state.active_gst_pid);
                state.active_gst_pid  = -1;
                state.active_cam_idx  = -1;
                state.active_mode_idx = -1;
                logger::line("[gst] stream stopped");
                mav.send_statustext(state, MAV_SEVERITY_INFO, "Video stream stopped");
            }
            mav.send_command_ack_cam(cc, state, cmd->command, MAV_RESULT_ACCEPTED,
                                     cmd->target_system, cmd->target_component);
            break;
        }

        case MAV_CMD_VIDEO_STOP_STREAMING:
            logger::line("rx: VIDEO_STOP_STREAMING cam=%d", cam_idx);
            if (state.active_gst_pid > 0) {
                gst_kill(state.active_gst_pid);
                state.active_gst_pid  = -1;
                state.active_cam_idx  = -1;
                state.active_mode_idx = -1;
                logger::line("[gst] stream stopped");
                mav.send_statustext(state, MAV_SEVERITY_INFO, "Video stream stopped");
            }
            mav.send_command_ack_cam(cc, state, cmd->command, MAV_RESULT_ACCEPTED,
                                     cmd->target_system, cmd->target_component);
            break;

        default:
            logger::line("rx: camera COMMAND_LONG(%u) unsupported cam=%d",
                         static_cast<unsigned>(cmd->command), cam_idx);
            mav.send_command_ack_cam(cc, state, cmd->command, MAV_RESULT_UNSUPPORTED,
                                     cmd->target_system, cmd->target_component);
            break;
    }
}

// ── Stream health monitor ──────────────────────────────────────────────────

void gst_monitor_tick(MavSender& mav, RoverState& state, uint64_t /*now*/)
{
    if (state.active_gst_pid <= 0 || state.gst_gave_up) return;

    int   wstatus = 0;
    pid_t result  = waitpid(state.active_gst_pid, &wstatus, WNOHANG);
    if (result == 0) return;  // still running

    logger::line("[gst] PID %d died (waitpid=%d) retry=%d mode=%d",
                 state.active_gst_pid, static_cast<int>(result),
                 state.gst_retry_count, state.active_mode_idx);
    state.active_gst_pid = -1;  // already gone — skip gst_kill

    if (state.active_cam_idx < 0 ||
        state.active_cam_idx >= static_cast<int>(state.cameras.size()) ||
        state.cameras[state.active_cam_idx].modes.empty()) {
        logger::line("[gst] monitor: invalid camera state — giving up");
        mav.send_statustext(state, MAV_SEVERITY_CRITICAL, "Video stream lost, no camera");
        state.gst_gave_up = true;
        return;
    }
    const CameraInfo& cam = state.cameras[state.active_cam_idx];

    auto spawn_mode = [&](int mode_idx) -> pid_t {
        const SensorMode& mode = cam.modes[mode_idx];
        int fps     = static_cast<int>(state.video_fps);
        int max_fps = static_cast<int>(std::floor(mode.fps));
        if (fps > max_fps) fps = max_fps;
        return gst_spawn(mode.width, mode.height, fps,
                         state.video_bitrate_bps, state.qgc_ip);
    };

    if (state.gst_retry_count < 3) {
        state.gst_retry_count += 1;
        const SensorMode& cur = cam.modes[state.active_mode_idx];
        int eff_fps = std::min(static_cast<int>(state.video_fps),
                               static_cast<int>(cur.fps));
        logger::line("[gst] monitor: retry %d/3 mode=%d (%s)",
                     state.gst_retry_count, state.active_mode_idx, cur.name);
        char stxt[50];
        std::snprintf(stxt, sizeof(stxt), "Stream retry (%d/3): %d/%dfps",
                      state.gst_retry_count, eff_fps, static_cast<int>(cur.fps));
        mav.send_statustext(state, MAV_SEVERITY_WARNING, stxt);
        state.active_gst_pid = spawn_mode(state.active_mode_idx);
        return;
    }

    if (state.active_mode_idx != 0) {
        logger::line("[gst] monitor: 3 failures on mode=%d — falling back to mode 0",
                     state.active_mode_idx);
        const SensorMode& fb = cam.modes[0];
        int eff_fps = std::min(static_cast<int>(state.video_fps),
                               static_cast<int>(fb.fps));
        char stxt[64];
        std::snprintf(stxt, sizeof(stxt), "Stream fallback: %ux%u %d/%dfps %ukbps",
                      fb.width, fb.height, eff_fps, static_cast<int>(fb.fps),
                      state.video_bitrate_bps / 1000);
        mav.send_statustext(state, MAV_SEVERITY_WARNING, stxt);
        state.active_mode_idx = 0;
        state.gst_retry_count = 0;
        state.active_gst_pid  = spawn_mode(0);
        return;
    }

    logger::line("[gst] monitor: 3 failures on mode 0 — giving up");
    mav.send_statustext(state, MAV_SEVERITY_CRITICAL, "Video stream failed, gave up");
    state.gst_gave_up     = true;
    state.active_cam_idx  = -1;
    state.active_mode_idx = -1;
}
