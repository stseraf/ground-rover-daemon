#include "camera_handlers.hpp"
#include "mav_sender.hpp"
#include "rover_state.hpp"
#include "logger.hpp"

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

    // Silently reject polled-but-unsupported messages
    if (msg_id == MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION) {
        mav.send_command_ack_cam(cc, state, cmd->command, MAV_RESULT_UNSUPPORTED,
                                 cmd->target_system, cmd->target_component);
        return;
    }

    mav.send_command_ack_cam(cc, state, cmd->command, MAV_RESULT_ACCEPTED,
                             cmd->target_system, cmd->target_component);

    logger::line("rx: camera REQUEST_MESSAGE(%u) cam=%d", msg_id, cam_idx);

    switch (msg_id) {
        case MAVLINK_MSG_ID_CAMERA_INFORMATION:
            logger::line("tx: CAMERA_INFORMATION(259) cam=%d", cam_idx);
            mav.send_camera_information(cc, state, cam);
            break;

        case MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION:
            logger::line("tx: VIDEO_STREAM_INFORMATION(269) cam=%d %zu stream(s)",
                         cam_idx, cam.modes.size());
            for (int i = 0; i < static_cast<int>(cam.modes.size()); ++i)
                mav.send_video_stream_information(cc, state, cam, cam_idx, i);
            break;

        case MAVLINK_MSG_ID_VIDEO_STREAM_STATUS: {
            auto stream_id = static_cast<uint8_t>(cmd->param2);
            if (stream_id == 0) {
                // All streams
                logger::line("tx: VIDEO_STREAM_STATUS(270) cam=%d all streams", cam_idx);
                for (int i = 0; i < static_cast<int>(cam.modes.size()); ++i)
                    mav.send_video_stream_status(cc, state, cam, cam_idx,
                                                  static_cast<uint8_t>(i + 1));
            } else {
                logger::line("tx: VIDEO_STREAM_STATUS(270) cam=%d stream=%u", cam_idx, stream_id);
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
            logger::line("tx: camera REQUEST_MESSAGE(%u) unsupported", msg_id);
            break;
    }
}

// ── Command-long handler ───────────────────────────────────────────────────

void handle_camera_command_long(MavSender& mav, const RoverState& state,
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

        // Video delivery is via the embedded RTSP server (pull model). These
        // commands are QGC-initiated "streaming on/off" hints; we ACK so the
        // UI stays happy, but the RTSP factory starts/stops the pipeline on
        // its own client-connect/-disconnect events.
        case MAV_CMD_VIDEO_START_STREAMING:
            logger::line("rx: VIDEO_START_STREAMING cam=%d (rtsp: no-op, client-driven)",
                         cam_idx);
            mav.send_command_ack_cam(cc, state, cmd->command, MAV_RESULT_ACCEPTED,
                                     cmd->target_system, cmd->target_component);
            break;

        case MAV_CMD_VIDEO_STOP_STREAMING:
            logger::line("rx: VIDEO_STOP_STREAMING cam=%d (rtsp: no-op, client-driven)",
                         cam_idx);
            mav.send_command_ack_cam(cc, state, cmd->command, MAV_RESULT_ACCEPTED,
                                     cmd->target_system, cmd->target_component);
            break;

        case MAV_CMD_RESET_CAMERA_SETTINGS:
            logger::line("rx: RESET_CAMERA_SETTINGS(529) cam=%d", cam_idx);
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
