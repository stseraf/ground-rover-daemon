#include "camera_handlers.hpp"
#include "mav_sender.hpp"
#include "rover_state.hpp"
#include "logger.hpp"

void handle_camera_request_message(MavSender& mav, const RoverState& state,
                                   const mavlink_command_long_t* cmd)
{
    auto msg_id = static_cast<uint32_t>(cmd->param1);
    mav.send_command_ack(state, cmd->command, MAV_RESULT_ACCEPTED,
                         cmd->target_system, cmd->target_component);
    switch (msg_id) {
        case MAVLINK_MSG_ID_CAMERA_INFORMATION:
            logger::line("tx: CAMERA_INFORMATION(259) — no capabilities");
            mav.send_camera_information(state);
            break;
        case MAVLINK_MSG_ID_CAMERA_SETTINGS:
            logger::line("tx: CAMERA_SETTINGS(260)");
            mav.send_camera_settings(state);
            break;
        case MAVLINK_MSG_ID_STORAGE_INFORMATION:
            logger::line("tx: STORAGE_INFORMATION(261)");
            mav.send_storage_information(state);
            break;
        case MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS:
            logger::line("tx: CAMERA_CAPTURE_STATUS(262)");
            mav.send_camera_capture_status(state);
            break;
        default: break;
    }
}

void handle_camera_command_long(MavSender& mav, const RoverState& state,
                                const mavlink_command_long_t* cmd)
{
    mav.send_command_ack(state, cmd->command, MAV_RESULT_ACCEPTED,
                         cmd->target_system, cmd->target_component);
    switch (cmd->command) {
        case MAV_CMD_REQUEST_CAMERA_INFORMATION:
            logger::line("tx: CAMERA_INFORMATION(259) — no capabilities");
            mav.send_camera_information(state);
            break;
        case MAV_CMD_REQUEST_CAMERA_SETTINGS:
            logger::line("tx: CAMERA_SETTINGS(260)");
            mav.send_camera_settings(state);
            break;
        case MAV_CMD_REQUEST_STORAGE_INFORMATION:
            logger::line("tx: STORAGE_INFORMATION(261)");
            mav.send_storage_information(state);
            break;
        default: break;
    }
}
