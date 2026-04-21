#include "command_handlers.hpp"
#include "mav_sender.hpp"
#include "rover_state.hpp"
#include "logger.hpp"

#include <cstdio>

namespace {

void handle_arm_disarm(MavSender& mav, RoverState& state,
                       const mavlink_command_long_t* cmd)
{
    state.armed = (cmd->param1 > 0.5f);
    logger::line("rx: MAVLINK_MSG_ID_COMMAND_LONG(76): MAV_CMD_COMPONENT_ARM_DISARM(400): Arm=%d",
                 static_cast<uint32_t>(cmd->param1));
    mav.send_command_ack(state, cmd->command, MAV_RESULT_ACCEPTED,
                         cmd->target_system, cmd->target_component);
}

void handle_request_message(MavSender& mav, RoverState& state,
                             const mavlink_command_long_t* cmd)
{
    auto msg_id = static_cast<uint32_t>(cmd->param1);

    // Silent ACK for known-unsupported IDs — no log to avoid terminal spam
    switch (msg_id) {
        case MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION: // 280 — polled ~1 Hz by QGC
        case MAVLINK_MSG_ID_COMPONENT_INFORMATION:      // 395
        case MAVLINK_MSG_ID_COMPONENT_METADATA:         // 397
            mav.send_command_ack(state, cmd->command, MAV_RESULT_UNSUPPORTED,
                                 cmd->target_system, cmd->target_component);
            return;
        default: break;
    }

    logger::line("rx: MAVLINK_MSG_ID_COMMAND_LONG(76): MAV_CMD_REQUEST_MESSAGE(512): msg_id=%u", msg_id);

    switch (msg_id) {
        case MAVLINK_MSG_ID_SYS_STATUS:
            mav.send_command_ack(state, cmd->command, MAV_RESULT_ACCEPTED,
                                 cmd->target_system, cmd->target_component);
            mav.send_sys_status(state);
            break;
        case MAVLINK_MSG_ID_AUTOPILOT_VERSION:
            mav.send_command_ack(state, cmd->command, MAV_RESULT_ACCEPTED,
                                 cmd->target_system, cmd->target_component);
            mav.send_autopilot_version(state);
            break;
        case MAVLINK_MSG_ID_PROTOCOL_VERSION:
            mav.send_command_ack(state, cmd->command, MAV_RESULT_ACCEPTED,
                                 cmd->target_system, cmd->target_component);
            mav.send_protocol_version(state);
            break;
        case MAVLINK_MSG_ID_AVAILABLE_MODES:
            mav.send_command_ack(state, cmd->command, MAV_RESULT_ACCEPTED,
                                 cmd->target_system, cmd->target_component);
            mav.send_available_modes(state, static_cast<uint32_t>(cmd->param2));
            break;
        default:
            logger::line("Unknown REQUEST_MESSAGE(%u): MAV_RESULT_UNSUPPORTED", msg_id);
            mav.send_command_ack(state, cmd->command, MAV_RESULT_UNSUPPORTED,
                                 cmd->target_system, cmd->target_component);
            break;
    }
}

void handle_set_mode(MavSender& mav, RoverState& state,
                     const mavlink_command_long_t* cmd)
{
    uint8_t  base_mode   = static_cast<uint8_t>(cmd->param1);
    uint32_t custom_mode = static_cast<uint32_t>(cmd->param2);
    logger::line("rx: MAVLINK_MSG_ID_COMMAND_LONG(76): MAV_CMD_DO_SET_MODE(176): base_mode=0x%02X custom_mode=%u",
                 base_mode, custom_mode);
    if (!rover_mode_supported(custom_mode)) {
        logger::line("    DENIED: ROVER_MODE %u not implemented", custom_mode);
        mav.send_command_ack(state, cmd->command, MAV_RESULT_DENIED,
                             cmd->target_system, cmd->target_component);
        return;
    }
    state.base_mode   = base_mode;
    state.custom_mode = custom_mode;
    mav.send_command_ack(state, cmd->command, MAV_RESULT_ACCEPTED,
                         cmd->target_system, cmd->target_component);
    mav.send_current_mode(state);
}

} // namespace

void handle_command_long(MavSender& mav, RoverState& state,
                         const mavlink_command_long_t* cmd)
{
    switch (cmd->command) {
        case MAV_CMD_COMPONENT_ARM_DISARM:
            handle_arm_disarm(mav, state, cmd);
            break;
        case MAV_CMD_REQUEST_MESSAGE:
            handle_request_message(mav, state, cmd);
            break;
        case MAV_CMD_DO_SET_MODE:
            handle_set_mode(mav, state, cmd);
            break;
        case MAV_CMD_NAV_TAKEOFF:
            logger::line("rx: MAVLINK_MSG_ID_COMMAND_LONG(76): MAV_CMD_NAV_TAKEOFF(22): MAV_RESULT_UNSUPPORTED");
            mav.send_command_ack(state, cmd->command, MAV_RESULT_UNSUPPORTED,
                                 cmd->target_system, cmd->target_component);
            break;
        case MAV_CMD_MISSION_START:
            logger::line("rx: MAVLINK_MSG_ID_COMMAND_LONG(76): MAV_CMD_MISSION_START(300): MAV_RESULT_UNSUPPORTED");
            mav.send_command_ack(state, cmd->command, MAV_RESULT_UNSUPPORTED,
                                 cmd->target_system, cmd->target_component);
            break;
        case MAV_CMD_REQUEST_CAMERA_INFORMATION: // deprecated, misdirected to autopilot
        case MAV_CMD_SET_CAMERA_ZOOM:
        case MAV_CMD_SET_CAMERA_FOCUS:
            mav.send_command_ack(state, cmd->command, MAV_RESULT_UNSUPPORTED,
                                 cmd->target_system, cmd->target_component);
            break;
        default:
            logger::line("rx: MAVLINK_MSG_ID_COMMAND_LONG(76): Unknown COMMAND_LONG(%u): MAV_RESULT_UNSUPPORTED",
                         static_cast<uint32_t>(cmd->command));
            mav.send_command_ack(state, cmd->command, MAV_RESULT_UNSUPPORTED,
                                 cmd->target_system, cmd->target_component);
            break;
    }
}
