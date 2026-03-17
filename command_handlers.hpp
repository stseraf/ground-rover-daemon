#pragma once

#include <cstdio>

#include "mav_sender.hpp"
#include "rover_state.hpp"
#include "logger.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include "external/mavlink/common/mavlink.h"
#pragma GCC diagnostic pop

/* ---- individual command handlers ---- */

inline void handle_arm_disarm(MavSender& mav, RoverState& state,
                               const mavlink_command_long_t *cmd)
{
    state.armed = (cmd->param1 > 0.5f);
    logger::line("rx: MAVLINK_MSG_ID_COMMAND_LONG(76): MAV_CMD_COMPONENT_ARM_DISARM(400): Arm=%d",
                 static_cast<uint32_t>(cmd->param1));
    mav.send_command_ack(state, cmd->command, MAV_RESULT_ACCEPTED,
                         cmd->target_system, cmd->target_component);
}

inline void handle_request_message(MavSender& mav, RoverState& state,
                                    const mavlink_command_long_t *cmd)
{
    logger::line("rx: MAVLINK_MSG_ID_COMMAND_LONG(76): MAV_CMD_REQUEST_MESSAGE(512): ");
    auto msg_id = static_cast<uint32_t>(cmd->param1);

    switch (msg_id) {
        case MAVLINK_MSG_ID_SYS_STATUS:
            std::printf("MAVLINK_MSG_ID_SYS_STATUS(1) handle\n");
            mav.send_command_ack(state, cmd->command, MAV_RESULT_ACCEPTED,
                                 cmd->target_system, cmd->target_component);
            mav.send_sys_status(state);
            break;
        case MAVLINK_MSG_ID_AUTOPILOT_VERSION:
            std::printf("MAVLINK_MSG_ID_AUTOPILOT_VERSION(148) handle\n");
            mav.send_command_ack(state, cmd->command, MAV_RESULT_ACCEPTED,
                                 cmd->target_system, cmd->target_component);
            mav.send_autopilot_version(state);
            break;
        case MAVLINK_MSG_ID_AVAILABLE_MODES:
            std::printf("MAVLINK_MSG_ID_AVAILABLE_MODES(435) handle requested mode=%u\n",
                        static_cast<uint32_t>(cmd->param2));
            mav.send_command_ack(state, cmd->command, MAV_RESULT_ACCEPTED,
                                 cmd->target_system, cmd->target_component);
            mav.send_available_modes(state, static_cast<uint32_t>(cmd->param2));
            break;
        case MAVLINK_MSG_ID_CAMERA_INFORMATION:
            std::printf("MAVLINK_MSG_ID_CAMERA_INFORMATION(259): MAV_RESULT_UNSUPPORTED\n");
            mav.send_command_ack(state, cmd->command, MAV_RESULT_UNSUPPORTED,
                                 cmd->target_system, cmd->target_component);
            break;
        case MAVLINK_MSG_ID_COMPONENT_METADATA:
            std::printf("MAVLINK_MSG_ID_COMPONENT_METADATA(397): MAV_RESULT_UNSUPPORTED\n");
            mav.send_command_ack(state, cmd->command, MAV_RESULT_UNSUPPORTED,
                                 cmd->target_system, cmd->target_component);
            break;
        case MAVLINK_MSG_ID_COMPONENT_INFORMATION:
            std::printf("MAVLINK_MSG_ID_COMPONENT_INFORMATION(395) MAV_RESULT_UNSUPPORTED\n");
            mav.send_command_ack(state, cmd->command, MAV_RESULT_UNSUPPORTED,
                                 cmd->target_system, cmd->target_component);
            break;
        case MAVLINK_MSG_ID_PROTOCOL_VERSION:
            std::printf("MAVLINK_MSG_ID_PROTOCOL_VERSION(300): MAV_RESULT_UNSUPPORTED - Deprecated\n");
            mav.send_command_ack(state, cmd->command, MAV_RESULT_UNSUPPORTED,
                                 cmd->target_system, cmd->target_component);
            break;
        case MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION:
            std::printf("MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION(280): MAV_RESULT_UNSUPPORTED\n");
            mav.send_command_ack(state, cmd->command, MAV_RESULT_UNSUPPORTED,
                                 cmd->target_system, cmd->target_component);
            break;
        default:
            std::printf("Unknown REQUEST_MESSAGE(%u): MAV_RESULT_UNSUPPORTED\n", msg_id);
            mav.send_command_ack(state, cmd->command, MAV_RESULT_UNSUPPORTED,
                                 cmd->target_system, cmd->target_component);
            break;
    }
}

inline void handle_set_mode(MavSender& mav, RoverState& state,
                             const mavlink_command_long_t *cmd)
{
    logger::line("rx: MAVLINK_MSG_ID_COMMAND_LONG(76): MAV_CMD_DO_SET_MODE(176): base_mode=0x%02X custom_mode=%u",
                 static_cast<uint32_t>(cmd->param1), static_cast<uint32_t>(cmd->param2));
    state.base_mode   = static_cast<uint8_t>(cmd->param1);
    state.custom_mode = static_cast<uint32_t>(cmd->param2);
    mav.send_command_ack(state, cmd->command, MAV_RESULT_ACCEPTED,
                         cmd->target_system, cmd->target_component);
}

/* ---- top-level COMMAND_LONG dispatcher ---- */

inline void handle_command_long(MavSender& mav, RoverState& state,
                                 const mavlink_command_long_t *cmd)
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
        case MAV_CMD_REQUEST_CAMERA_INFORMATION:
            logger::line("rx: MAVLINK_MSG_ID_COMMAND_LONG(76): MAV_CMD_REQUEST_CAMERA_INFORMATION(521): MAV_RESULT_UNSUPPORTED");
            mav.send_command_ack(state, cmd->command, MAV_RESULT_UNSUPPORTED,
                                 cmd->target_system, cmd->target_component);
            break;
        case MAV_CMD_MISSION_START:
            logger::line("rx: MAVLINK_MSG_ID_COMMAND_LONG(76): MAV_CMD_MISSION_START(300): MAV_RESULT_UNSUPPORTED");
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
