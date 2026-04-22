#include "command_handlers.hpp"
#include "mav_sender.hpp"
#include "param_store.hpp"
#include "rover_state.hpp"
#include "config.hpp"
#include "logger.hpp"

#include <cstdio>
#include <cmath>

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

    // GIMBAL_MANAGER_INFORMATION: QGC retries 6× if UNSUPPORTED. Reply
    // ACCEPTED and emit a "no gimbal" stub so QGC stops asking.
    if (msg_id == MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION) {
        mav.send_command_ack(state, cmd->command, MAV_RESULT_ACCEPTED,
                             cmd->target_system, cmd->target_component);
        mav.send_gimbal_manager_information(state, 1); // comp 1 = autopilot
        return;
    }
    // COMPONENT_INFORMATION / METADATA are one-shots in QGC — UNSUPPORTED
    // is fine and doesn't drive retry loops.
    switch (msg_id) {
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

// MAV_CMD_PREFLIGHT_CALIBRATION (241). We fake the ArduPilot calibration
// flow without any real IMU work so QGC's "Calibrate" button returns green:
//   - param1 > 0 → gyro cal:  jitter INS_GYROFFS_*
//   - param5 == 2 → level horizon: zero AHRS_TRIM_*
//   - param5 == 1 or 4 → accel cal (full or simple): jitter INS_ACCOFFS_*
// Each path bumps the relevant params, emits PARAM_VALUE so QGC sees them
// change, and persists via ParamStore::save().
void handle_preflight_calibration(MavSender& mav, RoverState& state,
                                   ParamStore& params,
                                   const mavlink_command_long_t* cmd)
{
    auto bump = [&](const char* name, float base, float delta) {
        int idx = params.find_by_name(name);
        if (idx < 0) return;
        params.set(static_cast<uint16_t>(idx), base + delta);
        mav.send_param(state, params.name(idx), params.get(idx),
                       static_cast<uint16_t>(idx), ParamStore::COUNT);
    };

    // Small deterministic jitter keyed on confirmation byte so repeated cals
    // produce visibly different values — QGC checks that offsets *changed*
    // rather than what they are.
    float j = static_cast<float>((cmd->confirmation & 0x0F) + 1) * 0.001f;

    bool gyro_cal   = cmd->param1 > 0.5f;
    int  accel_mode = static_cast<int>(cmd->param5 + 0.5f);

    if (gyro_cal) {
        logger::line("rx: MAV_CMD_PREFLIGHT_CALIBRATION(241): gyro cal");
        mav.send_statustext(state, MAV_SEVERITY_INFO, "Calibrating gyroscopes");
        bump("INS_GYROFFS_X",  0.001f,  j);
        bump("INS_GYROFFS_Y", -0.002f, -j);
        bump("INS_GYROFFS_Z",  0.001f,  j);
        mav.send_statustext(state, MAV_SEVERITY_INFO, "Gyro calibrated");
    }

    if (accel_mode == 2) {
        logger::line("rx: MAV_CMD_PREFLIGHT_CALIBRATION(241): level horizon");
        mav.send_statustext(state, MAV_SEVERITY_INFO, "Level horizon");
        bump("AHRS_TRIM_X", 0.0f, 0.0f);
        bump("AHRS_TRIM_Y", 0.0f, 0.0f);
        mav.send_statustext(state, MAV_SEVERITY_INFO, "Level complete");
    } else if (accel_mode == 1 || accel_mode == 4) {
        const char* label = (accel_mode == 4) ? "simple accel cal" : "accel cal";
        logger::line("rx: MAV_CMD_PREFLIGHT_CALIBRATION(241): %s", label);
        mav.send_statustext(state, MAV_SEVERITY_INFO, "Calibrating accelerometers");
        bump("INS_ACCOFFS_X",  0.02f,  j);
        bump("INS_ACCOFFS_Y", -0.01f, -j);
        bump("INS_ACCOFFS_Z",  0.03f,  j);
        bump("INS_ACCSCAL_X",  1.0f,   j * 0.01f);
        bump("INS_ACCSCAL_Y",  1.0f,  -j * 0.01f);
        bump("INS_ACCSCAL_Z",  1.0f,   j * 0.01f);
        mav.send_statustext(state, MAV_SEVERITY_INFO, "Calibration successful");
    }

    if (gyro_cal || accel_mode == 1 || accel_mode == 2 || accel_mode == 4) {
        params.save(Config::PARAM_FILE);
        mav.send_command_ack(state, cmd->command, MAV_RESULT_ACCEPTED,
                             cmd->target_system, cmd->target_component);
    } else {
        logger::line("rx: MAV_CMD_PREFLIGHT_CALIBRATION(241): unsupported combination "
                     "p1=%.0f p2=%.0f p3=%.0f p4=%.0f p5=%.0f p6=%.0f p7=%.0f",
                     cmd->param1, cmd->param2, cmd->param3, cmd->param4,
                     cmd->param5, cmd->param6, cmd->param7);
        mav.send_command_ack(state, cmd->command, MAV_RESULT_UNSUPPORTED,
                             cmd->target_system, cmd->target_component);
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

void handle_command_long(MavSender& mav, RoverState& state, ParamStore& params,
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
        case MAV_CMD_PREFLIGHT_CALIBRATION:
            handle_preflight_calibration(mav, state, params, cmd);
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
        case MAV_CMD_SET_MESSAGE_INTERVAL: // 511 — QGC throttling hint, polled often
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
