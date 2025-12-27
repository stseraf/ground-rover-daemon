#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <time.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include "external/mavlink/common/mavlink.h"
#pragma GCC diagnostic pop

#define MAV_SYS_ID    1
#define MAV_COMP_ID   1

#define UDP_PORT      14550
#define QGC_PORT      14550

static int armed = 0;

static struct sockaddr_in qgc_addr;
static socklen_t qgc_addr_len = sizeof(qgc_addr);

static uint8_t current_base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
static uint32_t current_custom_mode = 0;

/* -------------------- utils -------------------- */

static uint64_t micros()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000;
}

void log_message(bool new_line, const char *format, ...)
{
    static bool last_log_same_line = false;
    if (!new_line) {
        printf("\r");
        last_log_same_line = true;
    }
    else {
        if (last_log_same_line) printf("\n");
        last_log_same_line = false;
    }

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    static long sec_diff = 0;
    if (!sec_diff) {
        sec_diff = ts.tv_sec;
    }
    long seconds = ts.tv_sec - sec_diff;
    int millis = ts.tv_nsec / 1000000;

    printf("[%05ld.%03d] ", seconds, millis);
    if (!new_line) printf("*");

    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    
    if (new_line) printf("\n");
    fflush(stdout);
}

/* -------------------- MAVLink TX -------------------- */

static void mav_send(int sock, mavlink_message_t *msg)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    int len = mavlink_msg_to_send_buffer(buf, msg);
    sendto(sock, buf, len, 0,
           (struct sockaddr*)&qgc_addr, qgc_addr_len);
}

static void send_heartbeat(int sock)
{
    mavlink_message_t msg;

    uint8_t base_mode = current_base_mode | (armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0);

    mavlink_msg_heartbeat_pack(
        MAV_SYS_ID,
        MAV_COMP_ID,
        &msg,
        MAV_TYPE_GROUND_ROVER,
        MAV_AUTOPILOT_GENERIC,
        base_mode,
        current_custom_mode,
        MAV_STATE_ACTIVE
    );
    mav_send(sock, &msg);
}

static void send_sys_status(int sock)
{
    mavlink_message_t msg;

    mavlink_sys_status_t sys = {
        .onboard_control_sensors_present = 0,
        .onboard_control_sensors_enabled = 0,
        .onboard_control_sensors_health = 0,
        .load = 500,             // (0.1%)
        .voltage_battery = 12000,
        .current_battery = 0,
        .drop_rate_comm = 0,
        .errors_comm = 0,
        .errors_count1 = 0,
        .errors_count2 = 0,
        .errors_count3 = 0,
        .errors_count4 = 0,
        .battery_remaining = 80,
        .onboard_control_sensors_present_extended = 0,
        .onboard_control_sensors_enabled_extended = 0,
        .onboard_control_sensors_health_extended = 0
    };

    mavlink_msg_sys_status_encode(MAV_SYS_ID, MAV_COMP_ID, &msg, &sys);
    mav_send(sock, &msg);
}

static void send_autopilot_version(int sock)
{
    mavlink_message_t msg;

    mavlink_autopilot_version_t av = {0};
    av.capabilities =
      MAV_PROTOCOL_CAPABILITY_MISSION_INT
    | MAV_PROTOCOL_CAPABILITY_COMMAND_INT
    | MAV_PROTOCOL_CAPABILITY_MAVLINK2;
    av.flight_sw_version = 0;
    av.middleware_sw_version = 0;
    av.os_sw_version = 0;
    av.board_version = 0;
    memset(av.flight_custom_version, 0, sizeof(av.flight_custom_version));
    memset(av.middleware_custom_version, 0, sizeof(av.middleware_custom_version));
    memset(av.os_custom_version, 0, sizeof(av.os_custom_version));
    av.vendor_id = 0;
    av.product_id = 0;
    av.uid = 0;

    mavlink_msg_autopilot_version_encode(MAV_SYS_ID, MAV_COMP_ID, &msg, &av);
    log_message(true, "tx: MAVLINK_MSG_ID_AUTOPILOT_VERSION(148): capabilities=0x%016llX", (unsigned long long)av.capabilities);
    mav_send(sock, &msg);
}

static void send_available_modes(int sock, uint32_t mode)
{
    struct mode_info {
        const char *name;
        uint8_t standard_mode;
        uint32_t custom_mode;
        uint32_t properties;
    } modes[] = {
        { "HOLD", MAV_STANDARD_MODE_POSITION_HOLD, 0, 0 },
        { "MANUAL", MAV_STANDARD_MODE_NON_STANDARD, 0, MAV_MODE_PROPERTY_ADVANCED }
    };

    const uint8_t number_modes = (uint8_t)(sizeof(modes) / sizeof(modes[0]));

    mavlink_message_t msg;
    mavlink_available_modes_t am;
    memset(&am, 0, sizeof(am));

    am.number_modes = number_modes;
    am.mode_index = mode; /* indexed from 1 */
    am.standard_mode = modes[mode-1].standard_mode;
    am.custom_mode = modes[mode-1].custom_mode;
    am.properties = modes[mode-1].properties;
    strncpy(am.mode_name, modes[mode-1].name, sizeof(am.mode_name) - 1);

    mavlink_msg_available_modes_encode(MAV_SYS_ID, MAV_COMP_ID, &msg, &am);
    log_message(true, "tx: MAVLINK_MSG_ID_AVAILABLE_MODES(435) mode %u/%u: %s", mode, number_modes, modes[mode-1].name);
    mav_send(sock, &msg);
}

// Send a COMMAND_ACK message
static void send_command_ack(int sock, uint16_t command, uint8_t result,
                             uint8_t target_system, uint8_t target_component)
{
    mavlink_message_t ack;
    mavlink_msg_command_ack_pack(
        MAV_SYS_ID,
        MAV_COMP_ID,
        &ack,
        command,
        result,
        0, /* progress */
        0, /* result_param2 */
        target_system,
        target_component
    );
    log_message(true, "tx: MAVLINK_MSG_ID_COMMAND_ACK(77): command=%u result=%u", command, result);
    mav_send(sock, &ack);
}

/* -------------------- COMMAND HANDLING -------------------- */

static void handle_command_long(int sock,
                                const mavlink_command_long_t *cmd)
{
    if (cmd->command == MAV_CMD_COMPONENT_ARM_DISARM) {
        armed = (cmd->param1 > 0.5f);
        log_message(true, "rx: MAVLINK_MSG_ID_COMMAND_LONG(76): MAV_CMD_COMPONENT_ARM_DISARM(400): Arm=%d", (uint32_t)cmd->param1);
        send_command_ack(sock, cmd->command, MAV_RESULT_ACCEPTED, cmd->target_system, cmd->target_component);
    }
    else if (cmd->command == MAV_CMD_REQUEST_MESSAGE) {
        log_message(true, "rx: MAVLINK_MSG_ID_COMMAND_LONG(76): MAV_CMD_REQUEST_MESSAGE(512): ");

        if ((uint32_t)cmd->param1 == MAVLINK_MSG_ID_SYS_STATUS) {
            printf("MAVLINK_MSG_ID_SYS_STATUS(1) handle\n");
            send_command_ack(sock, cmd->command, MAV_RESULT_ACCEPTED, cmd->target_system, cmd->target_component);
            send_sys_status(sock);
        }
        else if ((uint32_t)cmd->param1 == MAVLINK_MSG_ID_AUTOPILOT_VERSION) {
            printf("MAVLINK_MSG_ID_AUTOPILOT_VERSION(148) handle\n");
            send_command_ack(sock, cmd->command, MAV_RESULT_ACCEPTED, cmd->target_system, cmd->target_component);
            send_autopilot_version(sock);
        }
        else if ((uint32_t)cmd->param1 == MAVLINK_MSG_ID_CAMERA_INFORMATION) {
            printf("MAVLINK_MSG_ID_CAMERA_INFORMATION(259): MAV_RESULT_UNSUPPORTED\n");
            send_command_ack(sock, cmd->command, MAV_RESULT_UNSUPPORTED, cmd->target_system, cmd->target_component);
        }
        else if ((uint32_t)cmd->param1 == MAVLINK_MSG_ID_AVAILABLE_MODES) {
            printf("MAVLINK_MSG_ID_AVAILABLE_MODES(435) handle requested mode=%u\n", (uint32_t)cmd->param2);
            send_command_ack(sock, cmd->command, MAV_RESULT_ACCEPTED, cmd->target_system, cmd->target_component);
            send_available_modes(sock, (uint32_t)cmd->param2);
        }
        else if ((uint32_t)cmd->param1 == MAVLINK_MSG_ID_COMPONENT_METADATA) {
            printf("MAVLINK_MSG_ID_COMPONENT_METADATA(397): MAV_RESULT_UNSUPPORTED\n");
            send_command_ack(sock, cmd->command, MAV_RESULT_UNSUPPORTED, cmd->target_system, cmd->target_component);
        }
        else if ((uint32_t)cmd->param1 == MAVLINK_MSG_ID_COMPONENT_INFORMATION) {
            printf("MAVLINK_MSG_ID_COMPONENT_INFORMATION(395) MAV_RESULT_UNSUPPORTED\n");
            send_command_ack(sock, cmd->command, MAV_RESULT_UNSUPPORTED, cmd->target_system, cmd->target_component);
        }
        else if ((uint32_t)cmd->param1 == MAVLINK_MSG_ID_PROTOCOL_VERSION) {
            printf("MAVLINK_MSG_ID_PROTOCOL_VERSION(300): MAV_RESULT_UNSUPPORTED - Deprecated\n");
            send_command_ack(sock, cmd->command, MAV_RESULT_UNSUPPORTED, cmd->target_system, cmd->target_component);
        }
        else if ((uint32_t)cmd->param1 == MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION) {
            printf("MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION(280): MAV_RESULT_UNSUPPORTED\n");
            send_command_ack(sock, cmd->command, MAV_RESULT_UNSUPPORTED, cmd->target_system, cmd->target_component);
        }
        else {
            printf("Unknown REQUEST_MESSAGE(%u): MAV_RESULT_UNSUPPORTED\n", (uint32_t)cmd->param1);
            send_command_ack(sock, cmd->command, MAV_RESULT_UNSUPPORTED, cmd->target_system, cmd->target_component);
        }
    }
    else if (cmd->command == MAV_CMD_DO_SET_MODE) {
        log_message(true, "rx: MAVLINK_MSG_ID_COMMAND_LONG(76): MAV_CMD_DO_SET_MODE(176): base_mode=0x%02X custom_mode=%u", (uint32_t)cmd->param1, (uint32_t)cmd->param2);
        current_base_mode = (uint8_t)cmd->param1;
        current_custom_mode = (uint32_t)cmd->param2;
        send_command_ack(sock, cmd->command, MAV_RESULT_ACCEPTED, cmd->target_system, cmd->target_component);
    }
    else if (cmd->command == MAV_CMD_NAV_TAKEOFF) {
        log_message(true, "rx: MAVLINK_MSG_ID_COMMAND_LONG(76): MAV_CMD_NAV_TAKEOFF(22): MAV_RESULT_UNSUPPORTED");
        send_command_ack(sock, cmd->command, MAV_RESULT_UNSUPPORTED, cmd->target_system, cmd->target_component);
    }
    else if (cmd->command == MAV_CMD_REQUEST_CAMERA_INFORMATION) {
        log_message(true, "rx: MAVLINK_MSG_ID_COMMAND_LONG(76): MAV_CMD_REQUEST_CAMERA_INFORMATION(521): MAV_RESULT_UNSUPPORTED");
        send_command_ack(sock, cmd->command, MAV_RESULT_UNSUPPORTED, cmd->target_system, cmd->target_component);
    }
    else if (cmd->command == MAV_CMD_MISSION_START) {
        log_message(true, "rx: MAVLINK_MSG_ID_COMMAND_LONG(76): MAV_CMD_MISSION_START(300): MAV_RESULT_UNSUPPORTED");
        send_command_ack(sock, cmd->command, MAV_RESULT_UNSUPPORTED, cmd->target_system, cmd->target_component);
    }
    else {
        log_message(true, "rx: MAVLINK_MSG_ID_COMMAND_LONG(76): Unknown COMMAND_LONG(%u): MAV_RESULT_UNSUPPORTED", (uint32_t)cmd->command);
        send_command_ack(sock, cmd->command, MAV_RESULT_UNSUPPORTED, cmd->target_system, cmd->target_component);
    }
}

/* -------------------- MAIN -------------------- */

int main(void)
{
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("socket");
        return 1;
    }

    struct sockaddr_in local = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = INADDR_ANY,
        .sin_port = htons(UDP_PORT)
    };

    if (bind(sock, (struct sockaddr*)&local, sizeof(local)) < 0) {
        perror("bind");
        return 1;
    }

    log_message(true, "MAVLink rover daemon started");

    uint64_t last_hb = 0;
    mavlink_message_t msg;
    mavlink_status_t status;

    while (1) {
        uint8_t receive_buffer[2048];
        ssize_t n = recvfrom(sock, receive_buffer, sizeof(receive_buffer), MSG_DONTWAIT, (struct sockaddr*)&qgc_addr, &qgc_addr_len);
        if (n > 0) {
            for (ssize_t i = 0; i < n; i++) {
                if (mavlink_parse_char(MAVLINK_COMM_0, receive_buffer[i], &msg, &status)) {
                    switch (msg.msgid) {
                        case MAVLINK_MSG_ID_HEARTBEAT: {
                            mavlink_heartbeat_t hb;
                            mavlink_msg_heartbeat_decode(&msg, &hb);
                            //log_message(true, "rx: MAVLINK_MSG_ID_HEARTBEAT(0): type=%u autopilot=%u base_mode=0x%02X", hb.type, hb.autopilot, hb.base_mode);
                            break;
                        }
                        case MAVLINK_MSG_ID_SET_MODE: {
                            mavlink_set_mode_t sm;
                            mavlink_msg_set_mode_decode(&msg, &sm);
                            log_message(true, "rx: MAVLINK_MSG_ID_SET_MODE(11): base=0x%02X custom=%u", sm.base_mode, sm.custom_mode);
                            current_base_mode = sm.base_mode;
                            current_custom_mode = sm.custom_mode;
                            break;
                        }
                        case MAVLINK_MSG_ID_MANUAL_CONTROL: {
                            mavlink_manual_control_t mc;
                            mavlink_msg_manual_control_decode(&msg, &mc);
                            log_message(false, "rx: MAVLINK_MSG_ID_MANUAL_CONTROL(69) x=%d y=%d z=%d r=%d buttons=0x%02X           ", mc.x, mc.y, mc.z, mc.r, mc.buttons);
                            break;
                        }
                        case MAVLINK_MSG_ID_COMMAND_LONG: {
                            mavlink_command_long_t cmd;
                            mavlink_msg_command_long_decode(&msg, &cmd);
                            handle_command_long(sock, &cmd);
                            break;
                        }
                        case MAVLINK_MSG_ID_REQUEST_DATA_STREAM: {
                            mavlink_request_data_stream_t rds;
                            mavlink_msg_request_data_stream_decode(&msg, &rds);
                            log_message(true, "rx: MAVLINK_MSG_ID_REQUEST_DATA_STREAM(66): req_stream_id=%u req_message_rate=%u start_stop=%u"
                                , rds.req_stream_id, rds.req_message_rate, rds.start_stop);
                            break;
                        }
                        case MAVLINK_MSG_ID_DATA_STREAM: {
                            mavlink_data_stream_t ds;
                            mavlink_msg_data_stream_decode(&msg, &ds);
                            log_message(true, "rx: MAVLINK_MSG_ID_DATA_STREAM(67): stream_id=%u message_rate=%u", ds.stream_id, ds.message_rate);
                            break;
                        }
                        case MAVLINK_MSG_ID_SYSTEM_TIME: {
                            mavlink_system_time_t st;
                            mavlink_msg_system_time_decode(&msg, &st);
                            log_message(true, "rx: MAVLINK_MSG_ID_SYSTEM_TIME(2): time_unix_usec=%llu time_boot_ms=%u", (unsigned long long)st.time_unix_usec, st.time_boot_ms);
                            break;
                        }
                        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
                            mavlink_mission_request_list_t mrl;
                            mavlink_msg_mission_request_list_decode(&msg, &mrl);
                            log_message(true, "rx: MAVLINK_MSG_ID_MISSION_REQUEST_LIST(43): target=%u:%u", mrl.target_system, mrl.target_component);
                            // send MISSION_COUNT
                            {
                                printf("MAVLINK_MSG_ID_MISSION_COUNT(44) handle\n");
                                mavlink_message_t mc;
                                mavlink_msg_mission_count_pack(
                                    MAV_SYS_ID, // system_id
                                    MAV_COMP_ID, // component_id
                                    &mc, // message
                                    mrl.target_system, // target_system
                                    mrl.target_component, // target_component
                                    0, // count
                                    MAV_MISSION_TYPE_MISSION, // mission_type
                                    0 // opaque_id
                                );
                                log_message(true, "tx: MAVLINK_MSG_ID_MISSION_COUNT(44): %u", 0);
                                mav_send(sock, &mc);
                            }
                            break;
                        }
                        case MAVLINK_MSG_ID_MISSION_ACK: {
                            mavlink_mission_ack_t ma;
                            mavlink_msg_mission_ack_decode(&msg, &ma);
                            log_message(true, "rx: MAVLINK_MSG_ID_MISSION_ACK(47): target=%u:%u type=%u", ma.target_system, ma.target_component, ma.type);
                            break;
                        }
                        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
                            mavlink_param_request_list_t prl;
                            mavlink_msg_param_request_list_decode(&msg, &prl);
                            log_message(true, "rx: MAVLINK_MSG_ID_PARAM_REQUEST_LIST(21) received target=%u:%u", prl.target_system, prl.target_component);

                            /* Send a few dummy parameters so QGC knows sequence is complete */
                            {
                                printf("MAVLINK_MSG_ID_PARAM_VALUE(22) handle\n");
                                const uint16_t param_count = 3;
                                const char *names[param_count];
                                float values[param_count];
                                names[0] = "DUMMY_P1";
                                names[1] = "DUMMY_P2";
                                names[2] = "DUMMY_P3";
                                values[0] = 0.000f;
                                values[1] = 0.000f;
                                values[2] = 0.000f;

                                for (uint16_t i = 0; i < param_count; i++) {
                                    mavlink_message_t pm;
                                    mavlink_msg_param_value_pack(
                                        MAV_SYS_ID,
                                        MAV_COMP_ID,
                                        &pm,
                                        names[i],          /* param_id */
                                        values[i],         /* param_value */
                                        MAV_PARAM_TYPE_REAL32,
                                        param_count,
                                        i                  /* param_index */
                                    );
                                    log_message(true, "tx: MAVLINK_MSG_ID_PARAM_VALUE(22): %s=%.2f (%u/%u)", names[i], values[i], i+1, param_count);
                                    mav_send(sock, &pm);
                                }
                            }
                            break;
                        }
                        default:
                            log_message(true, "rx: Unknown message ID: %u", msg.msgid);
                            break;
                    }
                }
            }
        }

        uint64_t now = micros();
    
        if (now - last_hb > 1000000) { // 1 Hz
            send_heartbeat(sock);
            send_sys_status(sock);
            last_hb = now;
        }
        usleep(1000); // 1 ms
    }
}