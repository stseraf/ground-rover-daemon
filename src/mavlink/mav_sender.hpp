#pragma once

#include <cstdint>

#include "udp_socket.hpp"
#include "rover_state.hpp"
#include "camera_info.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include "common/mavlink.h"
#pragma GCC diagnostic pop

class MavSender {
public:
    MavSender(UdpSocket& sock, uint8_t sys_id, uint8_t comp_id);

    // Autopilot component messages
    void send_heartbeat(const RoverState& state);
    void send_sys_status(const RoverState& state);
    void send_autopilot_version(const RoverState& state);
    void send_protocol_version(const RoverState& state);
    void send_current_mode(const RoverState& state);
    void send_available_modes(const RoverState& state, uint32_t mode);
    void send_command_ack(const RoverState& state, uint16_t command, uint8_t result,
                          uint8_t target_system, uint8_t target_component);
    void send_mission_count(const RoverState& state,
                            uint8_t target_system, uint8_t target_component);
    void send_servo_output_raw(const RoverState& state, int16_t left, int16_t right);
    void send_param(const RoverState& state, const char* name, float value,
                    uint16_t index, uint16_t total);
    void send_gps_raw_int(const RoverState& state);
    void send_global_position_int(const RoverState& state);
    void send_radio_status(const RoverState& state);
    void send_statustext(const RoverState& state, uint8_t severity, const char* text);

    // Camera component messages — all take cam_comp_id (MAV_COMP_ID_CAMERA + i)
    void send_camera_heartbeat(uint8_t cam_comp_id, const RoverState& state);
    void send_camera_information(uint8_t cam_comp_id, const RoverState& state,
                                 const CameraInfo& cam);
    void send_video_stream_information(uint8_t cam_comp_id, const RoverState& state,
                                       const CameraInfo& cam, int cam_idx, int mode_idx);
    void send_video_stream_status(uint8_t cam_comp_id, const RoverState& state,
                                   const CameraInfo& cam, int cam_idx, uint8_t stream_id);
    void send_command_ack_cam(uint8_t cam_comp_id, const RoverState& state,
                               uint16_t command, uint8_t result,
                               uint8_t target_system, uint8_t target_component);

    // Stub responses sent from the camera component
    void send_camera_settings(uint8_t cam_comp_id, const RoverState& state);
    void send_storage_information(uint8_t cam_comp_id, const RoverState& state);
    void send_camera_capture_status(uint8_t cam_comp_id, const RoverState& state);

private:
    void send(mavlink_message_t& msg, const RoverState& state);

    UdpSocket& sock_;
    uint8_t    sys_id_;
    uint8_t    comp_id_;
};
