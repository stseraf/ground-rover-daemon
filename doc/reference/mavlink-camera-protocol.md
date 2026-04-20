# MAVLink Video Streaming — Rover Implementation Guide

This document describes everything needed to implement a MAVLink camera component on a rover (MAV_AUTOPILOT_GENERIC) so that QGroundControl can discover, display, and control video streams.

---

## Overview

QGC video streaming uses the **MAVLink Camera Protocol v2**. It is fully autopilot-agnostic — `MAV_AUTOPILOT_GENERIC` gets identical support to PX4 or ArduPilot. The rover must run a MAVLink **camera component** (separate from the autopilot component) that responds to the messages described below.

---

## 1. MAVLink Component Setup

Your rover must expose a camera component alongside the autopilot component:

| Parameter | Value |
|-----------|-------|
| System ID | Same as rover autopilot (e.g. `1`) |
| Component ID | `MAV_COMP_ID_CAMERA` = `100` |
| Additional cameras | `MAV_COMP_ID_CAMERA2..6` = `101..105` |

The camera component runs on the same MAVLink bus. It can be:
- A separate process/thread on the rover that listens on the same UART/UDP
- Part of the autopilot firmware if it handles camera messages itself

---

## 2. Required Capability Flags

When QGC connects to a vehicle, it sends `MAV_CMD_REQUEST_MESSAGE` for `CAMERA_INFORMATION` (msg id 259) to each camera component. Your component must respond with a `CAMERA_INFORMATION` message that includes:

```
CAMERA_INFORMATION.cam_definition_version  = 0 (or your version)
CAMERA_INFORMATION.flags |= CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM  (bit 2, value 0x04)
```

### Minimum `CAMERA_INFORMATION` fields

| Field | Value |
|-------|-------|
| `time_boot_ms` | System uptime in ms |
| `vendor_name` | ASCII string, e.g. `"Rover"` |
| `model_name` | ASCII string, e.g. `"MainCam"` |
| `firmware_version` | `0` is fine |
| `focal_length` | `0` if unknown |
| `sensor_size_h/v` | `0` if unknown |
| `resolution_h/v` | Max resolution in pixels |
| `flags` | `CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM` (= `0x04`) |
| `cam_definition_uri` | Empty string `""` if no camera definition file |
| `gimbal_device_id` | `0` |

If you also support photo capture add:
- `CAMERA_CAP_FLAGS_CAPTURE_IMAGE` (bit 0)
- `CAMERA_CAP_FLAGS_CAPTURE_VIDEO` (bit 1)

---

## 3. Video Stream Information

After receiving camera capabilities, QGC requests stream information.

### Request sequence QGC sends

1. `MAV_CMD_REQUEST_MESSAGE` with `param1 = MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION` (269)
   - If no response after 1 second, retries up to 6 times
   - On odd retry number falls back to deprecated `MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION`

### Response: `VIDEO_STREAM_INFORMATION` (msg id 269)

Send one message per stream. For a single stream:

| Field | Type | Description | Example |
|-------|------|-------------|---------|
| `stream_id` | uint8 | Unique stream id, start from `1` | `1` |
| `count` | uint8 | Total number of streams | `1` |
| `type` | uint8 | `VIDEO_STREAM_TYPE_RTSP = 0` | `0` |
| `flags` | uint16 | See flags table below | `1` (RUNNING) |
| `framerate` | float | Frames per second | `30.0` |
| `resolution_h` | uint16 | Width in pixels | `1920` |
| `resolution_v` | uint16 | Height in pixels | `1080` |
| `bitrate` | uint32 | Bits per second | `4000000` |
| `rotation` | uint16 | Image rotation in degrees (0/90/180/270) | `0` |
| `hfov` | uint16 | Horizontal FOV in degrees, `0` if unknown | `90` |
| `name` | char[32] | Human-readable stream name | `"Main"` |
| `uri` | char[160] | Connection URI | `"rtsp://192.168.1.1:8554/main"` |
| `encoding` | uint8 | Codec (see encoding table) | `1` (H264) |

### `flags` values

| Flag | Value | Meaning |
|------|-------|---------|
| `VIDEO_STREAM_STATUS_FLAGS_RUNNING` | `1` | Stream is active/running |
| `VIDEO_STREAM_STATUS_FLAGS_THERMAL` | `2` | This is a thermal stream |

### `encoding` (codec) values

| Value | Constant | Codec |
|-------|----------|-------|
| `0` | `VIDEO_STREAM_ENCODING_UNKNOWN` | Unknown |
| `1` | `VIDEO_STREAM_ENCODING_H264` | H.264 |
| `2` | `VIDEO_STREAM_ENCODING_H265` | H.265 / HEVC |
| `3` | `VIDEO_STREAM_ENCODING_MJPEG` | Motion JPEG |

### `type` values

| Value | Constant | Protocol |
|-------|----------|---------|
| `0` | `VIDEO_STREAM_TYPE_RTSP` | RTSP |
| `1` | `VIDEO_STREAM_TYPE_RTPUDP` | RTP over UDP |
| `2` | `VIDEO_STREAM_TYPE_TCP_MPEG` | TCP MPEG |
| `3` | `VIDEO_STREAM_TYPE_MPEG_TS_H264` | MPEG-TS H264 |

---

## 4. Video Stream Status

QGC also polls `VIDEO_STREAM_STATUS` (msg id 270) to get live updates on a running stream.

### Request

`MAV_CMD_REQUEST_MESSAGE` with `param1 = MAVLINK_MSG_ID_VIDEO_STREAM_STATUS` (270)

### Response: `VIDEO_STREAM_STATUS` (msg id 270)

| Field | Type | Description |
|-------|------|-------------|
| `stream_id` | uint8 | Which stream |
| `flags` | uint16 | Running / thermal flags |
| `framerate` | float | Current FPS |
| `resolution_h` | uint16 | Current width |
| `resolution_v` | uint16 | Current height |
| `bitrate` | uint32 | Current bitrate bps |
| `rotation` | uint16 | Current rotation |
| `hfov` | uint16 | Current HFOV |

---

## 5. Start / Stop Streaming Commands

QGC sends these when the user switches streams or pauses/resumes.

| Command | MAVLink ID | param1 | Action |
|---------|-----------|--------|--------|
| `MAV_CMD_VIDEO_START_STREAMING` | 2502 | `stream_id` | Start streaming |
| `MAV_CMD_VIDEO_STOP_STREAMING` | 2503 | `stream_id` | Stop streaming |

Respond with `COMMAND_ACK`:
- `result = MAV_RESULT_ACCEPTED` on success
- `result = MAV_RESULT_DENIED` if not supported

If your rover always streams (e.g. RTSP server is always running), you can respond `MAV_RESULT_ACCEPTED` without actually doing anything.

---

## 6. Multiple Streams

If you have multiple cameras or stream qualities, expose them all:

- Send multiple `VIDEO_STREAM_INFORMATION` messages, one per stream, with unique `stream_id` values (`1`, `2`, `3`, ...)
- Set `count` = total number of streams in each message
- QGC will show a **stream selection dropdown** in the camera settings panel when `count > 1`
- When user selects a stream, QGC sends:
  1. `MAV_CMD_VIDEO_STOP_STREAMING(old_stream_id)`
  2. `MAV_CMD_VIDEO_START_STREAMING(new_stream_id)`

Stream IDs must be sequential starting from `1`.

---

## 7. Thermal Stream Support

If your rover has a thermal camera, expose it as a second stream with the thermal flag:

```
stream_id = 2
flags     = VIDEO_STREAM_STATUS_FLAGS_THERMAL  (= 2)
uri       = "rtsp://192.168.1.1:8554/thermal"
name      = "Thermal"
```

QGC will automatically detect it and show **thermal overlay controls**:
- Off / Blend / Full / Picture-in-Picture modes
- Opacity slider in blend mode

The thermal stream is not shown in the stream selection dropdown — it is handled separately as an overlay.

---

## 8. Message Handling Summary

### Messages your component must handle (incoming)

| Message / Command | Required | Notes |
|-------------------|----------|-------|
| `MAV_CMD_REQUEST_MESSAGE` (param1 = 259) | Yes | Respond with `CAMERA_INFORMATION` |
| `MAV_CMD_REQUEST_MESSAGE` (param1 = 269) | Yes | Respond with `VIDEO_STREAM_INFORMATION` |
| `MAV_CMD_REQUEST_MESSAGE` (param1 = 270) | Yes | Respond with `VIDEO_STREAM_STATUS` |
| `MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION` (deprecated) | Recommended | QGC falls back to this |
| `MAV_CMD_REQUEST_VIDEO_STREAM_STATUS` (deprecated) | Recommended | QGC falls back to this |
| `MAV_CMD_VIDEO_START_STREAMING` | Recommended | ACK with ACCEPTED |
| `MAV_CMD_VIDEO_STOP_STREAMING` | Recommended | ACK with ACCEPTED |

### Messages your component must send (outgoing)

| Message | Trigger |
|---------|---------|
| `CAMERA_INFORMATION` | On request |
| `VIDEO_STREAM_INFORMATION` | On request |
| `VIDEO_STREAM_STATUS` | On request |
| `COMMAND_ACK` | After every command received |
| `HEARTBEAT` | Every 1 second (component must heartbeat) |

---

## 9. Heartbeat

Your camera component must send a `HEARTBEAT` every second:

| Field | Value |
|-------|-------|
| `type` | `MAV_TYPE_CAMERA` = `26` |
| `autopilot` | `MAV_AUTOPILOT_INVALID` = `8` |
| `base_mode` | `0` |
| `custom_mode` | `0` |
| `system_status` | `MAV_STATE_ACTIVE` = `4` |

Without a heartbeat, QGC will not recognize the camera component.

---

## 10. Minimal Implementation Checklist

```
[ ] Camera component heartbeats at MAV_COMP_ID_CAMERA (100), sysid = rover sysid
[ ] Responds to CAMERA_INFORMATION request with CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM set
[ ] Responds to VIDEO_STREAM_INFORMATION request with valid URI and stream metadata
[ ] Responds to VIDEO_STREAM_STATUS request
[ ] ACKs MAV_CMD_VIDEO_START_STREAMING / STOP_STREAMING with MAV_RESULT_ACCEPTED
[ ] RTSP server (or other) accessible from QGC host on the URI provided
[ ] stream_id starts at 1 (not 0)
[ ] count field matches actual number of streams
```

---

## 11. Example VIDEO_STREAM_INFORMATION (single H.264 RTSP stream)

```c
mavlink_video_stream_information_t stream = {
    .stream_id    = 1,
    .count        = 1,
    .type         = VIDEO_STREAM_TYPE_RTSP,          // 0
    .flags        = VIDEO_STREAM_STATUS_FLAGS_RUNNING, // 1
    .framerate    = 30.0f,
    .resolution_h = 1920,
    .resolution_v = 1080,
    .bitrate      = 4000000,
    .rotation     = 0,
    .hfov         = 90,
    .encoding     = VIDEO_STREAM_ENCODING_H264,      // 1
};
strncpy(stream.name, "Main", sizeof(stream.name));
strncpy(stream.uri,  "rtsp://192.168.1.1:8554/main", sizeof(stream.uri));
```

---

## 12. RTSP Server Options for Rover

Common choices for running an RTSP server on a Linux-based rover:

| Option | Notes |
|--------|-------|
| **GStreamer + rtsp-server** | Most flexible, hardware encode support |
| **MediaMTX (rtsp-simple-server)** | Lightweight Go binary, easy config |
| **FFmpeg** | Quick test streaming, less suited for production |
| **Raspberry Pi Camera + libcamera** | Native RTSP via `libcamera-vid --codec h264` + MediaMTX |

QGC uses GStreamer on the receiving end. Any standard RTSP H.264 or H.265 stream will work.

---

## 13. Troubleshooting

| Symptom | Likely Cause |
|---------|-------------|
| No camera appears in QGC | Component not heartbeating or wrong component ID |
| Camera appears but no video controls | `CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM` not set in CAMERA_INFORMATION |
| Video stream info requested but times out | Component not responding to `MAV_CMD_REQUEST_MESSAGE` |
| QGC shows stream but video is black | RTSP URI not reachable from QGC host (firewall, wrong IP) |
| Stream dropdown not shown | Only one stream — dropdown only appears with 2+ non-thermal streams |
| Thermal overlay controls missing | Thermal stream's `flags` field doesn't have `VIDEO_STREAM_STATUS_FLAGS_THERMAL` set |

---

## References

- MAVLink Camera Protocol: https://mavlink.io/en/services/camera.html
- Video Stream messages: https://mavlink.io/en/messages/common.html#VIDEO_STREAM_INFORMATION
- QGC implementation: `src/Camera/VehicleCameraControl.cc` (stream discovery ~line 1618)
- QGC UI: `src/QmlControls/PhotoVideoControl.qml` (stream selection ~line 485)
