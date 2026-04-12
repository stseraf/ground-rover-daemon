# QGC Camera & Video Stream — Code Analysis

> Codebase: QGroundControl @ `/home/stseraf/dev/rover/qgroundcontrol`
> Date: 2026-04-11

---

## Q1 — Camera Heartbeat Acceptance

**Source:** [QGCCameraManager.cc:109-114](src/Camera/QGCCameraManager.cc#L109-L114)

The message router that dispatches to `_handleHeartbeat` filters incoming MAVLink messages by:

```cpp
if (message.sysid == _vehicle->id() &&
    (message.compid == MAV_COMP_ID_AUTOPILOT1 ||
     (message.compid >= MAV_COMP_ID_CAMERA && message.compid <= MAV_COMP_ID_CAMERA6)))
```

| Check | Answer |
|---|---|
| `type == MAV_TYPE_CAMERA` (29)? | **No.** Never read. |
| `autopilot == MAV_AUTOPILOT_INVALID`? | **No.** Never read. |
| `system_status` or `base_mode`? | **No.** Never read. |
| `sysid` must match vehicle sysid? | **Yes.** `message.sysid == _vehicle->id()` is mandatory. |

Recognition is purely by **component ID**: `MAV_COMP_ID_CAMERA` (100) through `MAV_COMP_ID_CAMERA6` (105), plus `MAV_COMP_ID_AUTOPILOT1` (1) for autopilot-connected cameras.

On the first heartbeat from an unknown component ID, a `CameraStruct` is created and `_requestCameraInfo()` is called immediately ([QGCCameraManager.cc:160-163](src/Camera/QGCCameraManager.cc#L160-L163)).

---

## Q2 — CAMERA_INFORMATION Validation (msg 259)

**Source:** [QGCCameraManager.cc:260-270](src/Camera/QGCCameraManager.cc#L260-L270), [VehicleCameraControl.cc:139-143](src/Camera/VehicleCameraControl.cc#L139-L143)

`_handleCameraInfo` performs **zero field validation**. It sets `infoReceived = true` and decodes the struct as-is. In the `VehicleCameraControl` constructor:

```cpp
if (info->cam_definition_uri[0] != 0) {
    _handleDefinitionFile(info->cam_definition_uri);   // download XML definition
} else {
    _initWhenReady();                                   // skip straight to init
}
```

| Field | Required / Checked? |
|---|---|
| `time_boot_ms != 0` | **No.** Not read after decode. |
| `firmware_version != 0` | **No.** Stored but not gated on. |
| `focal_length`, `sensor_size_h/v` | **No.** Not checked. |
| Empty `cam_definition_uri` | **Init succeeds immediately.** Calls `_initWhenReady()` directly — no stall. |

---

## Q3 — Video Stream Dropdown Trigger

**Source:** [VehicleCameraControl.cc:1620-1640](src/Camera/VehicleCameraControl.cc#L1620-L1640)

Each received `VIDEO_STREAM_INFORMATION` message calls `handleVideoInfo()`:

```cpp
_expectedCount = vi->count;
if (!_findStream(vi->stream_id, false)) {
    _streams.append(new QGCVideoStreamInfo(*vi, this));
    _streamLabels.append(pStream->name());
    emit streamLabelsChanged();
}
if (_streams.count() < _expectedCount)
    _streamInfoTimer.start(1000);   // request remaining streams
```

| Question | Answer |
|---|---|
| Appears after `VIDEO_STREAM_INFORMATION` or after full init? | **After `VIDEO_STREAM_INFORMATION` is received.** Does not wait for full init. Driven by `streamLabelsChanged()` signal. |
| Camera control panel or separate video settings? | **Video settings panel** (PhotoVideoControl.qml), gated on `streamLabels.length > 1`. |
| `count > 1` causes multiple entries, or one entry per message? | **One entry per message.** The `count` field is used only to know when to request more; it does not pre-populate entries. |

---

## Q4 — Persistent REQUEST_MESSAGE(259) Retries

**Source:** [QGCCameraManager.cc:441-514](src/Camera/QGCCameraManager.cc#L441-L514)

This is a **retry loop that runs because `infoReceived` never gets set to `true`**. The flag is only set inside `_handleCameraInfo` when an actual `CAMERA_INFORMATION` message arrives. Every ACK failure (`!= MAV_RESULT_ACCEPTED`) calls `_handleCameraInfoRetry()`, which increments `retryCount` and schedules the next attempt:

```cpp
// For even retryCount >= 2: exponential backoff
int delaySeconds = 1 << (cameraInfo->retryCount / 2);   // 2^(n/2)
// For odd retryCount: immediate retry
```

The method alternates between `MAV_CMD_REQUEST_MESSAGE` (even counts) and `MAV_CMD_REQUEST_CAMERA_INFORMATION` (odd counts).

**Full backoff schedule:**

| retryCount | Method used | Delay before firing |
|---|---|---|
| 0 | `REQUEST_MESSAGE` | immediate |
| 1 | `REQUEST_CAMERA_INFORMATION` | immediate |
| 2 | `REQUEST_MESSAGE` | **2 s** |
| 3 | `REQUEST_CAMERA_INFORMATION` | immediate |
| 4 | `REQUEST_MESSAGE` | **4 s** |
| 5 | `REQUEST_CAMERA_INFORMATION` | immediate |
| 6 | `REQUEST_MESSAGE` | **8 s** |
| 7 | `REQUEST_CAMERA_INFORMATION` | immediate |
| 8 | `REQUEST_MESSAGE` | **16 s** |
| ≥ 10 | — | **gives up** |

**Root cause:** If your `COMMAND_ACK` returns `MAV_RESULT_ACCEPTED` but `CAMERA_INFORMATION` never arrives (or is not routed correctly), `infoReceived` stays false and retries continue indefinitely until the 10-attempt limit. Once `CAMERA_INFORMATION` is correctly received, `infoReceived = true`, `retryCount` resets to 0, and the backoff timer stops.

---

## Q5 — VIDEO_STREAM_INFORMATION Field Requirements (msg 269)

**Source:** [VehicleCameraControl.cc:1620-1637](src/Camera/VehicleCameraControl.cc#L1620-L1637)

The handler performs **no field validation**. Any message with a unique `stream_id` creates a new stream entry unconditionally.

| Field | Required / Checked? |
|---|---|
| `encoding` | **Not checked in handler.** Used later in VideoManager to pick `udp://` vs `udp265://`. |
| `uri` non-empty | **Not checked in handler.** Empty URI is silently accepted; VideoManager will fail later. |
| `framerate > 0` | **Not checked.** |
| `camera_device_id` | **Not checked anywhere.** Stored in `QGCVideoStreamInfo` but never used for routing or filtering. Value `0` is fine. |

The only field with meaningful downstream effect (without being validated) is `type` (`VIDEO_STREAM_TYPE_RTPUDP`, `RTSP`, etc.) — VideoManager switches on `pInfo->type()` to construct the pipeline URL, defaulting to `videoSourceNoVideo` for unknown types.

---

## Q6 — GStreamer Receive Pipeline

**Source:** [VideoManager.cc:429-462](src/VideoManager/VideoManager.cc#L429-L462), [GstVideoReceiver.cc:627-770](src/VideoManager/VideoReceiver/GStreamer/GstVideoReceiver.cc#L627-L770)

### URI Construction

QGC uses the `uri` field from `VIDEO_STREAM_INFORMATION` directly. For `VIDEO_STREAM_TYPE_RTPUDP` ([VideoManager.cc:442-448](src/VideoManager/VideoManager.cc#L442-L448)):

```cpp
case VIDEO_STREAM_TYPE_RTPUDP:
    if (pInfo->encoding() == VIDEO_STREAM_ENCODING_H265)
        url = uri.contains("udp265://") ? uri : QStringLiteral("udp265://0.0.0.0:%1").arg(uri);
    else
        url = uri.contains("udp://")    ? uri : QStringLiteral("udp://0.0.0.0:%1").arg(uri);
```

- If `uri` is a bare port number (e.g., `"5600"`), QGC wraps it as `udp://0.0.0.0:5600`.
- If `uri` is already `"udp://0.0.0.0:5600"`, it is used as-is.

### GStreamer Pipeline (H264/RTP UDP)

```
udpsrc uri="udp://0.0.0.0:<port>"
    caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264"
  → rtpjitterbuffer   (added automatically on RTP caps detection)
  → parsebin          (auto-detects RTP depay + H264 parse)
  → decodebin
  → video sink
```

| Question | Answer |
|---|---|
| Listens on `udp://0.0.0.0:5600` automatically? | Only as the VideoSettings **fallback default** when no MAVLink camera is present. Otherwise uses the URI from msg 269. |
| Uses URI from msg 269? | **Yes** — primary source for pipeline URI. |
| Specific RTP packetization (pt, mtu)? | **No `pt` set** — GStreamer auto-detects from caps. **No `mtu` set** — GStreamer default (1400 bytes). Clock-rate is hardcoded to `90000` Hz in the caps string. |

---

## Q7 — What `VehicleCameraControl::_initWhenReady()` Actually Waits On

**Source:** [VehicleCameraControl.cc:139-143](src/Camera/VehicleCameraControl.cc#L139-L143), [VehicleCameraControl.cc:182-211](src/Camera/VehicleCameraControl.cc#L182-L211), [VehicleCameraControl.cc:2110-2262](src/Camera/VehicleCameraControl.cc#L2110-L2262), [VehicleCameraControl.cc:2266-2275](src/Camera/VehicleCameraControl.cc#L2266-L2275), [QGCCameraIO.cc:262-279](src/Camera/QGCCameraIO.cc#L262-L279), [QGCCameraIO.cc:336-360](src/Camera/QGCCameraIO.cc#L336-L360)

`_initWhenReady()` does **not** block on camera parameters, `CAMERA_SETTINGS`, `STORAGE_INFORMATION`, or stream info. It is entered once camera definition handling is complete enough to proceed.

### What gates entry into `_initWhenReady()`

| Condition | What happens |
|---|---|
| `cam_definition_uri[0] == 0` | Constructor calls `_initWhenReady()` immediately. |
| Cached XML exists and parses | `_handleDefinitionFile()` emits `dataReady(bytes)`, then `_dataReady()` calls `_initWhenReady()`. |
| HTTP download succeeds or fails | `_downloadFinished()` emits `dataReady(data)`; `_dataReady()` then calls `_initWhenReady()` even if `data` is empty. |
| MAVLink FTP download succeeds | `_ftpDownloadComplete()` emits `dataReady(bytes)`; `_dataReady()` then calls `_initWhenReady()`. |
| Definition download fails | `_dataReady()` tries an offline definition file, then still calls `_initWhenReady()`. |

So the real gate is: **camera definition URI resolution/parsing path has finished**. It is not waiting for any runtime MAVLink state message.

### What `_initWhenReady()` starts, but does not wait for

| Startup action | Triggered by `_initWhenReady()`? | Waited on before return? |
|---|---|---|
| `PARAM_EXT_REQUEST_LIST` for camera params | **Yes**, for non-basic cameras | **No** |
| `CAMERA_SETTINGS` request | **Yes**, after 500 ms | **No** |
| video stream discovery | **Yes**, after 1000 ms | **No** |
| capture status polling | **Yes**, after 1500 ms | **No** |
| storage info request | **Yes**, after 2000 ms | **No** |

### What “basic camera” means here

`isBasic()` is defined as `_settings.size() == 0`. That means a camera is treated as “basic” if no settings were loaded from the camera definition file. For a basic camera:

- `_paramComplete` is set immediately.
- `parametersReady()` is emitted immediately.
- No `PARAM_EXT_REQUEST_LIST` is sent.

### When parameters are actually considered ready

For non-basic cameras, `_initWhenReady()` only starts parameter loading. Parameter readiness is completed later:

1. `_requestAllParameters()` sends `PARAM_EXT_REQUEST_LIST`.
2. Each camera setting has a `QGCCameraParamIO`.
3. Each `QGCCameraParamIO` marks itself done when it:
   - receives `PARAM_EXT_VALUE`,
   - times out after retries,
   - or is write-only.
4. `_paramDone()` checks whether all `QGCCameraParamIO` instances are done.
5. Only then does `VehicleCameraControl` set `_paramComplete = true` and emit `parametersReady()`.

Net: `_initWhenReady()` is a **bootstrap point**, not a “camera fully ready” barrier.

---

## Q8 — What Causes a `VehicleCameraControl` To Be Destroyed or Reset

**Source:** [QGCCameraManager.cc:153-163](src/Camera/QGCCameraManager.cc#L153-L163), [QGCCameraManager.cc:257-274](src/Camera/QGCCameraManager.cc#L257-L274), [QGCCameraManager.cc:279-319](src/Camera/QGCCameraManager.cc#L279-L319), [Vehicle.cc:367-390](src/Vehicle/Vehicle.cc#L367-L390), [MultiVehicleManager.cc:216-219](src/Vehicle/MultiVehicleManager.cc#L216-L219), [MultiVehicleManager.cc:243-264](src/Vehicle/MultiVehicleManager.cc#L243-L264), [VehicleCameraControl.cc:572-582](src/Camera/VehicleCameraControl.cc#L572-L582), [VehicleCameraControl.cc:680-700](src/Camera/VehicleCameraControl.cc#L680-L700)

### Creation path

`VehicleCameraControl` is created only after this sequence:

1. `QGCCameraManager` sees a heartbeat from a camera component ID.
2. It creates a `CameraStruct` and begins requesting `CAMERA_INFORMATION`.
3. When `CAMERA_INFORMATION` arrives, `_handleCameraInfo()` decodes it and calls:
   `firmwarePlugin()->createCameraControl(&info, _vehicle, message.compid, this)`.
4. The resulting `VehicleCameraControl` is added to the manager list.

So a new `VehicleCameraControl` instance is created from **successful `CAMERA_INFORMATION` reception**, not from parameter readiness and not from stream info.

### Destroyed because the camera disappears

If QGC has already received `CAMERA_INFORMATION` for a camera and then heartbeat stops for more than 5 seconds:

- `QGCCameraManager::_checkForLostCameras()` finds the corresponding control,
- removes it from `_cameras` / `_cameraLabels`,
- calls `pCamera->deleteLater()`,
- deletes the corresponding `CameraStruct`,
- resets current camera selection.

This is the main per-camera destruction path during normal runtime.

### Destroyed because the owning vehicle is deleted

When a vehicle is removed:

1. `MultiVehicleManager` emits `vehicleRemoved(vehicle)`.
2. It calls `vehicle->prepareDelete()`.
3. `Vehicle::prepareDelete()` deletes `_cameraManager` immediately and sets it to `nullptr`.

Since each `VehicleCameraControl` is created with the camera manager as parent, deleting the manager destroys all camera controls under it as well.

### Reset does not recreate the object

`VehicleCameraControl::resetSettings()` sends `MAV_CMD_RESET_CAMERA_SETTINGS`. On accepted ACK:

- basic cameras re-request `CAMERA_SETTINGS`;
- non-basic cameras re-request camera parameters and then re-request `CAMERA_SETTINGS`.

This is an **in-place reprobe/reload** of the existing `VehicleCameraControl`. It does **not** destroy and recreate the object.

### Summary table

| Event | Reuses same `VehicleCameraControl`? | Creates new object? | Destroys old object? |
|---|---|---|---|
| `MAV_CMD_RESET_CAMERA_SETTINGS` accepted | **Yes** | No | No |
| camera stops heartbeating for >5 s | No | Not immediately | **Yes** |
| vehicle removed from `MultiVehicleManager` | No | No | **Yes** |
| fresh `CAMERA_INFORMATION` after camera discovery | N/A | **Yes** | No |
