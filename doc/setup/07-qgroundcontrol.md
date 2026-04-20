# 7. QGroundControl Connection

Bring QGC up for the first time and verify all features end-to-end.

---

## 7.1 Install QGroundControl

Download from [qgroundcontrol.com](https://qgroundcontrol.com/) (Windows / macOS / Linux / Android). No special build needed — stock QGC works.

---

## 7.2 First connection

Launch QGC. With the rover daemon running and the Pi reachable (via WireGuard, LTE, or local USB), QGC should auto-connect on UDP port **14550** within ~2 seconds and show:

- **Vehicle 1** in the top-left vehicle selector
- **MAV_TYPE_GROUND_ROVER** icon
- A heartbeat indicator pulsing green

If QGC doesn't connect automatically, add a link manually: **Application Settings → Comm Links → Add** → Type: UDP, Port: 14550, Server Address: `pi-rover.lan` (when on WireGuard) or the Pi's LTE-reachable IP.

---

## 7.3 Expected telemetry

Once connected you should see:

| Panel / widget | What to expect |
|---|---|
| **Fly view — RSSI status** | Local RSSI and Remote RSSI bars; `txbuf=50` (WiFi) or `100` (LTE) |
| **Fly view — GPS** | Satellite count and fix type (2D/3D) once GPS has a lock |
| **Parameters** | 8 params: `DRIVE_*`, `GPS_RAW_LOG`, `VIDEO_*`, `NET_LINK_PREF`, `CTRL_TIMEOUT_MS` |
| **Video view** | One or more camera streams (click the camera icon in the top-right of Fly view) |
| **Message console** | Boot banner, GPS fix events, link-switch notifications |

---

## 7.4 Joystick setup

QGC → **Application Settings → Joysticks** → select your joystick, enable it, and map the two axes used by the daemon:

| Axis | Function |
|---|---|
| Pitch (or Throttle) | Forward / backward |
| Roll (or Yaw) | Left / right turn |

The daemon reads `MANUAL_CONTROL.x` (forward/back) and `MANUAL_CONTROL.r` (yaw). Drive parameters are tuned in the Parameters panel — start with defaults and adjust `DRIVE_DEAD_ZONE` (typically 30–100) and `DRIVE_SLEW_MS` (250–500) to taste.

---

## 7.5 Video stream

Each camera detected via `libcamera-hello --list-cameras` is advertised as a separate MAVLink camera component. QGC shows the camera icon in Fly view; click it to see the list of streams (one per sensor mode).

When you select a stream, QGC sends `MAV_CMD_VIDEO_START_STREAMING`; the daemon spawns a GStreamer pipeline that delivers H.264 RTP to QGC at `udp://0.0.0.0:5600`.

For details on sensor modes, CPU cost, and manual pipeline testing see [features/video.md](../features/video.md).

---

## 7.6 LTE / WiFi uplink switching

The `NET_LINK_PREF` parameter controls which uplink the modem uses:

| Value | Behavior |
|---|---|
| `0` | Auto — keep current uplink |
| `1` | Prefer WiFi |
| `2` | Force LTE |

Change the value in QGC's Parameters panel; the modem switches in ~2 seconds (LTE) or ~15–45 seconds (WiFi, due to WCNSS reload + DHCP). The video stream may freeze briefly but MAVLink and WireGuard auto-recover.

See [features/lte-uplink.md](../features/lte-uplink.md) for the implementation.

---

## 7.7 Runtime parameters quick reference

| Parameter | Default | Description |
|---|---|---|
| `DRIVE_DEAD_ZONE` | 30 | Joystick dead zone (0–1000) |
| `DRIVE_SLEW_MS` | 500 | Slew-rate ramp time (ms) |
| `DRIVE_TRIM` | 0 | Motor balance offset |
| `CTRL_TIMEOUT_MS` | 500 | Failsafe: stop if no input for this many ms |
| `GPS_RAW_LOG` | 0 | 1 = log raw NMEA sentences |
| `VIDEO_BITRATE` | 5000000 | H.264 encoder bitrate (bps) |
| `VIDEO_FPS` | 30 | Frame rate |
| `NET_LINK_PREF` | 0 | Uplink preference (0/1/2) |

Full details: [reference/parameters.md](../reference/parameters.md).

---

## Setup complete

The rover is ready to drive. For deep-dives on any feature, see the [features/](../features/) directory from the [main README](../../README.md).
