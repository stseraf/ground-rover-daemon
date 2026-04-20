# 7. QGroundControl Connection

Bring QGC up for the first time and verify all features end-to-end.

---

## 7.1 Install QGroundControl

Download from [qgroundcontrol.com](https://qgroundcontrol.com/) (Windows / macOS / Linux / Android). No special build needed — stock QGC works.

---

## 7.2 Add a comm link (required for WireGuard)

QGC's auto-discover uses UDP broadcast, which does not cross a WireGuard tunnel. **You must add a manual comm link the first time.**

**Application Settings → Comm Links → Add**

| Field | Value |
|---|---|
| Name | `rover` (or anything) |
| Type | UDP |
| Listening Port | 14550 |
| Server Address | `pi-rover.lan` *(when on home LAN / WireGuard)* |
| High Latency | off |

Click **Connect**, then **OK**. Tick **Automatically Connect on Start** so you don't have to repeat this.

Once connected, QGC shows:

- **Vehicle 1** in the top-left vehicle selector
- **MAV_TYPE_GROUND_ROVER** icon
- A heartbeat indicator pulsing green

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

When you select a stream, QGC sends `MAV_CMD_VIDEO_START_STREAMING`; the daemon spawns a GStreamer pipeline that delivers H.264 RTP to QGC's IP on port **5600**.

The GStreamer pipeline sends H.264 RTP to the QGC machine's IP on port 5600. The daemon normally learns this IP from the first MAVLink packet, but in practice it is more reliable to set it explicitly in the `qgc_ip` file:

```bash
# on the Pi, in /home/pi/ground-rover-daemon/
echo "192.168.50.46" > qgc_ip
sudo systemctl restart ground-rover-daemon
```

Use the **local LAN IP** of the machine running QGC (not its WireGuard IP — the Pi routes home-LAN traffic through the tunnel). Find it on macOS: `System Settings → Network` or run `ifconfig | grep "inet "` in Terminal; on Windows: `ipconfig`; on Linux: `ip addr`.

The `qgc_ip` file holds one IP with no port and is read once at startup.

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

| Parameter | Recommended | Description |
|---|---|---|
| `DRIVE_DEAD_ZONE` | **50** | Joystick dead zone — inputs below ±50 are zero; prevents creep from stick slop |
| `DRIVE_SLEW_MS` | **250** | Ramp time 0 → full power; snappier than the 500 ms default |
| `DRIVE_TRIM` | **−45** | Motor balance — adjust until the rover drives straight (negative = boost right motor) |
| `CTRL_TIMEOUT_MS` | **300** | Failsafe stop if no joystick packet for 300 ms |
| `GPS_RAW_LOG` | **0** | 0 = off; 1 = print raw NMEA to logs (debug only) |
| `VIDEO_BITRATE` | **5 000 000** | H.264 bitrate (bps); 5 Mbps suits LTE |
| `VIDEO_FPS` | **30** | Frame rate cap |
| `NET_LINK_PREF` | **1** | 1 = prefer WiFi when in range, fall back to LTE automatically |

Full details and compile-time defaults: [reference/parameters.md](../reference/parameters.md).

---

## Setup complete

The rover is ready to drive. For deep-dives on any feature, see the [features/](../features/) directory from the [main README](../../README.md).
