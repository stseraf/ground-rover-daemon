# MAVLink Parameters Reference

All runtime-tunable parameters exposed by the daemon over the MAVLink parameter protocol (`PARAM_REQUEST_LIST`, `PARAM_REQUEST_READ`, `PARAM_SET`, `PARAM_VALUE`). QGC shows them in the **Parameters** panel; values persist to a plain-text `params` file next to the binary.

Source: [`src/mavlink/param_store.cpp`](../../src/mavlink/param_store.cpp).

---

## Parameter list

| # | Name | Type | Default | Range | Effect |
|---|---|---|---|---|---|
| 0 | `DRIVE_DEAD_ZONE` | int | 30 | 0–1000 | Joystick dead zone (axis units). Inputs below ±N are zero. |
| 1 | `DRIVE_SLEW_MS` | int | 500 | 0–5000 | Ramp time from 0 to full motor power (ms). |
| 2 | `DRIVE_TRIM` | int | 0 | −500 to +500 | Left/right motor balance. Positive → more left. |
| 3 | `CTRL_TIMEOUT_MS` | int | 500 | 50–5000 | Failsafe: stop motors after this many ms with no `MANUAL_CONTROL`. |
| 4 | `GPS_RAW_LOG` | bool | 0 | 0 or 1 | 1 = also log raw NMEA sentences to stdout. |
| 5 | `VIDEO_BITRATE` | int | 5 000 000 | 25 000 – 25 000 000 | H.264 encoder target bitrate (bps). |
| 6 | `VIDEO_FPS` | int | 30 | 1 – 60 | Video frame rate cap. |
| 7 | `NET_LINK_PREF` | enum | 0 | 0/1/2 | 0 = auto, 1 = WiFi-prefer, 2 = LTE-force. |

All parameters are `MAV_PARAM_TYPE_REAL32` on the wire; the daemon rounds to int where appropriate internally.

---

## Persistence

On boot the daemon reads `params` (a plain-text `KEY=VALUE` file) relative to the process `WorkingDirectory`:

```
DRIVE_DEAD_ZONE=30
DRIVE_SLEW_MS=500
DRIVE_TRIM=0
CTRL_TIMEOUT_MS=500
GPS_RAW_LOG=0
VIDEO_BITRATE=5000000
VIDEO_FPS=30
NET_LINK_PREF=0
```

When QGC sends `PARAM_SET`, the daemon writes the file back atomically. Missing keys fall back to their compile-time defaults.

The file is safe to edit by hand (stop the service, edit, start).

---

## Feature links

- `DRIVE_*` + `CTRL_TIMEOUT_MS` → [../features/drive.md](../features/drive.md)
- `GPS_RAW_LOG` → [../features/gps.md](../features/gps.md)
- `VIDEO_*` → [../features/video.md](../features/video.md)
- `NET_LINK_PREF` → [../features/lte-uplink.md](../features/lte-uplink.md)

---

## Compile-time defaults

The starting values come from `include/config.hpp`:

| Config constant | Parameter default |
|---|---|
| `Config::DRIVE_DEAD_ZONE` | `DRIVE_DEAD_ZONE` |
| `Config::DRIVE_SLEW_TIME_MS` | `DRIVE_SLEW_MS` |
| `Config::MC_TIMEOUT_US / 1000` | `CTRL_TIMEOUT_MS` |

The `VIDEO_*`, `NET_LINK_PREF`, and `GPS_RAW_LOG` defaults are hard-coded in `ParamStore` — edit `src/mavlink/param_store.cpp` to change them.
