# MAVLink Parameters Reference

All runtime-tunable parameters exposed by the daemon over the MAVLink parameter protocol (`PARAM_REQUEST_LIST`, `PARAM_REQUEST_READ`, `PARAM_SET`, `PARAM_VALUE`). QGC shows them in the **Parameters** panel; values persist to a plain-text `params` file next to the binary.

Source: [`src/mavlink/param_store.cpp`](../../src/mavlink/param_store.cpp).

---

## Parameter list

| # | Name | Type | Default | Recommended | Range | Effect |
|---|---|---|---|---|---|---|
| 0 | `DRIVE_DEAD_ZONE` | int | 30 | **50** | 0–1000 | Joystick inputs below ±N are treated as zero. Prevents motor jitter from stick imprecision; increase if the rover creeps when the stick is released. |
| 1 | `DRIVE_SLEW_MS` | int | 500 | **250** | 0–5000 | Time to ramp from 0 to full motor power (ms). Lower = snappier response. 0 = instant (hard jerk). |
| 2 | `DRIVE_TRIM` | int | 0 | **−45** | −500 to +500 | Left/right motor balance. Positive → more power to left motor (corrects veer-right). Negative → more power to right (corrects veer-left). Tune by driving straight and adjusting until the rover tracks a straight line. |
| 3 | `CTRL_TIMEOUT_MS` | int | 500 | **300** | 50–5000 | Failsafe: slew motors to zero if no `MANUAL_CONTROL` packet arrives within this time. Lower = faster stop on link glitch; too low causes spurious stops on a busy network. |
| 4 | `GPS_RAW_LOG` | bool | 0 | **0** | 0 or 1 | 1 = also print raw NMEA sentences to daemon stdout (journalctl). Useful for diagnosing GPS issues; keep off in normal operation. |
| 5 | `VIDEO_BITRATE` | int | 5 000 000 | **5 000 000** | 25 000–25 000 000 | H.264 encoder target bitrate (bps). 5 Mbps is a good balance for LTE — raise for local WiFi, lower if the stream stutters on LTE. |
| 6 | `VIDEO_FPS` | int | 30 | **30** | 1–60 | Frame rate cap. Capped further by the sensor mode's physical limit; 1296×972 mode tops out at ~46 fps. |
| 7 | `NET_LINK_PREF` | enum | 0 | **1** | 0/1/2 | 0 = auto (keep current uplink), 1 = prefer WiFi (switch to home WiFi when available), 2 = force LTE. Set to 1 when home WiFi is provisioned — the modem falls back to LTE automatically if WiFi drops. |

All parameters are `MAV_PARAM_TYPE_REAL32` on the wire; the daemon rounds to int where appropriate internally. The `params` file may store floats in scientific notation (e.g., `VIDEO_BITRATE=5e+06`) — this is equivalent to `5000000` and handled correctly.

---

## Persistence

On boot the daemon reads `params` (a plain-text `KEY=VALUE` file) relative to the process `WorkingDirectory` (`/home/pi/ground-rover-daemon/`):

```
DRIVE_DEAD_ZONE=50
DRIVE_SLEW_MS=250
DRIVE_TRIM=-45
CTRL_TIMEOUT_MS=300
GPS_RAW_LOG=0
VIDEO_BITRATE=5e+06
VIDEO_FPS=30
NET_LINK_PREF=1
```

When QGC sends `PARAM_SET`, the daemon writes the file back atomically. Missing keys fall back to compile-time defaults.

The file is safe to edit by hand — stop the service, edit, start:

```bash
sudo systemctl stop ground-rover-daemon
nano /home/pi/ground-rover-daemon/params
sudo systemctl start ground-rover-daemon
```

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
