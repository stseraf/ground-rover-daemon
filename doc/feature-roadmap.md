# Feature Roadmap: ground-rover-daemon (Mini RC Rover)

## Context

Minimal RC rover on RPi Zero 2W, controlled primarily via QGroundControl over LTE.
ELRS radio is a failsafe/backup input. Current daemon handles QGC joystick→differential drive
over MAVLink UDP; it needs real hardware I/O, tunable config, sensor telemetry, and
link monitoring to become a deployable field system.

---

## Architecture Boundaries

### In ground-rover-daemon (this repo)
All real-time control, MAVLink protocol, sensor I/O, and safety logic.

### Separate / System-level
| Concern | Approach |
|---|---|
| Video streaming | Standalone systemd service (GStreamer or camera-streamer); daemon advertises RTSP URL via `VIDEO_STREAM_INFORMATION` |
| LTE networking | ModemManager + NetworkManager; daemon treats it as transparent IP — but monitors signal quality |
| RPi Zero 2W deployment | Cross-compile toolchain + systemd unit — done (`deploy/ground-rover-daemon.service`, `make deploy`) |

---

## Feature Backlog (Prioritized)

### Feature 1 — Motor Driver Abstraction + Gimbal Control — DONE

Motor backends behind a common `IMotorDriver` interface:
- `TB6612Driver` — direct RPi GPIO PWM via `/sys/class/pwm`
- `UartMotorDriver` — no-op stub; UART motor backend reserved for future use

Gimbal backends behind a common `IGimbalController` interface:
- `I2cGimbalController` — 4-byte binary write over `/dev/i2c-1` to Arduino slave
- `StubGimbalController` — no-op stub for host/GIMBAL=stub builds

Also delivered: MANUAL_CONTROL 500 ms failsafe, QGC camera discovery, codebase
restructured into topic modules (`src/motor/`, `src/mavlink/`, `src/drive/`, `src/gimbal/`).

---

### Feature 2 — MAVLink Parameter Protocol — DONE (merged dev 2026-03-29)

Full `PARAM_REQUEST_LIST` / `PARAM_REQUEST_READ` / `PARAM_SET` / `PARAM_VALUE` exchange.
Parameter store: in-memory array + flat file persistence (`params`).

**Parameters:**
| Name | Default | Description |
|---|---|---|
| `DRIVE_DEAD_ZONE` | 50 | Joystick dead zone (axis units ±1000) |
| `DRIVE_SLEW_MS` | 250 | Motor slew time (ms) |
| `DRIVE_TRIM` | 0 | Left/right motor trim offset |
| `CTRL_TIMEOUT_MS` | 300 | MANUAL_CONTROL timeout before failsafe |
| `GPS_RAW_LOG` | 0 | Toggle raw NMEA sentence logging at runtime |

---

### Feature 3 — ELRS RX Integration (radio failsafe) — PLANNED

**Blocked:** RPi Zero 2W hardware UART (`/dev/serial0`) is allocated to the GPS module.
ELRS requires a dedicated UART at 420000 baud with inverted logic — revisit when a
USB UART adapter is added to the hardware stack.

**Scope:**
- CRSF frame parser (UART, 420000 baud, inverted logic via ELRS module)
- Channel mapping: ch1=throttle, ch2=steering (configurable via params)
- Control source priority: QGC active → ignore ELRS; QGC silent >N ms → switch to ELRS
- Failsafe state: all channels at center if ELRS also drops
- MAVLink `RC_CHANNELS` telemetry forwarded to QGC

---

### Feature 4 — GPS Module + MAVLink Telemetry — DONE (merged dev 2026-03-30)

GY-GPS6MV2 / u-blox NEO-M8N over UART NMEA 0183 (`GPS=nmea` build variant).

**Delivered:**
- `IGpsProvider` interface; `NmeaGpsProvider` (UART) and `StubGpsProvider` (host builds)
- Parses `$GNRMC` + `$GNGGA`; sends `GPS_RAW_INT` and `GLOBAL_POSITION_INT` to QGC at 1 Hz
- State-change event logging: fix acquired/lost, visible satellites per constellation,
  signal quality class (good/moderate/poor), antenna status, module identity on boot
- `GPS_RAW_LOG` parameter to toggle raw NMEA output at runtime
- Home altitude set on first valid fix for `relative_alt` in `GLOBAL_POSITION_INT`

**RPi UART setup:** disable BT overlay (`dtoverlay=disable-bt` in `/boot/firmware/config.txt`),
disable serial console via `raspi-config` → `/dev/serial0` resolves to `ttyAMA0` (PL011).

---

### Feature 5 — LTE Link Monitoring — PLANNED (depends on 3)

**Scope:**
- Poll `ModemManager` via D-Bus or `mmcli` subprocess for signal quality (RSSI/RSRP)
- Send signal strength in `SYS_STATUS` extended fields or custom MAVLink message
- Configurable drop timeout → same failsafe as ELRS (stop motors / hold)
- Log LTE state transitions

---

### Feature 6 — Basic Autopilot Modes — PLANNED (depends on 1, 4)

**Scope:**
- **HOLD mode:** stop motors and hold position using GPS; engage when QGC switches mode
- **RETURN mode:** proportional controller drives rover back to home position (set on arm)
- Mode state tracked in `RoverState::custom_mode`, reported via `HEARTBEAT` and `CURRENT_MODE`

**Note on QGC mode switching:** QGC does not expose a mode switcher UI for
`MAV_AUTOPILOT_GENERIC` vehicles regardless of `AVAILABLE_MODES` / `CURRENT_MODE`
support. Mode switching will be triggered via `COMMAND_LONG` / `SET_MODE`, not QGC UI.

---

## What Is Explicitly Out of Scope (for daemon)
- Video streaming pipeline (separate systemd service)
- LTE network bring-up (ModemManager/NetworkManager)
- Cross-compile/deployment scripts — done (`make deploy` + systemd service)
- Structured logging (spdlog) — simple `logger.hpp` shim is sufficient; spdlog dropped from backlog

---

## Implementation Order Summary

| # | Feature | Status | Depends on |
|---|---|---|---|
| 1 | Motor driver (TB6612) + I2C gimbal | done | — |
| 2 | MAVLink param protocol + persistence | done (2026-03-29) | — |
| 3 | ELRS RX + failsafe logic | planned (UART blocked) | 1, 2 |
| 4 | GPS module + MAVLink telemetry | done (2026-03-30) | — |
| 5 | LTE monitoring + failsafe | planned | 3 |
| 6 | Basic autopilot modes (HOLD, RETURN) | planned | 1, 4 |

Features 1, 2, and 4 had no interdependencies and were built independently.
Feature 6 is now unblocked (depends on 1 and 4, both done).
Feature 3 is the current blocker for Feature 5.
