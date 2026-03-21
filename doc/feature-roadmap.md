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
| RPi Zero 2W deployment | Cross-compile toolchain + systemd unit file (separate from feature work) |

---

## Feature Backlog (Prioritized)

### Priority 1 — Motor Driver Abstraction (MVP hardware control)
**What:** Replace stub `SERVO_OUTPUT_RAW` telemetry with real motor output.
Two backends behind a common interface:
- `TB6612Driver` — direct RPi GPIO PWM via `/sys/class/pwm` or pigpio
- `UartDriver` — simple framed protocol over UART to an external motor controller

**Scope:**
- Abstract `IMotorDriver` interface (`set(left, right)` in PWM µs)
- Both backends; runtime selection via config param or build flag
- TB6612: PWM on 2 GPIO pairs + STBY pin
- UART: define minimal frame format (e.g. `[0xAA][left_hi][left_lo][right_hi][right_lo][csum]`)

**Why first:** No real rover without actual motor output. Everything else is telemetry.

---

### Priority 2 — MAVLink Parameter Protocol (config R/W from QGC)
**What:** Replace 3 dummy params with a real, persistent parameter store.
QGC can read/write params at runtime; values survive restart.

**Scope:**
- Implement full `PARAM_REQUEST_LIST`, `PARAM_REQUEST_READ`, `PARAM_SET`, `PARAM_VALUE` exchange
- Parameter store: in-memory map + flat file persistence (`/etc/rover/params.cfg`)
- Initial params: `DRIVE_SLEW_MS`, `DRIVE_DEAD_ZONE`, `MOTOR_BACKEND` (0=TB6612, 1=UART), UART baud, TB6612 GPIO pins
- Hook param changes into live config (e.g. slew time takes effect immediately)

**Why second:** Enables tuning without recompiling; needed before field testing.

---

### Priority 3 — ELRS RX Integration (radio failsafe)
**What:** Read CRSF (Crossfire Serial) frames from ELRS RX module over UART.
Primary role: failsafe — if LTE/QGC link drops, ELRS takes over motor control.

**Scope:**
- CRSF frame parser (UART, 420000 baud, inverted logic via ELRS module)
- Channel mapping: ch1=throttle, ch2=steering (configurable via params)
- Control source priority logic: QGC joystick active → ignore ELRS; QGC silent >N ms → switch to ELRS
- Failsafe state: all channels at center if ELRS also drops
- MAVLink `RC_CHANNELS` telemetry forwarded to QGC

**Why third:** Safety-critical — rover must not run away if LTE drops.

---

### Priority 4 — GPS Module (position telemetry to QGC)
**What:** Read NMEA or UBX from a UART/USB GPS module; forward as MAVLink.

**Scope:**
- NMEA parser (GGA, RMC) or UBX binary (u-blox)
- Send `GPS_RAW_INT`, `GLOBAL_POSITION_INT` to QGC
- Display rover position on QGC map
- Store home position on arm for RTH reference

**Why fourth:** Enables position awareness in QGC; prerequisite for autopilot modes.

---

### Priority 5 — LTE Link Monitoring
**What:** Daemon reads modem signal quality and reports it; triggers failsafe on drop.

**Scope:**
- Poll `ModemManager` via D-Bus or `mmcli` subprocess for signal quality (RSSI/RSRP)
- Send signal strength in `SYS_STATUS` extended fields or custom MAVLink msg
- Configurable drop timeout → same failsafe as ELRS (stop motors / hold)
- Log LTE state transitions

**Why fifth:** Completes the reliability picture; ELRS failsafe covers the gap.

---

### Priority 6 — Basic Autopilot Modes (requires GPS)
**What:** A few simple autonomous behaviors beyond manual.

**Scope:**
- `HOLD` — stop and hold position (GPS-assisted)
- `RETURN` — navigate back to home position (basic waypoint logic)
- Mode commanded via QGC flight mode buttons (already handled by `handle_set_mode`)
- Simple proportional controller; no full PID stack needed initially

**Why last:** Needs GPS + tuned drive + stable link — all the above.

---

## What Is Explicitly Out of Scope (for daemon)
- Video streaming pipeline (separate systemd service)
- LTE network bring-up (ModemManager/NetworkManager)
- Cross-compile/deployment scripts (separate concern, tracked separately)

---

## Implementation Order Summary

| # | Feature | Depends on |
|---|---|---|
| 1 | Motor driver abstraction (TB6612 + UART) | nothing |
| 2 | MAVLink param protocol + persistence | nothing |
| 3 | ELRS RX + failsafe logic | 1, 2 |
| 4 | GPS module + MAVLink telemetry | nothing |
| 5 | LTE monitoring + failsafe | 3 |
| 6 | Autopilot modes | 4, 1 |

Features 1, 2, and 4 can be worked in parallel (no interdependencies).
