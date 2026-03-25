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

### Priority 1 — Motor Driver Abstraction + Gimbal Control (MVP hardware control) — IN PROGRESS
**What:** Replace stub `SERVO_OUTPUT_RAW` telemetry with real motor output, and add
2-axis camera gimbal control via I2C to an Arduino acting as I2C slave.

Motor backends behind a common `IMotorDriver` interface:
- `TB6612Driver` — direct RPi GPIO PWM via `/sys/class/pwm` (complete)
- `UartMotorDriver` — no-op stub; UART motor backend reserved for future use

Gimbal backends behind a common `IGimbalController` interface:
- `I2cGimbalController` — 4-byte binary write over `/dev/i2c-1` to Arduino slave
- `StubGimbalController` — no-op stub for host/GIMBAL=stub builds

**Scope:**
- [x] Abstract `IMotorDriver` interface (`set(left, right)` in ±1000 axis units)
- [x] TB6612 backend; build flag `DRIVER=tb6612`
- [x] TB6612: PWM at 50 Hz on 2 GPIO pairs + STBY pin; driver released on disarm
- [x] `UartMotorDriver` stub retained — UART motor backend remains a future option
- [x] Abstract `IGimbalController` interface (`set(pan, tilt)`, `center()`)
- [ ] I2C gimbal controller: pan=`mc.x`, tilt=`mc.r`; build flag `GIMBAL=i2c`
- [ ] I2C protocol: 4-byte big-endian int16 write to Arduino at `Config::Gimbal::I2C_ADDR`
- [ ] Gimbal always active (not arm-gated); holds last position on MC timeout
- [ ] `Config::Gimbal` namespace: `I2C_BUS` (`/dev/i2c-1`), `I2C_ADDR` (0x10), `DEAD_ZONE`

**Arduino firmware contract:**
- I2C slave at address `0x10` (default); receives 4-byte frames `[pan_hi][pan_lo][tilt_hi][tilt_lo]`
- Map ±1000 → 0°–180°: `angle = (value + 1000) * 180 / 2000`
- Buffer bytes in `Wire.onReceive()` ISR; call `Servo.write()` from `loop()` only

**Also delivered in this feature:**
- MANUAL_CONTROL 500 ms failsafe — stops motors if QGC link goes silent
- QGC camera discovery cycle completed to stop repeated polling spam
- Codebase restructured into topic modules (`src/motor/`, `src/mavlink/`, `src/drive/`, `src/gimbal/`)

**Why first:** No real rover without actual motor output. Gimbal included here as it shares
the same MANUAL_CONTROL axis data path and requires no additional MAVLink plumbing.

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

### Priority 3 — Structured Logging (spdlog)
**What:** Replace the minimal `logger.hpp` printf shim with a production-grade logging
stack using spdlog. Enables file persistence, per-subsystem log levels, and MAVLink
STATUSTEXT forwarding to QGC — essential for field diagnostics on RPi Zero.

**Scope (4 phases):**

#### Phase 1 — Foundation
- Add spdlog via CMake FetchContent, pinned to v1.13+. Verify async logger compiles on arm64.
- `RoverLogger` singleton (`logger.hpp` rework): `init_thread_pool(8192, 1)`, two sinks:
  `stdout_color_sink_mt` + `rotating_file_sink_mt` (5 MB × 3 files).
  Pattern: `[timestamp] [level] [subsystem] message`.
  `async_overflow_policy::discard_new` — never block the MAVLink loop.
- Replace all `logger::line` / `logger::same_line` calls with `spdlog::info/warn/error`.
  Console sink active in debug builds only.

#### Phase 2 — Subsystem Loggers + MAVLink STATUSTEXT
- Named loggers: `spdlog::get("mavlink")`, `("rover")`, `("system")`.
  Per-subsystem log level override at runtime without recompile.
- Custom sink extending `base_sink<std::mutex>`: forwards WARN+ messages as
  `MAVLINK_MSG_ID_STATUSTEXT` to QGC. Maps spdlog levels → `MAV_SEVERITY` enum.
  Hooks into existing `send_message()` path.

#### Phase 3 — Telemetry & SD Card Hygiene *(pair with Priority 5 — GPS)*
- `TelemetryLogger` class: CSV output (`timestamp_ms, lat, lon, heading, speed_mps, mode, armed`).
  Buffered writes — flush every 50 rows to avoid SD card wear on RPi Zero.
- `tmpfs` mount for runtime logs (`/var/log/rover`, 32 MB in `/etc/fstab`).
  Systemd oneshot syncs to SD on clean shutdown.
- `logrotate` config (`/etc/logrotate.d/rover`): daily, rotate 7, compress, delaycompress.

#### Phase 4 — Runtime Log Level via MAVLink Param *(requires Priority 2 — Param Protocol)*
- Expose `LOG_LEVEL` as a MAVLink parameter (`PARAM_SET`).
  Receiving it calls `spdlog::set_level()` at runtime.
  Allows toggling debug verbosity from QGC without restarting the daemon.

**Why third:** Foundation (Phases 1-2) should land before ELRS/GPS add new subsystems.
Mechanical printf→spdlog swap is cheapest while the codebase is still one file. STATUSTEXT
sink makes ELRS failsafe events visible in QGC from day one of Priority 4 work.
Phases 3-4 are explicitly gated on their dependencies (GPS and Param Protocol respectively).

---

### Priority 4 — ELRS RX Integration (radio failsafe) — DEPRIORITIZED

**Status:** Deprioritized. The RPi Zero 2W hardware UART (`/dev/serial0`) is allocated
to the GPS module (Priority 5). ELRS requires a dedicated UART at 420000 baud with
inverted logic — revisit when a USB UART adapter is added to the hardware stack.

**Original scope (preserved for reference):**
- CRSF frame parser (UART, 420000 baud, inverted logic via ELRS module)
- Channel mapping: ch1=throttle, ch2=steering (configurable via params)
- Control source priority: QGC active → ignore ELRS; QGC silent >N ms → switch to ELRS
- Failsafe state: all channels at center if ELRS also drops
- MAVLink `RC_CHANNELS` telemetry forwarded to QGC

---

### Priority 5 — GPS Module (position telemetry to QGC)
**What:** Read NMEA or UBX from a UART/USB GPS module; forward as MAVLink.

**Scope:**
- NMEA parser (GGA, RMC) or UBX binary (u-blox)
- Send `GPS_RAW_INT`, `GLOBAL_POSITION_INT` to QGC
- Display rover position on QGC map
- Store home position on arm for RTH reference

**UART device:** `/dev/serial0` (configurable). PL011 (`/dev/ttyAMA0`) is preferred over
mini UART (`/dev/ttyS0`) for baud rate stability. If `/dev/serial0` resolves to ttyS0,
disable the Bluetooth overlay or remap BT to mini UART in `/boot/firmware/config.txt`.

**Why fifth:** Enables position awareness in QGC; prerequisite for autopilot modes.

---

### Priority 6 — LTE Link Monitoring
**What:** Daemon reads modem signal quality and reports it; triggers failsafe on drop.

**Scope:**
- Poll `ModemManager` via D-Bus or `mmcli` subprocess for signal quality (RSSI/RSRP)
- Send signal strength in `SYS_STATUS` extended fields or custom MAVLink msg
- Configurable drop timeout → same failsafe as ELRS (stop motors / hold)
- Log LTE state transitions

**Why sixth:** Completes the reliability picture; ELRS failsafe covers the gap.

---

## What Is Explicitly Out of Scope (for daemon)
- Video streaming pipeline (separate systemd service)
- LTE network bring-up (ModemManager/NetworkManager)
- Cross-compile/deployment scripts (separate concern, tracked separately)
- **Flight mode switching in QGC** — QGC does not expose a mode switcher UI for
  `MAV_AUTOPILOT_GENERIC` vehicles regardless of `AVAILABLE_MODES` / `CURRENT_MODE`
  microservice support. Switching to `MAV_AUTOPILOT_ARDUPILOTMEGA` enables the UI but
  triggers QGC's mandatory ArduPilot parameter validation which is not worth implementing.
  Mode state is tracked internally (`RoverState::custom_mode`) and reported via HEARTBEAT
  and CURRENT_MODE, but QGC cannot be used to switch modes on this vehicle type.

---

## Implementation Order Summary

| # | Feature | Status | Depends on |
|---|---|---|---|
| 1 | Motor driver (TB6612) + I2C gimbal | TB6612 done; gimbal in progress | nothing |
| 2 | MAVLink param protocol + persistence | not started | nothing |
| 3 | Structured logging (spdlog, Phases 1-2) | not started | nothing |
| 4 | ELRS RX + failsafe logic | DEPRIORITIZED (UART conflict) | 1, 2 |
| 5 | GPS module + MAVLink telemetry | not started | nothing |
| 6 | LTE monitoring + failsafe | not started | 4 |

Features 1, 2, 3 (Phases 1-2), and 5 can be worked in parallel (no interdependencies).
Logging Phases 3-4 are gated: Phase 3 (CSV telemetry) pairs with Feature 5 (GPS); Phase 4 (log level param) requires Feature 2 (param protocol).
