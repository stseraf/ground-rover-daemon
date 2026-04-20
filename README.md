# Ground Rover Daemon

A field-deployable differential-drive rover built on a Raspberry Pi Zero 2W and controlled from [QGroundControl](https://qgroundcontrol.com/) over LTE. The daemon is the MAVLink bridge between QGC and the hardware: joystick in, motor PWM out, GPS + radio telemetry back, H.264 video to QGC.

```
  ┌─── QGC (operator PC) ────────────┐
  │  joystick · video · parameters   │
  └──────────────┬───────────────────┘
                 │ MAVLink UDP + H.264 RTP
                 ▼
  ┌─── Router (WireGuard peer) ──────┐
  │  VPN endpoint                    │
  └──────────────┬───────────────────┘
                 │ encrypted UDP over Internet
                 ▼
  ┌─── UZ801 LTE modem ──────────────┐
  │  rover gateway · LTE ⇄ WiFi      │
  │  NAT / port-forward to Pi        │
  └──────────────┬───────────────────┘
                 │ USB (RNDIS + ADB)
                 ▼
  ┌─── Raspberry Pi Zero 2W ─────────┐
  │  ground_rover_daemon             │
  │   ├─ TB6612 motor driver         │
  │   ├─ I2C gimbal (Arduino) *      │
  │   ├─ NMEA GPS (NEO-6M)           │
  │   └─ GStreamer H.264 (MIPI cam)  │
  └──────────────────────────────────┘

  * designed and implemented, not yet hardware-tested
```

---

## Getting started

Fresh Pi to ready-to-drive rover in seven steps. Start from [doc/setup/01-raspberry-pi.md](doc/setup/01-raspberry-pi.md) and work through in order.

| # | Step | What it covers |
|---|---|---|
| 1 | [Raspberry Pi OS setup](doc/setup/01-raspberry-pi.md) | Flash Pi OS, edit `config.txt` (PWM, UART, I2C, GPU memory), permissions |
| 2 | [Power topology](doc/setup/02-power.md) | 2s2p 18650 pack, XL4015 DC-DC, rail distribution, bench-debug mode |
| 3 | [Hardware wiring](doc/setup/03-wiring.md) | TB6612, GPS, camera, optional gimbal, custom UZ801 cable |
| 4 | [Build & deploy the daemon](doc/setup/04-build-deploy.md) | Cross-compile for aarch64, `make deploy`, systemd service |
| 5 | [UZ801 LTE modem setup](doc/setup/05-lte-modem.md) | ADB, `make deploy-modem`, optional home WiFi credentials |
| 6 | [WireGuard VPN](doc/setup/06-wireguard.md) | Tunnel to the router so QGC can reach the rover on any uplink |
| 7 | [QGroundControl connection](doc/setup/07-qgroundcontrol.md) | First connection, joystick, video stream, runtime parameters |

**Tested on:** Raspberry Pi OS (Debian GNU/Linux 13 "Trixie") 13.4, 64-bit, kernel 6.12.75. Newer releases should work.

---

## Features

| # | Feature | Status | Doc |
|---|---|---|---|
| 1 | Differential drive (TB6612) | ✅ done | [drive.md](doc/features/drive.md) |
| 1a | I2C gimbal (Arduino + servos) | 🧪 implemented, not yet hardware-tested (gimbal build descoped) | [gimbal.md](doc/features/gimbal.md) |
| 2 | MAVLink parameter protocol + persistence | ✅ done | [parameters.md](doc/reference/parameters.md) |
| 3 | ELRS RX (radio failsafe) | ⏸ blocked (UART in use) | see *Improvement plan* |
| 4 | GPS module + MAVLink telemetry | ✅ done | [gps.md](doc/features/gps.md) |
| 5 | LTE link monitoring + uplink switching | ✅ done | [lte-uplink.md](doc/features/lte-uplink.md) |
| 6 | Video streaming (GStreamer + H.264 RTP) | ✅ done | [video.md](doc/features/video.md) |
| 7 | Autopilot modes (HOLD, RETURN) | 🗓 planned | see *Improvement plan* |
| 8 | INA219 battery voltage + current monitoring | 🗓 planned | see *Improvement plan* |
| 9 | LuckFox Pico Mini + SC3336 (alt hardware) | 🔬 under evaluation | see *Improvement plan* |
| 10 | Custom ground station (alt / from-scratch) | 🔬 under evaluation | see *Improvement plan* |

---

## Documentation index

### Feature deep-dives — [doc/features/](doc/features/)

- [drive.md](doc/features/drive.md) — joystick → differential mix → TB6612 PWM, dead zone, slew, trim, failsafe
- [gps.md](doc/features/gps.md) — NEO-6M over UART, NMEA parsing, `GPS_RAW_INT` / `GLOBAL_POSITION_INT`
- [gimbal.md](doc/features/gimbal.md) — I2C protocol to the Arduino slave, 4-byte frame format, firmware contract
- [video.md](doc/features/video.md) — camera discovery, sensor modes, GStreamer pipeline, CPU cost, latency
- [lte-uplink.md](doc/features/lte-uplink.md) — modem status polling, `NET_LINK_PREF` switching, WireGuard auto-recovery

### Reference — [doc/reference/](doc/reference/)

- [parameters.md](doc/reference/parameters.md) — all 8 runtime-tunable MAVLink parameters
- [mavlink-camera-protocol.md](doc/reference/mavlink-camera-protocol.md) — how the daemon implements the camera side of MAVLink
- [qgc-camera-internals.md](doc/reference/qgc-camera-internals.md) — how QGC discovers and drives cameras (source-level notes)
- [uz801-internals.md](doc/reference/uz801-internals.md) — UZ801 hardware, Android, WCNSS quirks, LED semantics

---

## Improvement plan

Features in the backlog, not yet implemented.

### LuckFox Pico Mini + SC3336 camera (alternative hardware platform)

The LuckFox Pico Mini (RV1103, Cortex-A7 + RISC-V, 64 MB DDR2) is a candidate replacement or parallel target for the Pi Zero 2W — smaller form factor, built-in VPU for hardware H.264, native MIPI CSI for the SC3336 (3 MP, 2304 × 1296).

**Evaluation goals:**
- Build toolchain: confirm the daemon compiles for `arm-linux-gnueabihf` (RV1103 runs 32-bit ARMv7 Linux); add `ARCH=luckfox` target to Makefile
- Camera pipeline: `libcamera` is not available on LuckFox — evaluate `rkmpp` (Rockchip MPP) or V4L2 + `v4l2h264enc` as a drop-in replacement for the GStreamer source element
- GPIO / I2C / UART: verify pin availability for TB6612 PWM, gimbal I2C, GPS UART, and INA219 I2C on the 26-pin header
- USB: check if the UZ801 modem enumerates as `usb0` (RNDIS) under the LuckFox kernel

**If evaluation passes — implementation scope:**
- `ARCH=luckfox` cross-compile target (`arm-linux-gnueabihf-g++`)
- `CAMERA=rkmpp` build variant with an MPP-backed `gst_pipeline` implementation
- Hardware bring-up notes in `doc/setup/` covering flashing, USB OTG mode, pin mapping

### Custom ground station (evaluate alternatives or build from scratch)

QGC is a mature and well-supported ground station but was designed around ArduPilot/PX4 workflows. Several rover-specific features require workarounds or are simply not exposed in the QGC UI.

**Known QGC gaps for this rover:**
- Stream selection panel exists but is limited — resolution can be switched via stream list, but bitrate and FPS cannot be changed from the UI (require MAVLink parameter panel)
- No LTE/WiFi connectivity panel — uplink state, signal strength, and session traffic are visible only as STATUSTEXT toasts, not as persistent HUD elements
- `MAV_AUTOPILOT_GENERIC` is not recognised by the mode switcher — HOLD/RETURN modes have no dedicated UI control; switching requires workarounds
- STATUSTEXT log is ephemeral — notifications (GPS fix, stream start, LTE warning) disappear and are not scrollable in the standard view
- No battery cell-level display for custom `BATTERY_STATUS` sources

**Evaluation goals — open-source alternatives:**
- [Mission Planner](https://ardupilot.org/planner/) — Windows-centric, plugin-friendly, but heavy
- [MAVProxy](https://ardupilot.org/mavproxy/) — CLI-based, scriptable; good for testing, poor as a daily driver
- [MAVSDK](https://mavsdk.mavlink.io/) — SDK, not a UI, but useful for building a custom app
- [Cockpit](https://github.com/bluerobotics/cockpit) — web-based, BlueRobotics origin, extensible widget system; most promising candidate for a custom rover panel
- Evaluate widget extensibility, MAVLink passthrough fidelity, and mobile/desktop support

**If evaluation favours a custom implementation:**
- Web app (React / Vue) talking MAVLink over WebSocket (use `mavlink-router` or similar bridge on the Pi)
- Widgets: video stream selector with bitrate/FPS controls (from `VIDEO_STREAM_INFORMATION`), LTE/WiFi status panel (parses `RADIO_STATUS` + session STATUSTEXT), mode switcher for `SET_MODE`, persistent STATUSTEXT log, battery HUD from `BATTERY_STATUS`
- Configurable layout, mobile-friendly (phone as ground station over WireGuard)

### ELRS RX (radio failsafe)

**Blocked:** the Pi Zero 2W's only hardware UART is used by the GPS. Unblocking requires a USB UART adapter for ELRS (420 000 baud, inverted).

**Scope:**
- CRSF frame parser (UART, 420 000 baud, inverted logic via ELRS module)
- Channel mapping: throttle + steering, assignable via parameters
- Priority: QGC active → ignore ELRS; QGC silent > N ms → switch to ELRS
- Failsafe: all channels centred if ELRS also drops
- `RC_CHANNELS` telemetry forwarded to QGC

### INA219 current / voltage sensor (battery monitoring)

**Scope:**
- Read voltage and current from INA219 over I2C (addr `0x40`, same bus as gimbal)
- Forward as MAVLink `BATTERY_STATUS` (msg 147) — QGC displays voltage, current, and estimated charge level in the HUD
- Parameters: shunt resistance, cell count, warn / critical voltage thresholds
- Edge log + QGC `STATUSTEXT` on low-voltage warning and battery-critical events
- Build variant: `make BATTERY=ina219`; stub (`make BATTERY=stub`) keeps behaviour unchanged when sensor is absent

### Autopilot modes

Depends on features 1 and 4 (both done). Not started.

**Scope:**
- **HOLD:** stop motors, hold GPS position; engage on QGC mode switch
- **RETURN:** proportional controller drives the rover back to home (home set on arm)
- State tracked in `RoverState::custom_mode`, reported via `HEARTBEAT` + `CURRENT_MODE`
- QGC does not expose a mode switcher for `MAV_AUTOPILOT_GENERIC`, so mode changes will be triggered via `COMMAND_LONG` / `SET_MODE`

---

## Out of scope

- ModemManager / NetworkManager LTE bring-up — replaced by the UZ801's custom TCP status + link-switch servers
- Structured logging (spdlog) — the simple `logger.hpp` shim is enough

---

## Project structure

```
src/
  main.cpp                 event loop
  drive/                   dead zone, slew, diff mixing
  motor/                   TB6612 driver + stubs
  gimbal/                  I2C gimbal + stub
  gps/                     NMEA provider + stub
  camera/                  discovery + GStreamer pipeline
  lte/                     modem status poll + link switcher
  mavlink/                 senders, param store, command/camera handlers
include/
  config.hpp               compile-time constants (pins, ports, timeouts)
  rover_state.hpp          daemon state
  logger.hpp               timestamped stdout
external/
  mavlink/                 MAVLink C library (submodule)
deploy/
  ground-rover-daemon.service   systemd unit
  modem/                   UZ801 boot scripts (pushed by make deploy-modem)
```
