# Ground Rover Daemon

A field-deployable differential-drive rover built on a Raspberry Pi Zero 2W and controlled from [QGroundControl](https://qgroundcontrol.com/) over LTE. The daemon is the MAVLink bridge between QGC and the hardware: joystick in, motor PWM + gimbal servos out, GPS + radio telemetry back.

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
  │   ├─ I2C gimbal (Arduino)        │
  │   ├─ NMEA GPS (NEO-6M)           │
  │   └─ GStreamer H.264 (MIPI cam)  │
  └──────────────────────────────────┘
```

---

## Getting started

Fresh Pi to ready-to-drive rover in six steps. Start from [doc/setup/01-raspberry-pi.md](doc/setup/01-raspberry-pi.md) and work through in order.

| # | Step | What it covers |
|---|---|---|
| 1 | [Raspberry Pi OS setup](doc/setup/01-raspberry-pi.md) | Flash Pi OS, edit `config.txt` (PWM, UART, I2C, GPU memory), permissions |
| 2 | [Hardware wiring](doc/setup/02-wiring.md) | TB6612, GPS, I2C gimbal, MIPI camera, LTE modem, power |
| 3 | [Build & deploy the daemon](doc/setup/03-build-deploy.md) | Cross-compile for aarch64, `make deploy`, systemd service |
| 4 | [UZ801 LTE modem setup](doc/setup/04-lte-modem.md) | ADB, `make deploy-modem`, optional home WiFi credentials |
| 5 | [WireGuard VPN](doc/setup/05-wireguard.md) | Tunnel to the home router so QGC can reach the rover on any uplink |
| 6 | [QGroundControl connection](doc/setup/06-qgroundcontrol.md) | First connection, joystick, video stream, runtime parameters |

**Tested on:** Raspberry Pi OS (Debian GNU/Linux 13 "Trixie") 13.4, 64-bit, kernel 6.12.75. Newer releases should work.

---

## Features

| # | Feature | Status | Doc |
|---|---|---|---|
| 1 | Differential drive (TB6612) + I2C gimbal | ✅ done | [drive.md](doc/features/drive.md), [gimbal.md](doc/features/gimbal.md) |
| 2 | MAVLink parameter protocol + persistence | ✅ done | [parameters.md](doc/reference/parameters.md) |
| 3 | ELRS RX (radio failsafe) | ⏸ blocked (UART in use) | see *Improvement plan* |
| 4 | GPS module + MAVLink telemetry | ✅ done | [gps.md](doc/features/gps.md) |
| 5 | LTE link monitoring + uplink switching | ✅ done | [lte-uplink.md](doc/features/lte-uplink.md) |
| 6 | Video streaming (GStreamer + H.264 RTP) | ✅ done | [video.md](doc/features/video.md) |
| 7 | Autopilot modes (HOLD, RETURN) | 🗓 planned | see *Improvement plan* |

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

### ELRS RX (radio failsafe)

**Blocked:** the Pi Zero 2W's only hardware UART is used by the GPS. Unblocking requires a USB UART adapter for ELRS (420 000 baud, inverted).

**Scope:**
- CRSF frame parser (UART, 420 000 baud, inverted logic via ELRS module)
- Channel mapping: throttle + steering, assignable via parameters
- Priority: QGC active → ignore ELRS; QGC silent > N ms → switch to ELRS
- Failsafe: all channels centred if ELRS also drops
- `RC_CHANNELS` telemetry forwarded to QGC

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
