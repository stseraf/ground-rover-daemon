# Ground Rover Daemon

A MAVLink UDP daemon that acts as a simulated ground rover autopilot, allowing [QGroundControl](https://qgroundcontrol.com/) to connect and send joystick commands via `MANUAL_CONTROL` messages.

It implements enough of the MAVLink protocol for QGC to recognise the vehicle, arm/disarm it, switch flight modes, and stream joystick input — without requiring real hardware.

## How it works

- Binds a UDP socket on port **14550** (the standard QGC port)
- Sends periodic `HEARTBEAT` and `SYS_STATUS` telemetry at 1 Hz
- Responds to `COMMAND_LONG`, `SET_MODE`, `PARAM_REQUEST_LIST`, and `MISSION_REQUEST_LIST` messages
- Logs all traffic to stdout with a monotonic timestamp

## Requirements

- C++17 compiler (GCC or Clang)
- `make`

## Building

Clone with submodules (MAVLink C library is included as a submodule):

```sh
git clone --recurse-submodules <repo-url>
```

Or if already cloned:

```sh
git submodule update --init --recursive
```

Build:

```sh
make
```

The binary is placed at `build/ground_rover_daemon`.

## Running

```sh
./build/ground_rover_daemon
```

Then open QGroundControl — it will auto-connect on UDP port 14550.

## Project structure

```
src/                    # compiled sources
  main.cpp              # event loop (~90 lines)
inc/                    # project headers
  config.hpp            # compile-time constants
  udp_socket.hpp        # RAII UDP socket wrapper
  rover_state.hpp       # mutable daemon state struct
  logger.hpp            # timestamped stdout logging
  mav_sender.hpp        # MAVLink TX methods
  command_handlers.hpp  # incoming command dispatch
external/
  mavlink/              # MAVLink C library (submodule)
build/                  # build output (gitignored)
```

## Configuration

All tunable constants are in `inc/config.hpp`:

| Constant | Default | Description |
|---|---|---|
| `MAV_SYS_ID` | `1` | MAVLink system ID |
| `MAV_COMP_ID` | `1` | MAVLink component ID |
| `UDP_BIND_PORT` | `14550` | UDP port to bind |
| `HEARTBEAT_INTERVAL_US` | `1 000 000` | Heartbeat period (µs) |
| `LOOP_SLEEP_US` | `1 000` | Main loop sleep (µs) |
