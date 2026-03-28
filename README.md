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
- For `ARCH=rpi`: cross-compiler (RPi Zero 2W runs 64-bit aarch64 OS):
  ```sh
  sudo apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu
  ```

## Building

Clone with submodules (MAVLink C library is included as a submodule):

```sh
git clone --recurse-submodules <repo-url>
```

Or if already cloned:

```sh
git submodule update --init --recursive
```

Three independent Makefile variables control the build:

| Variable | Values | Default | Description |
|---|---|---|---|
| `ARCH` | `host`, `rpi` | `host` | Target architecture |
| `DRIVER` | `stub`, `tb6612` | `stub` | Motor driver backend |
| `GIMBAL` | `stub`, `i2c` | `stub` | Gimbal controller backend |

| Command | Output |
|---|---|
| `make` | x86, stub motors, stub gimbal |
| `make ARCH=rpi` | aarch64, stub motors, stub gimbal |
| `make ARCH=rpi DRIVER=tb6612` | aarch64, TB6612 motors, stub gimbal |
| `make ARCH=rpi DRIVER=tb6612 GIMBAL=i2c` | aarch64, TB6612 motors, I2C gimbal |

The binary is placed at `build/ground_rover_daemon`.

GPIO and PWM are accessed via Linux sysfs (`/sys/class/gpio`, `/sys/class/pwm`) — no external libraries required.

## Deploying to RPi

The `deploy` target builds for RPi and copies the binary over SSH in one step:

```sh
make deploy
```

Default: `RPI=pi@pi-rover.lan`. Override as needed:

```sh
make deploy RPI=pi@192.168.1.x
```

Set a permanent default in your shell:
```sh
echo 'export RPI=pi@pi-rover.lan' >> ~/.bashrc
source ~/.bashrc
```

## RPi hardware setup (DRIVER=tb6612)

### 1. Enable hardware PWM

Add to `/boot/firmware/config.txt` (under `[all]`):

```
dtoverlay=pwm-2chan
```

Also disable audio to avoid GPIO18 conflict (PWM0 shares GPIO18 with audio):

```
dtparam=audio=off
```

Reboot, then verify:
```sh
ls /sys/class/pwm/pwmchip0/   # should exist
cat /sys/class/pwm/pwmchip0/npwm  # should print 2
```

### 2. GPIO pin wiring

Pin numbers are configured in `inc/config.hpp` (`Config::Tb6612` namespace). The current values are placeholders — fill in actual BCM pin numbers to match your wiring before running.

### 3. Permissions

The daemon requires access to `/dev/gpiochip0` and `/sys/class/pwm/`. Run as root (`sudo`) or configure udev rules for the `gpio` group.

---

## RPi hardware setup (GIMBAL=i2c)

The gimbal controller sends 2-axis commands over I2C to an Arduino acting as an I2C slave.
If the I2C bus is unavailable the daemon starts normally with the gimbal silently disabled.

### 1. Enable I2C

Add to `/boot/firmware/config.txt` (under `[all]`):

```
dtparam=i2c_arm=on
```

Reboot, then verify the bus exists:

```sh
ls /dev/i2c-*   # should show /dev/i2c-1
```

### 2. Verify Arduino is visible

With the Arduino connected and running the gimbal firmware:

```sh
sudo apt install i2c-tools   # if not already installed
i2cdetect -y 1               # scan I2C bus 1
```

You should see a device at address `0x10` (configured in `Config::Gimbal::I2C_ADDR`).

### 3. Permissions

Add your user to the `i2c` group to avoid running as root:

```sh
sudo usermod -aG i2c $USER
# log out and back in, or: newgrp i2c
```

### 4. Arduino firmware contract

The Arduino must implement an I2C slave that:

- Listens at address `0x10` (match `Config::Gimbal::I2C_ADDR`)
- Receives 4-byte frames: `[pan_hi][pan_lo][tilt_hi][tilt_lo]`
- Values are `int16_t` big-endian in range `[-1000, +1000]`
- Maps to servo angle: `angle = (value + 1000) * 180 / 2000` (0°–180°)

> **Important:** `Wire.onReceive()` runs in interrupt context on Arduino. Buffer the
> 4 bytes in the ISR and call `Servo.write()` from `loop()` only — not from the callback.

## Running

```sh
sudo ./ground_rover_daemon
```

Then open QGroundControl — it will auto-connect on UDP port 14550.

## Project structure

```
src/                    # compiled sources
  main.cpp              # event loop
inc/                    # project headers
  config.hpp            # compile-time constants (pins, PWM paths, MAVLink IDs)
  motor_driver.hpp      # IMotorDriver interface
  tb6612_driver.hpp     # TB6612 H-bridge driver (libgpiod 2.x + sysfs PWM)
  uart_motor_driver.hpp # no-op stub driver (host builds)
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
| `Config::Tb6612::AIN1_PIN` | `0` (placeholder) | BCM pin for Motor A direction bit 1 |
| `Config::Tb6612::AIN2_PIN` | `0` (placeholder) | BCM pin for Motor A direction bit 2 |
| `Config::Tb6612::BIN1_PIN` | `0` (placeholder) | BCM pin for Motor B direction bit 1 |
| `Config::Tb6612::BIN2_PIN` | `0` (placeholder) | BCM pin for Motor B direction bit 2 |
| `Config::Tb6612::STBY_PIN` | `0` (placeholder) | BCM pin for TB6612 standby |
| `Config::Tb6612::PWM_PERIOD_NS` | `25000` | PWM period in ns (40 kHz) |
