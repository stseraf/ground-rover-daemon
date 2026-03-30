# Ground Rover Daemon

A MAVLink UDP daemon that connects [QGroundControl](https://qgroundcontrol.com/) to a differential-drive ground rover. QGC sends joystick input via `MANUAL_CONTROL` messages; the daemon drives a TB6612FNG H-bridge and optionally a 2-axis I2C gimbal.

## How it works

- Binds a UDP socket on port **14550** (the standard QGC port)
- Sends periodic `HEARTBEAT` and `SYS_STATUS` telemetry at 1 Hz
- Handles arm/disarm, mode switching, joystick input, and parameter read/write
- Applies dead zone, slew-rate limiting, and motor trim to joystick axes before driving motors
- Stops motors and slews to zero if no `MANUAL_CONTROL` arrives within the configured timeout (failsafe)
- Reads a GPS module (GY-GPS6MV2 / u-blox NEO-6M) over UART and forwards `GPS_RAW_INT` and `GLOBAL_POSITION_INT` telemetry to QGC at 1 Hz
- Logs all traffic to stdout with a monotonic timestamp

## Requirements

- C++17 compiler (GCC or Clang)
- `make`
- For `ARCH=rpi`: cross-compiler (RPi Zero 2W runs 64-bit aarch64):
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

Four independent Makefile variables control the build:

| Variable | Values | Default | Description |
|---|---|---|---|
| `ARCH` | `host`, `rpi` | `host` | Target architecture |
| `DRIVER` | `stub`, `tb6612` | `stub` | Motor driver backend |
| `GIMBAL` | `stub`, `i2c` | `stub` | Gimbal controller backend |
| `GPS` | `stub`, `nmea` | `stub` | GPS provider backend |

| Command | Output |
|---|---|
| `make` | x86, stub motors, stub gimbal, stub GPS |
| `make ARCH=rpi` | aarch64, stub motors, stub gimbal, stub GPS |
| `make ARCH=rpi DRIVER=tb6612` | aarch64, TB6612 motors, stub gimbal, stub GPS |
| `make ARCH=rpi DRIVER=tb6612 GIMBAL=i2c` | aarch64, TB6612 motors, I2C gimbal, stub GPS |
| `make ARCH=rpi DRIVER=tb6612 GIMBAL=i2c GPS=nmea` | aarch64, TB6612 motors, I2C gimbal, NMEA GPS |

The binary is placed at `build/ground_rover_daemon`.

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

## Running

Run the daemon from its own directory so the `params` file is written alongside the binary:

```sh
cd /path/to/binary
sudo ./ground_rover_daemon
```

Then open QGroundControl — it will auto-connect on UDP port 14550.

## Runtime parameters

The daemon exposes 4 tunable parameters via the MAVLink parameter protocol. QGC shows them in the **Parameters** panel; values are saved to a `params` file next to the binary and survive restarts.

| # | Name | Default | Description |
|---|---|---|---|
| 0 | `DRIVE_DEAD_ZONE` | 30 | Joystick dead zone (axis units, 0–1000) |
| 1 | `DRIVE_SLEW_MS` | 500 | Slew-rate ramp time from stop to full speed (ms) |
| 2 | `DRIVE_TRIM` | 0 | Motor balance offset — positive shifts power to left motor, negative to right |
| 3 | `CTRL_TIMEOUT_MS` | 500 | Failsafe: stop motors after this many ms with no joystick input |

`params` file format (plain text, editable by hand):
```
DRIVE_DEAD_ZONE=30
DRIVE_SLEW_MS=500
DRIVE_TRIM=0
CTRL_TIMEOUT_MS=500
```

---

## RPi hardware setup (GPS=nmea)

The GPS provider reads NMEA 0183 sentences from a GY-GPS6MV2 / u-blox NEO-6M module over the hardware UART at 9600 baud. The RPi Zero 2W has one hardware UART (`ttyAMA0`) which is shared with Bluetooth by default and also used as a serial console — both must be freed before the GPS module can use it.

### 1. Free the hardware UART from Bluetooth

Add to `/boot/firmware/config.txt` (under `[all]`):

```
dtoverlay=disable-bt
```

This routes `ttyAMA0` directly to the GPIO header pins (GPIO14/TX, GPIO15/RX) instead of the Bluetooth controller. Reboot after this change.

### 2. Disable the serial console

Open `raspi-config`:

```sh
sudo raspi-config
```

Navigate to **Interface Options → Serial Port**, then:

- **"Would you like a login shell to be accessible over the serial port?"** → **No**
- **"Would you like the serial port hardware to be enabled?"** → **Yes**

This removes `console=serial0,115200` from `/boot/firmware/cmdline.txt` while keeping the UART hardware enabled.

### 3. Verify

After rebooting:

```sh
ls -la /dev/serial0
# should show: /dev/serial0 -> ttyAMA0
```

If it shows `ttyS0` instead, the disable-bt overlay didn't apply — double-check `config.txt` and reboot.

### 4. Wiring (GY-GPS6MV2)

| GPS module pin | RPi pin |
|---|---|
| VCC | 3.3 V (pin 1) or 5 V (pin 2) — module accepts both |
| GND | GND (pin 6) |
| TX | GPIO 15 / RXD (pin 10) |
| RX | GPIO 14 / TXD (pin 8) — optional, module needs no commands |

> **Note:** The NEO-6M module's TX output is 3.3 V logic; connect directly to the RPi RXD pin. The RPi TXD is also 3.3 V so no level shifter is needed.

---

## RPi hardware setup (DRIVER=tb6612)

### 1. Enable hardware PWM

Add to `/boot/firmware/config.txt` (under `[all]`):

```
dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4
dtparam=audio=off
```

`audio=off` avoids a GPIO18 conflict. Reboot, then verify:
```sh
ls /sys/class/pwm/pwmchip0/   # should exist
cat /sys/class/pwm/pwmchip0/npwm  # should print 2
```

### 2. GPIO pin wiring

| Signal | BCM pin |
|--------|---------|
| AIN1 (Motor A / left, dir bit 1) | GPIO 20 |
| AIN2 (Motor A / left, dir bit 2) | GPIO 16 |
| BIN1 (Motor B / right, dir bit 1) | GPIO 26 |
| BIN2 (Motor B / right, dir bit 2) | GPIO 19 |
| STBY (standby, active-HIGH) | GPIO 6 |
| PWMA (Motor A speed) | GPIO 12 (PWM0) |
| PWMB (Motor B speed) | GPIO 13 (PWM1) |

### 3. Permissions

The daemon requires access to `/dev/gpiochip0` and `/sys/class/pwm/`. Run as root (`sudo`) or configure udev rules for the `gpio` group.

---

## RPi hardware setup (GIMBAL=i2c)

The gimbal controller sends 2-axis pan/tilt commands over I2C to an Arduino acting as an I2C slave.
If the I2C bus is unavailable the daemon starts normally with the gimbal silently disabled.

### 1. Enable I2C

Add to `/boot/firmware/config.txt` (under `[all]`):

```
dtparam=i2c_arm=on
```

Reboot, then verify:

```sh
ls /dev/i2c-*   # should show /dev/i2c-1
```

### 2. Verify Arduino is visible

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

---

## Project structure

```
src/
  main.cpp                       # main event loop
  drive/
    diff_drive.hpp               # dead zone, slew, differential mixing
  motor/
    uart_motor_driver.hpp/.cpp   # no-op stub (host builds)
    tb6612_driver.hpp/.cpp       # TB6612 H-bridge (gpio_v2 ioctl + sysfs PWM)
  gimbal/
    stub_gimbal_controller.hpp   # no-op stub
    i2c_gimbal_controller.hpp/.cpp  # I2C 2-axis gimbal
  gps/
    gps_provider.hpp             # IGpsProvider interface + GpsFix struct
    stub_gps_provider.hpp        # no-op stub (host/non-GPS builds)
    nmea_gps_provider.hpp/.cpp   # NMEA 0183 UART parser (GY-GPS6MV2 / NEO-6M)
  mavlink/
    mav_sender.hpp/.cpp          # MAVLink TX helpers
    param_store.hpp/.cpp         # runtime parameter store (file-backed)
    command_handlers.hpp/.cpp    # COMMAND_LONG dispatch
    camera_handlers.hpp/.cpp     # camera stub responses
include/
  config.hpp                     # compile-time constants (pins, PWM, MAVLink IDs, GPS UART)
  gps_fix.hpp                    # GpsFix data struct
  udp_socket.hpp                 # RAII UDP socket wrapper
  rover_state.hpp                # mutable daemon state struct
  logger.hpp                     # timestamped stdout logging
external/
  mavlink/                       # MAVLink C library (submodule)
build/                           # build output (gitignored)
```

## Compile-time configuration

All hardware constants are in `include/config.hpp`:

| Constant | Default | Description |
|---|---|---|
| `MAV_SYS_ID` | `1` | MAVLink system ID |
| `MAV_COMP_ID` | `1` | MAVLink component ID |
| `UDP_BIND_PORT` | `14550` | UDP port to bind |
| `HEARTBEAT_INTERVAL_US` | `1 000 000` | Heartbeat period (µs) |
| `DRIVE_DEAD_ZONE` | `30` | Default dead zone (overridable at runtime) |
| `DRIVE_SLEW_TIME_MS` | `500` | Default slew time (overridable at runtime) |
| `MC_TIMEOUT_US` | `500 000` | Default failsafe timeout (overridable at runtime) |
| `Tb6612::PWM_PERIOD_NS` | `20 000 000` | PWM period — 50 Hz |
| `Gimbal::I2C_BUS` | `/dev/i2c-1` | I2C bus path (override at compile time) |
| `Gimbal::I2C_ADDR` | `0x10` | Arduino I2C slave address |
| `Gps::UART_DEV` | `/dev/serial0` | GPS UART device path |
| `Gps::BAUD_RATE` | `9600` | GPS UART baud rate (NEO-6M default) |
