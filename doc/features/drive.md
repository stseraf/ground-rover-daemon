# Differential Drive

Two independently-controlled DC motors with hardware PWM speed control and direction bits on a TB6612FNG H-bridge.

---

## Signal flow

```
QGC joystick
  ↓ MAVLink MANUAL_CONTROL (x=forward/back, r=yaw)
DiffDrive
  ↓ dead zone → slew limit → differential mix → trim
IMotorDriver (TB6612 or stub)
  ↓ sysfs PWM + /dev/gpiochip0 direction bits
Motors
```

`src/drive/diff_drive.hpp` handles dead zone, slew rate limiting, and the left/right mix. `src/motor/tb6612_driver.cpp` writes to `/sys/class/pwm/pwmchip0/pwm[0,1]/duty_cycle` for speed and `/dev/gpiochip0` for `AINx`/`BINx` direction bits.

---

## Runtime parameters

| Param | Default | Range | Effect |
|---|---|---|---|
| `DRIVE_DEAD_ZONE` | 30 | 0–1000 | Joystick input below ±N is treated as zero. Prevents drift from stick slop. |
| `DRIVE_SLEW_MS` | 500 | 0–5000 | Time to ramp from 0 → full power. Higher = smoother start, lower = snappier response. |
| `DRIVE_TRIM` | 0 | −500 to +500 | Per-motor power bias. Positive shifts more power to the **left** motor (rover tends to veer right → increase trim). |
| `CTRL_TIMEOUT_MS` | 500 | 50–5000 | Failsafe: if no `MANUAL_CONTROL` received within this time, motors slew to zero. |

All four are tunable live from QGC's Parameters panel and persisted to `params`.

---

## Failsafe behavior

If no `MANUAL_CONTROL` arrives within `CTRL_TIMEOUT_MS`:

1. The target velocity is set to zero.
2. The slew limiter ramps the current command down over `DRIVE_SLEW_MS`.
3. `STBY` is driven LOW when both motors reach zero (saves power, silences the driver).

When joystick input resumes, the rover ramps back up using the same slew — you won't get an instant full-throttle jerk on reconnect.

---

## Differential mixing

Given forward axis `X` and yaw axis `R` (both −1000…+1000 after dead zone and slew):

```
left_raw  = X + R
right_raw = X − R
left      = clamp(left_raw  + TRIM, ±1000)
right     = clamp(right_raw − TRIM, ±1000)
```

Each wheel's sign determines direction bits; magnitude is scaled to `duty_cycle` against a 20 ms PWM period (50 Hz).

When driving backward (`X < 0`), the turn direction is inverted internally so the joystick's "turn right" always pivots the rover to the right from the operator's frame of reference.

---

## Motor driver backends

Selected at build time via `make DRIVER=…`.

| Backend | Use |
|---|---|
| `stub` (default) | No-op. Use for host builds and protocol testing without hardware. |
| `tb6612` | TB6612FNG H-bridge on RPi via `/sys/class/pwm/` + `/dev/gpiochip0`. |

Pin assignments and PWM period are compile-time constants in `include/config.hpp` under `Config::Tb6612`. Wiring is in [../setup/02-wiring.md](../setup/02-wiring.md).

---

## Testing without a vehicle

The stub build logs motor commands to stdout at every update. Useful for verifying protocol-level behavior:

```bash
make                                 # ARCH=host DRIVER=stub
./build/ground_rover_daemon
# then connect QGC and wiggle the joystick
```

Output includes clamped left/right values, slew state, and failsafe events.
