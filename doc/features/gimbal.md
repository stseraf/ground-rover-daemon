# I2C Gimbal

A 2-axis pan/tilt gimbal driven by an Arduino I2C slave. The Pi sends 4-byte frames over `/dev/i2c-1` at address `0x10`; the Arduino maps them to servo angles.

Build variant: `make GIMBAL=i2c`.

Implementation: [`src/gimbal/i2c_gimbal_controller.cpp`](../../src/gimbal/i2c_gimbal_controller.cpp).

---

## Signal flow

```
QGC joystick
  ↓ MANUAL_CONTROL.x (pan) / .r (tilt)
I2cGimbalController
  ↓ 4-byte frame: [pan_hi][pan_lo][tilt_hi][tilt_lo]  (int16 BE, ±1000)
Arduino I2C slave @ 0x10
  ↓ Servo.write()
pan servo + tilt servo
```

The daemon writes one 4-byte frame per update cycle. If the I2C bus is unavailable the daemon starts normally with the gimbal silently disabled.

---

## Compile-time constants

| Constant | Value | Override |
|---|---|---|
| `Config::Gimbal::I2C_BUS` | `/dev/i2c-1` | `make CXXFLAGS_EXTRA=-DGIMBAL_I2C_BUS='"/dev/i2c-0"'` |
| `Config::Gimbal::I2C_ADDR` | `0x10` | (edit `config.hpp`) |
| `Config::Gimbal::DEAD_ZONE` | 30 | (edit `config.hpp`) |

---

## Arduino firmware contract

The Arduino must implement an I2C slave that:

- Listens at address `0x10` (matches `Config::Gimbal::I2C_ADDR`).
- Receives 4-byte frames: `[pan_hi][pan_lo][tilt_hi][tilt_lo]`.
- Values are `int16_t`, big-endian, range `[−1000, +1000]`.
- Maps to servo angle: `angle = (value + 1000) * 180 / 2000` (0° – 180°).

### Critical: do not call `Servo.write()` inside `Wire.onReceive()`

`Wire.onReceive()` runs in interrupt context. `Servo.write()` uses timer interrupts internally and can deadlock or produce glitches. Always buffer the 4 bytes in the ISR and drive the servos from `loop()`:

```cpp
volatile int16_t pan_target  = 0;
volatile int16_t tilt_target = 0;
volatile bool    new_frame   = false;

void onReceive(int n) {
  if (n < 4) return;
  uint8_t buf[4];
  for (int i = 0; i < 4 && Wire.available(); ++i) buf[i] = Wire.read();
  pan_target  = (int16_t)((buf[0] << 8) | buf[1]);
  tilt_target = (int16_t)((buf[2] << 8) | buf[3]);
  new_frame   = true;
}

void loop() {
  if (new_frame) {
    new_frame = false;
    pan_servo.write(  (pan_target  + 1000) * 180 / 2000);
    tilt_servo.write( (tilt_target + 1000) * 180 / 2000);
  }
}
```

---

## Verification

On the Pi:

```bash
sudo apt install i2c-tools
i2cdetect -y 1
# Expected: device at 10
```

If the Arduino doesn't show up, check:
1. Wiring — SDA on GPIO 2 (pin 3), SCL on GPIO 3 (pin 5), GND common.
2. I2C enabled — `dtparam=i2c_arm=on` in `/boot/firmware/config.txt`.
3. Pi user is in the `i2c` group.
4. Arduino sketch calls `Wire.begin(0x10)` (slave mode) and has `Wire.onReceive()` registered.

---

## Wiring

See [../setup/02-wiring.md §2.3](../setup/02-wiring.md#23-i2c-gimbal-arduino-slave).
