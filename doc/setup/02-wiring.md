# 2. Hardware Wiring

All wiring between the Pi Zero 2W and rover peripherals. Do this with the Pi powered off.

---

## 2.1 TB6612FNG motor driver

Two DC motors, one H-bridge.

| Signal | BCM pin | Physical header pin | Purpose |
|---|---|---|---|
| `PWMA` | GPIO 12 | 32 | Motor A speed (PWM channel 0) |
| `AIN1` | GPIO 20 | 38 | Motor A direction bit 1 |
| `AIN2` | GPIO 16 | 36 | Motor A direction bit 2 |
| `PWMB` | GPIO 13 | 33 | Motor B speed (PWM channel 1) |
| `BIN1` | GPIO 26 | 37 | Motor B direction bit 1 |
| `BIN2` | GPIO 19 | 35 | Motor B direction bit 2 |
| `STBY` | GPIO 6 | 31 | Standby (active-HIGH enables driver) |
| `VCC` | — | 3V3 (pin 1) | Logic supply |
| `GND` | — | GND (pin 6) | — |
| `VM` | — | battery + | Motor supply (separate from logic — do **not** use 5 V rail) |

Motor A drives the **left** wheel, Motor B the **right**. If the rover turns the wrong way, swap the two wires to one motor (or set `DRIVE_TRIM` to 0 and flip the motor's polarity in firmware by inverting `AINx`/`BINx` wiring).

---

## 2.2 GY-GPS6MV2 / u-blox NEO-6M GPS

4-wire UART connection.

| GPS pin | RPi pin | Notes |
|---|---|---|
| `VCC` | 3V3 (pin 1) *or* 5V (pin 2) | Module accepts both |
| `GND` | GND (pin 6) | — |
| `TX` | GPIO 15 / RXD (pin 10) | Module → Pi |
| `RX` | GPIO 14 / TXD (pin 8) | Optional, module doesn't need commands |

The NEO-6M logic is 3.3 V — no level shifter needed.

---

## 2.3 I2C gimbal (Arduino slave) — optional, not hardware-tested

> **Optional.** The gimbal is designed and the Pi-side code is in place, but the physical build (Arduino + servos + mount) is currently descoped and has not been tested end-to-end. Skip this section if you aren't building the gimbal — the daemon runs fine without it (builds with `GIMBAL=stub`, or with `GIMBAL=i2c` and silently disables if no slave responds on the bus).

The gimbal is an Arduino Nano/Uno acting as an I2C slave controlling two servos. The Pi is the I2C master.

| Signal | RPi pin | Arduino pin |
|---|---|---|
| `SDA` | GPIO 2 (pin 3) | A4 (Uno/Nano) |
| `SCL` | GPIO 3 (pin 5) | A5 (Uno/Nano) |
| `GND` | GND (pin 9) | GND |

The Arduino has its own 5 V supply. **Do not connect 5 V to the Pi's 3V3 rail** — the Pi's I2C is 3.3 V-tolerant only.

After wiring, confirm the Arduino responds at address `0x10`:

```bash
sudo apt install i2c-tools
i2cdetect -y 1
```

Expected: a device at `10`.

See [features/gimbal.md](../features/gimbal.md) for the Arduino firmware contract (4-byte frame format).

---

## 2.4 MIPI camera (OV5647 / IMX219)

Connect the ribbon cable to the Pi's CSI camera connector (next to HDMI). The Zero 2W uses the small CSI connector — use a **Zero-compatible ribbon** (22-pin, narrow end to Pi, wide end to camera).

- Blue side of ribbon faces the **Ethernet / USB** side on the Pi.
- Blue side of ribbon faces **away from the lens** on the camera board.

Verify detection once booted:

```bash
libcamera-hello --list-cameras
# Should list at least one camera with sensor modes
```

Supported sensors: OV5647 (Pi Camera v1), IMX219 (v2), IMX477 (HQ). See [features/video.md](../features/video.md) for sensor mode details.

---

## 2.5 UZ801 LTE modem

USB, no GPIO. Plug the dongle into the Pi's **USB data** port (the one labelled `USB`, not `PWR`). Use a USB OTG adapter if your dongle is full-size USB-A.

The modem exposes itself as a USB composite device (RNDIS + serial + ADB). Once plugged in, the Pi should see `usb0` appear — but it needs the static-IP configuration from the next step to actually use it.

For now:

```bash
ip link show usb0 2>/dev/null && echo "modem detected"
```

Full modem setup is covered in [04-lte-modem.md](04-lte-modem.md).

---

## 2.6 Power

The Pi Zero 2W's 5 V rail cannot supply both the motors (high current) and the modem (1–2 A peak on LTE TX). Use a separate supply for:

- **Motors / TB6612 VM:** rover battery (e.g., 2S LiPo + regulator to match motor rating)
- **Pi + modem:** a 5 V / ≥ 2.5 A buck regulator from the same battery

Share a common GND across all supplies.

---

## Next

→ [03-build-deploy.md](03-build-deploy.md) — build the daemon and deploy to the Pi.
