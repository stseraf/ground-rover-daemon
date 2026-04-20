# 3. Hardware Wiring

All signal wiring between the Pi Zero 2W and rover peripherals. Do this with the Pi powered off. For battery / DC-DC / power rails, see [02-power.md](02-power.md) first.

---

## 3.1 TB6612FNG motor driver

Two DC motors, one H-bridge. Logic is Pi-side; motor power comes from the battery pack directly (see [02-power.md](02-power.md#22-rails)).

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
| `GND` | — | GND (pin 6 or any GND pin) | — |
| `VM` | — | battery + direct | Motor supply — 2s2p 18650 pack, **not** the Pi's 5 V rail |
| `AO1 / AO2` | — | Motor A terminals | To left motor |
| `BO1 / BO2` | — | Motor B terminals | To right motor |

Motor A drives the **left** wheel, Motor B the **right**. If the rover turns the wrong way, swap the two wires to one motor (or flip the polarity in firmware by inverting the `AINx` / `BINx` wiring).

---

## 3.2 GY-GPS6MV2 / u-blox NEO-6M GPS

4-wire connection: power off the Pi's 5 V rail, UART data on GPIO 14/15.

| GPS pin | RPi pin | Notes |
|---|---|---|
| `VCC` | 5 V (pin 2) | Powered from the same XL4015 rail that feeds the Pi |
| `GND` | GND (pin 9) | — |
| `TX` | GPIO 15 / RXD (pin 10) | Module → Pi |
| `RX` | GPIO 14 / TXD (pin 8) | Optional, module doesn't need commands |

The NEO-6M logic is 3.3 V — no level shifter needed even though VCC is 5 V.

---

## 3.3 I2C gimbal (Arduino slave) — optional, not hardware-tested

> **Optional.** The gimbal is designed and the Pi-side code is in place, but the physical build (Arduino + servos + mount) is currently descoped and has not been tested end-to-end. Skip this section if you aren't building the gimbal — the daemon runs fine without it (builds with `GIMBAL=stub`, or with `GIMBAL=i2c` and silently disables if no slave responds on the bus).

The gimbal is an Arduino Nano/Uno acting as an I2C slave controlling two servos. The Pi is the I2C master.

| Signal | RPi pin | Arduino pin |
|---|---|---|
| `SDA` | GPIO 2 (pin 3) | A4 (Uno/Nano) |
| `SCL` | GPIO 3 (pin 5) | A5 (Uno/Nano) |
| `GND` | GND (any GND pin) | GND |

The Arduino has its own 5 V supply. **Do not connect 5 V to the Pi's 3V3 rail** — the Pi's I2C is 3.3 V-tolerant only.

After wiring, confirm the Arduino responds at address `0x10`:

```bash
sudo apt install i2c-tools
i2cdetect -y 1
```

Expected: a device at `10`.

See [features/gimbal.md](../features/gimbal.md) for the Arduino firmware contract (4-byte frame format).

---

## 3.4 MIPI camera (OV5647 / IMX219)

Connect the ribbon cable to the Pi's CSI camera connector (next to HDMI). The Zero 2W uses the small CSI connector — use a **Zero-compatible ribbon** (22-pin, narrow end to Pi, wide end to camera).

- Blue side of ribbon faces the **Ethernet / USB** side on the Pi.
- Blue side of ribbon faces **away from the lens** on the camera board.

Power and data both ride the CSI cable — no separate wiring.

Verify detection once booted:

```bash
libcamera-hello --list-cameras
# Should list at least one camera with sensor modes
```

Supported sensors: OV5647 (Pi Camera v1), IMX219 (v2), IMX477 (HQ). See [features/video.md](../features/video.md) for sensor mode details.

---

## 3.5 UZ801 LTE modem — custom cable required

The UZ801 has a **USB-A female** socket; the Pi Zero 2W has only **micro-USB**. No off-the-shelf cable bridges the two *and* routes 5 V from an external rail — a custom 4-wire cable is needed.

### Cable pinout

| Wire | Modem end (USB-A male) | Pi end |
|---|---|---|
| VBUS (+5 V) | USB-A pin 1 | **Solder to the XL4015 output node** — same rail that feeds Pi header pin 4. Not to a Pi pin. |
| D− | USB-A pin 2 | Solder to the **D−** test pad on the bottom of the Pi, next to the USB micro-B connector |
| D+ | USB-A pin 3 | Solder to the **D+** test pad on the bottom of the Pi |
| GND | USB-A pin 4 | Star-ground with the rest of the system (XL4015 GND is fine) |

### Why 5 V cannot come from the Pi

The modem draws 1–2 A peaks on LTE TX. The Pi's 3V3 regulator (~250 mA) is way short, and the 5 V rail — even back-fed via header pin 4 — routes through thin PCB traces that sag under peaks and reset the Pi. See [02-power.md §2.4](02-power.md#24-why-the-modem-does-not-power-off-a-pi-pin).

### Test-pad location

The two test pads are on the **underside of the Pi Zero 2W**, adjacent to the micro-USB data connector (the one labelled `USB`, not `PWR`). Under magnification you'll see them as the two pads directly in line with the D+ / D− pins of the connector.

Tin the pads first, tin the wires, then reflow briefly — the pads lift if over-soldered.

### Verifying the modem is seen

Once the custom cable is in place and powered:

```bash
ip link show usb0 2>/dev/null && echo "modem detected"
```

Full modem setup (ADB, firmware scripts, static IP) is covered in [05-lte-modem.md](05-lte-modem.md).

---

## 3.6 Pi power input

When running on-rover, power the Pi from the XL4015 through the GPIO header:

- **Pin 4** ← +5.1 V (XL4015 output)
- **Pin 6** ← GND (star ground)

Do **not** simultaneously connect power to the micro-USB `PWR` port — pick one input per session.

On the bench (no motors), you can instead plug a 5 V USB adapter into the Pi's micro-USB `PWR` port. The Pi back-feeds its GPIO 5 V rail, which in turn powers the GPS and TB6612 logic; the modem's VBUS test-pad wire can temporarily land on the Pi's 5 V rail too. See [02-power.md §2.5](02-power.md#25-bench-debug-mode-no-motors).

---

## Next

→ [04-build-deploy.md](04-build-deploy.md) — build the daemon and deploy to the Pi.
