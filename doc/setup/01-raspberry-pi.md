# 1. Raspberry Pi OS Setup

Prepare a fresh Raspberry Pi Zero 2W for the rover daemon. This configures the kernel interfaces the daemon needs (UART for GPS, PWM for motors, I2C for the gimbal, extra GPU memory for the HW H.264 encoder).

> **Tested on:** Raspberry Pi OS (Debian GNU/Linux 13 "Trixie"), version 13.4, 64-bit, kernel 6.12.75. Newer releases should work identically.

---

## 1.1 Flash the OS

Use **Raspberry Pi Imager** to write *Raspberry Pi OS (64-bit) Lite* (no desktop) onto an SD card.

In the imager's advanced options set:

- **Hostname:** `pi-rover`
- **Username / password:** `pi` / your choice
- **Enable SSH:** yes, password authentication (or SSH key)
- **WiFi credentials:** your home WiFi (only needed for the initial setup — the Pi's WiFi is disabled afterwards in favour of the LTE modem)
- **Locale:** your timezone

Boot the Pi, connect via SSH, and update the system:

```bash
ssh pi@pi-rover.lan
sudo apt update && sudo apt full-upgrade -y
sudo reboot
```

---

## 1.2 Edit `/boot/firmware/config.txt`

All the rover's hardware needs a single-file edit. Open it once:

```bash
sudo nano /boot/firmware/config.txt
```

Under the `[all]` section, add:

```
# --- Ground rover daemon ---
# Hardware PWM on GPIO12/13 for TB6612 motor driver
dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4
dtparam=audio=off

# Free hardware UART for NEO-6M GPS (disable Bluetooth, no-op if not present)
dtoverlay=disable-bt

# I2C for gimbal Arduino
dtparam=i2c_arm=on

# Extra GPU memory for HW H.264 encoder (bcm2835-codec)
gpu_mem=128
```

Save (Ctrl-O, Enter) and exit (Ctrl-X). The settings take effect on the next reboot — don't reboot yet, the serial console disable below is also required.

**Notes:**
- `audio=off` avoids a GPIO18 conflict that would block PWM channel 1.
- `disable-bt` routes `ttyAMA0` to GPIO14/15 (no-op on a Zero 2W since Bluetooth is rarely used, but required for the UART to be available).
- `gpu_mem=128` is mandatory — default 64 MB causes the HW encoder to fail at 1296×972 and above. It leaves 288 MB for ARM, enough for headless streaming.

---

## 1.3 Disable the serial console

The GPS needs exclusive use of the UART; the kernel must not use it for a login shell.

```bash
sudo raspi-config
```

Navigate: **Interface Options → Serial Port** → answer:
- *"Would you like a login shell to be accessible over the serial port?"* → **No**
- *"Would you like the serial port hardware to be enabled?"* → **Yes**

Exit (**Finish**), let it reboot.

After reboot, verify the UART points at the PL011:

```bash
ls -l /dev/serial0
# /dev/serial0 -> ttyAMA0    ← correct
# /dev/serial0 -> ttyS0      ← Bluetooth not disabled, re-check config.txt
```

---

## 1.4 Verify PWM and I2C

```bash
# PWM
ls /sys/class/pwm/pwmchip0/
cat /sys/class/pwm/pwmchip0/npwm
# expected: "2"

# I2C
ls /dev/i2c-*
# expected: /dev/i2c-1
```

---

## 1.5 System permissions and groups

Add the `pi` user to the `gpio`, `i2c`, and `dialout` groups (for `/dev/serial0`):

```bash
sudo usermod -aG gpio,i2c,dialout pi
```

The rover daemon needs passwordless `sudo` (for `wg-quick` on uplink switches). Check if it's already enabled:

```bash
sudo cat /etc/sudoers.d/010_pi-nopasswd 2>/dev/null || \
  echo 'pi ALL=(ALL) NOPASSWD: ALL' | sudo tee /etc/sudoers.d/010_pi-nopasswd
sudo chmod 440 /etc/sudoers.d/010_pi-nopasswd
```

Log out and back in so group membership takes effect:

```bash
exit
ssh pi@pi-rover.lan
groups   # should include: pi gpio i2c dialout ...
```

---

## 1.6 Disable on-board WiFi (recommended)

Once the LTE modem is set up, the Pi's BCM43438 WiFi is no longer used and can cause problems (see [reference/uz801-internals.md](../reference/uz801-internals.md) for the BCM43438 firmware bug). Keep it on for now — disable it only after [04-lte-modem.md](04-lte-modem.md) is complete:

```bash
sudo nmcli radio wifi off
```

This setting persists across reboots.

---

## Next

→ [02-wiring.md](02-wiring.md) — wire up motors, gimbal, GPS, camera, and the LTE modem.
