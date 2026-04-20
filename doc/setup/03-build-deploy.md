# 3. Build & Deploy the Daemon

Cross-compile from your dev machine, deploy to the Pi via SSH, run as a `systemd` service.

---

## 3.1 Dev machine prerequisites

```bash
sudo apt install build-essential make gcc-aarch64-linux-gnu g++-aarch64-linux-gnu
```

Clone with the MAVLink submodule:

```bash
git clone --recurse-submodules <repo-url> ground-rover-daemon
cd ground-rover-daemon
# or, if already cloned:
git submodule update --init --recursive
```

---

## 3.2 Build variants

Five independent `make` variables control the build:

| Variable | Values | Default | Notes |
|---|---|---|---|
| `ARCH` | `host`, `rpi` | `host` | `rpi` cross-compiles for aarch64 |
| `DRIVER` | `stub`, `tb6612` | `stub` | Motor driver backend |
| `GIMBAL` | `stub`, `i2c` | `stub` | Gimbal controller backend (`i2c` implemented but not yet hardware-tested) |
| `GPS` | `stub`, `nmea` | `stub` | GPS backend |
| `LTE` | `stub`, `usb` | `stub` | LTE modem monitoring + link switching |

A native (host) build for local testing:

```bash
make
./build/ground_rover_daemon
```

The default rover build (used by `make deploy`) enables motors, GPS, and LTE — the gimbal stays on the stub:

```bash
make rebuild ARCH=rpi DRIVER=tb6612 GPS=nmea LTE=usb
```

Add `GIMBAL=i2c` only when the optional gimbal hardware is present (implemented but not yet hardware-tested — see [features/gimbal.md](../features/gimbal.md)).

---

## 3.3 Deploy to the Pi

The default target host is `pi@pi-rover.lan`. Override with `RPI=pi@…` on any target.

```bash
make deploy                              # default
make deploy RPI=pi@192.168.1.42          # custom host
```

`make deploy` does:

1. Cross-compiles with all hardware backends enabled.
2. Stops the running service (if any).
3. Copies `build/ground_rover_daemon` to `/home/pi/ground-rover-daemon/`.
4. Installs `deploy/ground-rover-daemon.service` into `/etc/systemd/system/`.
5. Enables and starts the service.

To pin the target in your shell:

```bash
echo 'export RPI=pi@pi-rover.lan' >> ~/.bashrc
```

---

## 3.4 Service management

The daemon runs as `pi` with `WorkingDirectory=/home/pi/ground-rover-daemon/`. The `params` file lives next to the binary.

```bash
sudo systemctl start   ground-rover-daemon
sudo systemctl stop    ground-rover-daemon
sudo systemctl restart ground-rover-daemon
systemctl status       ground-rover-daemon
journalctl -u ground-rover-daemon -f       # tail logs
journalctl -u ground-rover-daemon -b       # logs since last boot
```

To run manually (useful for debugging):

```bash
sudo systemctl stop ground-rover-daemon
cd /home/pi/ground-rover-daemon
./ground_rover_daemon
```

---

## 3.5 Verify QGC can connect

On your ground control PC, launch QGroundControl. It auto-discovers rovers on UDP 14550. Expect:

- **Vehicle ID 1** appears in the top bar
- **MAV_TYPE_GROUND_ROVER** icon
- Telemetry RSSI Status widget shows signal bars (LTE/WiFi)
- Parameters panel lists 8 `DRIVE_*` / `GPS_*` / `VIDEO_*` / `CTRL_*` / `NET_*` params

For detailed QGC walkthrough see [06-qgroundcontrol.md](06-qgroundcontrol.md).

---

## Next

→ [04-lte-modem.md](04-lte-modem.md) — flash the UZ801 modem with the rover firmware (one-time).
