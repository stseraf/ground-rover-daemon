# GPS Telemetry

NMEA 0183 GPS module (GY-GPS6MV2 / u-blox NEO-6M) on the Pi's hardware UART, forwarded to QGC as `GPS_RAW_INT` + `GLOBAL_POSITION_INT` at 1 Hz.

Build variant: `make GPS=nmea`.

---

## Data path

```
NEO-6M
  ↓ NMEA 0183 @ 9600 baud
/dev/serial0 (PL011 UART)
  ↓
NmeaGpsProvider  (parses $GNRMC + $GNGGA)
  ↓ GpsFix struct
main loop (1 Hz)
  ↓ MAVLink
QGC — map shows vehicle position
```

Source: [`src/gps/nmea_gps_provider.cpp`](../../src/gps/nmea_gps_provider.cpp). Interface: [`src/gps/gps_provider.hpp`](../../src/gps/gps_provider.hpp).

---

## MAVLink messages emitted

| Message | Fields populated | Frequency |
|---|---|---|
| `GPS_RAW_INT` (24) | `lat`, `lon`, `alt`, `eph`, `epv`, `vel`, `cog`, `fix_type`, `satellites_visible` | 1 Hz |
| `GLOBAL_POSITION_INT` (33) | `lat`, `lon`, `alt`, `relative_alt`, `hdg`, `vx`, `vy`, `vz` | 1 Hz |

`relative_alt` is altitude above the first valid fix (set as "home" on first `fix_type >= 2`).

---

## State-change logging

Rather than flooding stdout with every NMEA sentence, the daemon logs only meaningful state changes:

- Fix acquired or lost (`fix_type` transitions)
- Number of visible satellites per constellation (GPS, GLONASS, Galileo, BeiDou)
- Signal quality class (good / moderate / poor)
- Antenna status (OK / open / shorted), when advertised by the module
- Module identity on boot (reads the `$GNTXT` banner from NEO-6M on startup)

This gives a clean 5-10 line boot log plus one line per fix change, visible via `journalctl -u ground-rover-daemon`.

---

## `GPS_RAW_LOG` parameter

| Value | Behavior |
|---|---|
| `0` (default) | State-change logs only |
| `1` | Also log every raw NMEA sentence |

Toggle live from QGC's Parameters panel for field diagnostics — no restart needed. Setting persists to `params`.

---

## UART configuration

`/dev/serial0` must resolve to `/dev/ttyAMA0` (the PL011). If it points to `ttyS0` (the mini UART), Bluetooth has the PL011 — see [../setup/01-raspberry-pi.md](../setup/01-raspberry-pi.md) for the `disable-bt` overlay.

Compile-time constants (`include/config.hpp`):

| Constant | Value |
|---|---|
| `Config::Gps::UART_DEV` | `/dev/serial0` |
| `Config::Gps::BAUD_RATE` | `9600` |

---

## Troubleshooting

**No fix indoors** — expected. The NEO-6M needs clear sky. First fix after a cold start takes 30–60 s.

**QGC shows "GPS off"** — verify the daemon is reading NMEA:
```bash
ssh pi@pi-rover.lan
sudo systemctl stop ground-rover-daemon
cat /dev/serial0    # should show $GNRMC, $GNGGA sentences scrolling
```

**Permission denied on `/dev/serial0`** — ensure the `pi` user is in the `dialout` group (handled during Pi setup).

**Wrong device (ttyS0 instead of ttyAMA0)** — Bluetooth is still using the PL011. Check that `dtoverlay=disable-bt` is in `/boot/firmware/config.txt` and reboot.

---

## Wiring

See [../setup/02-wiring.md §2.2](../setup/02-wiring.md#22-gy-gps6mv2--u-blox-neo-6m-gps).
