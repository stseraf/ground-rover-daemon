# UZ801 Modem — Internals & Pitfalls

Reference material for the UZ801 4G USB dongle: hardware specs, Android OS details, service architecture, and the non-obvious traps encountered when replacing the factory hotspot with a rover uplink.

For the end-user setup flow see [../setup/04-lte-modem.md](../setup/04-lte-modem.md). For the runtime monitoring and switching feature see [../features/lte-uplink.md](../features/lte-uplink.md).

---

## Hardware

| Property | Value |
|---|---|
| Model | UZ801 |
| Manufacturer | Qualcomm Technology |
| SoC | Qualcomm MSM8916 (Snapdragon 410) |
| CPU ABI | armeabi-v7a (32-bit ARM) |
| Board | ALK |

---

## Software / OS

| Property | Value |
|---|---|
| Android version | 4.4.4 (KitKat) |
| SDK level | 19 |
| Kernel | Linux 3.10.28 (GCC 4.7, built 2024-08-12) |
| Firmware build ID | V2.3.12 |
| Build type | `userdebug` — root + ADB enabled, no verified boot |
| Build fingerprint | `qcom/msm8916_32_512/msm8916_32_512:4.4.4/KTU84P/…:userdebug/test-keys` |
| Firmware locale | zh-CN (Chinese mainland firmware) |

---

## Cellular / Radio

| Property | Value |
|---|---|
| Baseband firmware | N958St_Z85_CN_JSXPH1IDN1H213 |
| RIL implementation | Qualcomm RIL 1.0 (`libril-qc-qmi-1.so`) |
| Data roaming | enabled (`ro.com.android.dataroaming=true`) |
| Default network | 11 (LTE preferred) |

DNS servers for the LTE uplink are read dynamically from `net.rmnet0.dns1` / `net.rmnet0.dns2` — they reflect the current carrier automatically.

---

## USB interface

`persist.sys.usb.config` keeps multiple functions active simultaneously:

| Function | Purpose |
|---|---|
| `rndis` | Ethernet-over-USB — provides internet to the Pi |
| `serial_smd` | Serial / AT command interface |
| `diag` | Qualcomm diagnostic interface (QXDM) |
| `adb` | Android Debug Bridge — shell + file access |

ADB TCP port is also open on **7628** (`persist.adb.tcp.port`).

---

## Network architecture

```
Home WiFi or LTE (uplink selected at boot, switchable at runtime)
     │
   wlan0 (WiFi client)  or  rmnet0 (LTE)
     │
   NAT / MASQUERADE (iptables)
     │
   br0  (192.168.100.1/24)
     └── rndis0  ←USB RNDIS→  usb0 on Pi  (192.168.100.100, static)
```

The factory AP-mode `wlan0` + `hostapd` + `dnsmasq` is disabled; `wlan0` is reused as a WiFi client when `/data/misc/wifi/rover_wpa.conf` is present.

---

## Running services (after hotspot disabled)

| Service | State | Role |
|---|---|---|
| `ril-daemon` | running | Radio interface layer — manages LTE modem |
| `netmgrd` | running | Network manager daemon |
| `qmuxd` | running | QMI multiplexer |
| `imsdatadaemon` / `imsqmidaemon` | running | IMS data / QMI (VoLTE) |
| `adbd` | running | ADB daemon |
| `thermal-engine` | running | Thermal management |
| `led_status.sh` | running | LED connectivity indicator |
| `lte_status_srv.sh` | running | TCP server on 8080 — LTE + WiFi signal info |
| `link_switch_srv.sh` | running | TCP server on 8081 — uplink switch commands |
| `wpa_supplicant` | running (WiFi) / stopped (LTE) | WiFi client — runs when `rover_wpa.conf` present |
| `com.mifiservice.hello` | **disabled** | Factory hotspot manager |
| `hostapd` / `dnsmasq` | stopped | Not started (hotspot disabled) |

---

## Filesystem changes made by the rover deploy

| Path | Change |
|---|---|
| `/system/etc/nat_forward.sh` | Created — bridge + NAT at boot; picks WiFi or LTE uplink |
| `/system/etc/led_status.sh` | Created — LED connectivity daemon |
| `/system/etc/lte_status_srv.sh` | Created — status TCP server, port 8080 |
| `/system/etc/link_switch_srv.sh` | Created — switch-command TCP server, port 8081 |
| `/system/etc/init.qcom.post_boot.sh` | Appended — calls `nat_forward.sh` on `sys.boot_completed=1` |
| `/data/logs/` | Created (`chmod 777`) — all rover-script logs |
| `/data/misc/wifi/rover_wpa.conf` | Optional — enables WiFi client uplink |
| package state | `com.mifiservice.hello` marked disabled in `/data/system/packages.xml` |

Logs at runtime:
- `/data/logs/nat_forward.log` — boot trace
- `/data/logs/led_status.log` — LED daemon
- `/data/logs/lte_status_srv.log` — status server
- `/data/logs/link_switch_srv.log` — switch server (runtime uplink changes)
- `/data/logs/wpa_supplicant.log` — WiFi client mode only

All changes survive reboots. `/system` changes survive firmware updates only if the update doesn't wipe the system partition.

---

## LED semantics

Three GPIO-driven LEDs at `/sys/class/leds/{red,green,wifi}/brightness` (0 = off, 255 = on). Kernel driver is `leds-gpio`; no hardware `timer` or `netdev` triggers, so blinking is done in software by `led_status.sh`.

| State | Meaning |
|---|---|
| Green solid | LTE data up (`rmnet0` has carrier-assigned DNS) |
| Red blinking (1 s on, 1 s off) | LTE data down / connecting |
| `wifi` LED | Always off (reserved for future use) |

---

## Why L2 bridging doesn't work

`brctl addif br0 wlan0` fails with *"Operation not supported on transport endpoint"* — 802.11 client frames use 3-address format; transparent L2 bridging requires 4-address WDS frames, which home APs don't support. NAT routing is the correct approach (`wlan0` replaces `rmnet0` as the egress interface).

---

## Port-forward rules when WiFi uplink is active

When the modem is on home WiFi (e.g. `wlan0 = 192.168.50.173`), it DNATs three ports to the Pi at `192.168.100.100`:

| Port | Protocol | Service |
|---|---|---|
| 22 | TCP | SSH |
| 14550 | UDP | MAVLink |
| 5600 | UDP | Video (RTP H.264) |

From any host on the home network:

```bash
ssh pi@192.168.50.173
# MAVLink GCS: connect to 192.168.50.173:14550
# Video: udpsrc port=5600 on 192.168.50.173
```

**Asymmetric-routing fix:** without an extra MASQUERADE, DNAT'd packets from `192.168.50.x` would be replied to via the Pi's own WiFi interface (if enabled), not via `usb0`. `iptables -t nat -A POSTROUTING -o br0 -j MASQUERADE` on the modem makes the Pi see all incoming connections as coming from `192.168.100.1`, forcing the reply via `usb0`.

Since the Pi's on-board WiFi is disabled on the production rover, this is moot — but the rule is kept in place for setups that do use it.

---

## WCNSS WiFi pitfalls

The UZ801's WCNSS chip + Android 4.4 userspace combination has a dozen non-obvious traps. These are the ones that were hit during development — documenting them saves significant re-debugging time.

### 1. Config filename `wpa_supplicant.conf` is special-cased
Android's `wpa_supplicant` has hardcoded logic for the filename `wpa_supplicant.conf` (triggers Android WiFi-framework socket-path handling) and exits 255 when started with that path outside the framework. **Use any other filename** — we use `rover_wpa.conf`.

### 2. `update_config=1` corrupts the config file
With `update_config=1`, `wpa_supplicant` rewrites the config after reading, appending a NUL byte. On the next boot the parser fails silently and the modem falls back to LTE. **Omit `update_config` entirely.**

### 3. `ip link set wlan0 up` before wpa_supplicant breaks init
Manually bringing `wlan0` UP puts the WCNSS chip into `<NO-CARRIER,BROADCAST,MULTICAST,UP>` state. `wpa_supplicant` then can't initialise the interface and exits immediately. **Do not set wlan0 up** — `wpa_supplicant` manages interface state.

### 4. Config must be `chmod 644`, not `600`
`wpa_supplicant` uses Linux capabilities to switch to the `wifi` user (uid=wifi) even when started as root. A `chmod 600 root:root` config is unreadable by that user → `"Failed to open config file: Permission denied"` in logcat. **Always `chmod 644`.**

### 5. Stale control sockets block new starts
`/data/misc/wifi/sockets/{wlan0,p2p0}` are left from the previous run. `/data` persists across reboots, so these stale sockets prevent `wpa_supplicant` from binding its control socket. **`rm -f` the socket files** before starting.

### 6. WCNSS firmware init timing is variable
After a cold power cycle the chip can take 0 to 60+ seconds to finish firmware init. On a warm reboot (module already loaded) it's usually instant. The 45–90 s association timeout in rover scripts accommodates the worst case.

### 7. Android 4.4 shell is missing most Unix tools
`tr`, `cut`, `awk`, `sed`, `killall`, `printf`, `tail` — none available. Use only `grep` and POSIX shell built-ins. Examples:
- Parse `ip route` with `set -- $line; field=$3` instead of `awk '{print $3}'`.
- Check wpa state with `grep -q "wpa_state=COMPLETED"` instead of piping through `cut`.

`busybox` *is* present, so `busybox killall`, `busybox nc`, etc. do work.

### 8. WiFi RSSI is only readable via `wpa_cli`, not `dumpsys` or `/proc`
- `dumpsys wifi` → always `RSSI: -9999` (WiFi framework is `UNINITIALIZED` — we started `wpa_supplicant` directly).
- `/proc/net/wireless` → always `0 0 0` (WCNSS driver doesn't populate it).
- `wpa_cli … signal_poll` → **correct value**; returns `RSSI=-65\nLINKSPEED=…` or `FAIL` if not associated (safe to parse: `atoi("FAIL") = 0`).

### 9. `pidof wpa_supplicant` misses the real process
`wpa_supplicant` re-execs itself as `user=wifi` (via file capabilities). Android 4.4's `pidof` inspects only a subset of `/proc` and silently misses processes that have changed uid. A root script's `kill $(pidof wpa_supplicant)` returns nothing and leaves the old instance running. **Use `busybox killall wpa_supplicant`** — it scans all of `/proc` regardless of uid. Same for `dhcpcd`.

### 10. `wpa_cli` ctrl socket disappears over time
After minutes to hours, `/data/misc/wifi/sockets/wlan0` can vanish even though `wpa_supplicant` is still running and associated. Subsequent `wpa_cli` calls fail with `"Failed to connect to non-global ctrl_ifname"`. **Don't rely on `wpa_cli` for long-running association detection.** Use `/sys/class/net/wlan0/carrier` (kernel flag, always accurate: `1` = associated).

### 11. Killing `wpa_supplicant` leaves the chip stuck
After `busybox killall wpa_supplicant`, a freshly-started `wpa_supplicant` can initialise (socket appears) but **never associates** — `carrier` stays `0` indefinitely. The chip firmware is in an internal state the driver can't clear via a simple restart. The only reliable recovery is a full module reload: `rmmod wlan && insmod /system/lib/modules/wlan.ko`, then a ~5 s settle before starting `wpa_supplicant`. `link_switch_srv.sh` always does this on `wifi` switches.

### 12. `dhcpcd wlan0` fails when LTE is still the default route
`dhcpcd` sends a DHCP Discover to `255.255.255.255`. The kernel routes it via the default route — if that's still `rmnet0`, the broadcast goes out LTE and never reaches the home router. Symptom: `no IPv4_addresses`, no `default.*wlan0` route appears. **Delete the LTE default route before invoking `dhcpcd`:**

```sh
ip route del default via $(getprop net.rmnet0.gw) dev rmnet0
```

If WiFi association then fails, restore the LTE route.

---

## CPU measurement

```bash
make modem-cpu [INTERVAL=5]
```

Sample output:

```
=== Modem CPU Measurement (5s window) ===

--- Total CPU ---
  User:    20%
  System:  33%
  Active:  55%
  Idle:    44%

--- Per-core ---
  cpu0:   8% active
  cpu1: 100% active

--- Processes with >0% CPU ---
  TOTAL%  PID      USER%  SYS%   NAME
  50%     1295      18%    31%    dnsmasq
   1%     1553       1%     0%    fiservice.hello
```

**Note:** `dnsmasq` consistently uses ~50 % of one core on this device (Snapdragon 410, kernel 3.10). It's a known behavior of the embedded `dnsmasq` build on Android 4.4 — the original MifiService `dnsmasq` behaves the same way (~37 % when the factory hotspot is active). It does not affect rover operation.

---

## Pi-side WiFi disable (not related to modem, but relevant to the uplink path)

The Pi's on-board BCM43438 WiFi is disabled on the rover:

```bash
sudo nmcli radio wifi off    # persistent
```

Reason: the BCM43438 firmware has an internal ~60 s timer that triggers a locally-generated deauthentication (reason=3) on every connection, regardless of AP configuration. Root cause is a conflict between firmware `ccode=ALL` and the AP's country IE (UA), triggering periodic `CTRL-EVENT-REGDOM-CHANGE` cycles. `ccode=UA` in NVRAM doesn't fix it — the firmware ignores the NVRAM parameter on this chip variant (43436s).

Mitigation if you need to re-enable Pi WiFi: limit `wpa_supplicant` scan to a single channel (e.g. `scan_freq=2462`) so reconnect is sub-second. Persistable via `/etc/NetworkManager/dispatcher.d/99-scan-freq`.

With the modem providing all uplink, disabling Pi WiFi eliminates this noise and the asymmetric-routing complexity.
