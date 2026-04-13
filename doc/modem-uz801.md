# UZ801 LTE USB Modem

The rover uses a **UZ801** 4G LTE USB dongle connected to the Raspberry Pi Zero 2W. It runs a custom Android AOSP image on a Qualcomm MSM8916 SoC and exposes ADB + RNDIS interfaces over USB.

---

## Hardware

| Property       | Value                                  |
|----------------|----------------------------------------|
| Model          | UZ801                                  |
| Manufacturer   | Qualcomm Technology                    |
| SoC            | Qualcomm MSM8916 (Snapdragon 410)      |
| CPU ABI        | armeabi-v7a (32-bit ARM)               |
| Board          | ALK                                    |
| Serial number  | a1a2a0f7                               |

---

## Software / OS

| Property            | Value                                                             |
|---------------------|-------------------------------------------------------------------|
| Android version     | 4.4.4 (KitKat)                                                    |
| SDK level           | 19                                                                |
| Kernel              | Linux 3.10.28 (built 2024-08-12 by GCC 4.7)                      |
| Firmware build ID   | V2.3.12                                                           |
| Build date          | 2024-08-12 (UTC 1723443742)                                       |
| Build type          | `userdebug` — root + ADB enabled, no verified boot                |
| Build fingerprint   | `qcom/msm8916_32_512/msm8916_32_512:4.4.4/KTU84P/eng..20240812:userdebug/test-keys` |
| Firmware locale     | zh-CN (Chinese mainland firmware)                                 |
| Timezone (persist)  | Europe/Kiev                                                       |

---

## Cellular / Radio

| Property            | Value                                          |
|---------------------|------------------------------------------------|
| Baseband firmware   | N958St_Z85_CN_JSXPH1IDN1H213                   |
| RIL implementation  | Qualcomm RIL 1.0 (`libril-qc-qmi-1.so`)        |
| Network type        | LTE                                            |
| Current operator    | KYIVSTAR (Ukraine, MCC/MNC 255-03)             |
| SIM state           | READY                                          |
| Data roaming        | enabled (`ro.com.android.dataroaming=true`)    |
| DNS (rmnet0)        | 193.41.60.16 / 193.41.60.15                    |
| NITZ operator cache | lifecell (255-06) — previous operator stored   |
| Default network     | 11 (LTE preferred)                             |

---

## USB Interface

The modem exposes multiple USB functions simultaneously (persisted in `persist.sys.usb.config`):

| Function     | Purpose                                          |
|--------------|--------------------------------------------------|
| `rndis`      | Ethernet-over-USB — provides internet to the Pi  |
| `serial_smd` | Serial/AT command interface                      |
| `diag`       | Qualcomm diagnostic interface (QXDM)             |
| `adb`        | Android Debug Bridge — shell + file access       |

ADB TCP port is also open on **7628** (`persist.adb.tcp.port`).

---

## ADB Access from Pi

Verify the device is detected:

```bash
adb devices
# 0123456789ABCDEF  device
```

Open a shell:

```bash
adb shell
```

Root is available (`ro.secure=0`, `ro.debuggable=1`, `userdebug` build):

```bash
adb root    # restart adbd as root
adb shell   # now runs as root
```

---

## Network Architecture

```
Home WiFi or LTE (uplink selected at boot)
     │
   wlan0 (WiFi client) or rmnet0 (LTE)
     │
   NAT / masquerade (iptables)
     │
   br0  (192.168.100.1/24)
     └── rndis0  ←USB RNDIS→  usb0 on Pi  (192.168.100.100, static)
```

The modem acts as a NAT gateway: traffic from `br0` is masqueraded out through `wlan0` (preferred,
when `/data/misc/wifi/rover_wpa.conf` exists and association succeeds) or `rmnet0` (LTE fallback).

The Pi uses a **static IP** on `usb0`:
- IP: `192.168.100.100/24`
- Gateway: `192.168.100.1`
- DNS: `192.168.100.1` (modem forwards to carrier or home router DNS)

The factory WiFi hotspot (`wlan0` in AP mode, `hostapd`) is disabled — `wlan0` is used as a
WiFi client instead when a credential file is present.

---

## WiFi Hotspot

The factory firmware starts a WiFi hotspot on every boot, managed by **`com.mifiservice.hello`** — a privileged system APK at `/system/priv-app/MifiService.apk`.

**Default credentials:**
- SSID: `4G-UFI-3D4`
- Password: `1234567890`
- Channel: 6 (2.4 GHz, 802.11g), WPA2

MifiService also runs a **Jetty web UI on port 80** (`http://192.168.100.1`) with pages for WiFi, SIM, DHCP, and firmware settings.

**For the rover, the hotspot is disabled** (unnecessary RF emissions, wastes resources). See [Disabling the WiFi Hotspot](#disabling-the-wifi-hotspot) below.

---

## Disabling the WiFi Hotspot

This section documents exactly what was done on this device and how to repeat it on a replacement unit.

### Overview

MifiService manages the hotspot by:
1. Creating the `br0` bridge and adding `rndis0` + `wlan0`
2. Starting `hostapd` (WiFi AP)
3. Starting `dnsmasq` (DHCP + DNS)
4. Configuring iptables NAT rules

When disabled, we replace all of step 1, 3, and 4 with a boot script. Step 2 (hostapd/WiFi) is simply not started.

### Step 1 — Disable MifiService

Connect from the Pi and get root:

```bash
adb root
```

Disable the package permanently. This prevents its `BootCompletedReceiver` from firing on boot:

```bash
adb shell pm disable com.mifiservice.hello
```

Expected output: `Package com.mifiservice.hello new state: disabled`

To undo: `adb shell pm enable com.mifiservice.hello`

> LTE and RNDIS are **not** affected — they are managed by `ril-daemon` and `netmgrd` independently of MifiService.

### Step 2 — Write the RNDIS setup script

Remount `/system` as writable:

```bash
adb remount
```

The script is maintained at [`deploy/modem/nat_forward.sh`](../deploy/modem/nat_forward.sh).
It handles bridge setup, IP forwarding, NAT rules, WiFi client uplink (with LTE fallback),
the LED status daemon, and the LTE status TCP server. See the
[WiFi Client Mode](#wifi-client-mode-home-wifi-uplink) section for full details.

> **Important:** `adb push` strips the execute bit — always run `chmod 755` after pushing scripts to `/system`.

Deploy via Makefile (recommended):

```bash
make deploy-modem [RPI=pi@pi-rover.lan]
```

Or manually from the Pi:

```bash
adb remount
adb push /tmp/nat_forward.sh /system/etc/nat_forward.sh
adb shell chmod 755 /system/etc/nat_forward.sh
```

### Step 3 — Write the LED status script

Write `/system/etc/led_status.sh` with the following content:

```sh
#!/system/bin/sh
# LED connectivity status monitor
# Green solid  = LTE data up (rmnet0 has carrier IP + DNS)
# Red blinking = LTE data down / connecting (1s on, 1s off)
# wifi LED     = off (reserved for future use)

LED_RED=/sys/class/leds/red/brightness
LED_GREEN=/sys/class/leds/green/brightness
LED_WIFI=/sys/class/leds/wifi/brightness

PID_FILE=/data/logs/led_status.pid
LOG=/data/logs/led_status.log

log() { echo "$(getprop ro.build.date.utc) $$ $1" >> $LOG; }

# Singleton: kill any previous instance
if [ -f $PID_FILE ]; then
    kill $(cat $PID_FILE) 2>/dev/null
    rm -f $PID_FILE
fi
echo $$ > $PID_FILE

log "started"

led_on()  { echo 255 > $1; }
led_off() { echo 0   > $1; }

led_off $LED_WIFI

iter=0
while true; do
    iter=$((iter + 1))
    # LTE data is up if rmnet0 has a carrier-assigned DNS
    DNS=$(getprop net.rmnet0.dns1)
    if [ -n "$DNS" ]; then
        # LTE up — green solid, red off
        led_on  $LED_GREEN
        led_off $LED_RED
        [ $iter -le 3 ] && log "iter=$iter LTE_UP dns=$DNS"
        sleep 3
    else
        # LTE down / connecting — red blink (1s on, 1s off)
        led_off $LED_GREEN
        led_on  $LED_RED
        [ $iter -le 3 ] && log "iter=$iter LTE_DOWN"
        sleep 1
        led_off $LED_RED
        sleep 1
    fi
done
```

Push and make executable:

```bash
adb push led_status.sh /system/etc/led_status.sh
adb shell chmod 755 /system/etc/led_status.sh
```

### Step 5 — Hook into boot

`/system/etc/init.qcom.post_boot.sh` is executed automatically on every boot via the `qcom-post-boot` init service, triggered by `on property:sys.boot_completed=1` in `/init.qcom.rc`. Append the script call to the end of `init.qcom.post_boot.sh`:

Pull the file, append, push back:

```bash
adb pull /system/etc/init.qcom.post_boot.sh .
echo '' >> init.qcom.post_boot.sh
echo '# Restore NAT forwarding rules for RNDIS tethering (MifiService WiFi hotspot disabled)' >> init.qcom.post_boot.sh
echo '/system/etc/nat_forward.sh' >> init.qcom.post_boot.sh
adb push init.qcom.post_boot.sh /system/etc/init.qcom.post_boot.sh
```

### Step 6 — Verify

Reboot the modem (disconnect/reconnect USB or `adb reboot`) and confirm:

```bash
# 1. ADB is up and boot is complete
adb shell getprop sys.boot_completed
# expected: 1

# 2. MifiService, hostapd, and dnsmasq are NOT running
adb shell ps | grep -E '(mifiservice|hostapd|dnsmasq)'
# expected: no output

# 3. Bridge is up with correct IP
adb shell ip addr show br0
# expected: inet 192.168.100.1/24 ... state UP

# 4. IP forwarding is enabled
adb shell cat /proc/sys/net/ipv4/ip_forward
# expected: 1

# 5. NAT rules are in place
adb shell iptables -t nat -L natctrl_nat_POSTROUTING -n
# expected: MASQUERADE rule present (wlan0 if WiFi uplink, rmnet0 if LTE)

# 6. Internet works from Pi
ping -I usb0 -c 4 google.com
# expected: 0% packet loss, ~130ms (WiFi uplink) or ~35-50ms (LTE)

# 7. LED shows green (LTE up or WiFi up)
adb shell cat /sys/class/leds/green/brightness
# expected: 255
adb shell cat /sys/class/leds/red/brightness
# expected: 0
```

### What changes on the modem filesystem

| File | Change |
|------|--------|
| `/system/etc/nat_forward.sh` | **Created** — bridge + NAT setup, WiFi client uplink with LTE fallback, launches LED and LTE status daemons |
| `/system/etc/led_status.sh` | **Created** — background LED connectivity monitor |
| `/system/etc/lte_status_srv.sh` | **Created** — TCP server on port 8080 serving LTE signal/operator info to Pi daemon |
| `/system/etc/init.qcom.post_boot.sh` | **Appended** — calls `nat_forward.sh` at boot |
| `/data/logs/` | **Created** — log directory (`mkdir -p /data/logs && chmod 777 /data/logs`) |
| `/data/misc/wifi/rover_wpa.conf` | **Optional** — WiFi credentials; presence enables WiFi client uplink |
| Package state (in `/data/system/packages.xml`) | `com.mifiservice.hello` marked `disabled` |

All changes survive reboots. `/system` changes survive firmware updates only if the update doesn't wipe the system partition.

Logs at runtime:
- `/data/logs/nat_forward.log` — boot execution trace (one entry per reboot)
- `/data/logs/led_status.log` — LED daemon startup and first 3 poll cycles
- `/data/logs/wpa_supplicant.log` — wpa_supplicant output (WiFi client mode only)
- `/data/logs/lte_status_srv.log` — LTE status TCP server output

---

## LED Status

The modem has three LEDs controllable via sysfs (`/sys/class/leds/{red,green,wifi}/brightness`, values 0=off / 255=on). The kernel driver is `leds-gpio`; no `timer` or `netdev` triggers are compiled in, so blinking is done in software.

After disabling MifiService the LED defaults to solid red. The `led_status.sh` daemon (launched by `nat_forward.sh` at boot) replaces this with meaningful status:

| LED state        | Meaning                                      |
|------------------|----------------------------------------------|
| Green solid      | LTE data up (`rmnet0` has carrier-assigned IP + DNS) |
| Red blinking     | LTE data down / connecting (1 s on, 1 s off) |

**Connectivity indicator:** the script polls `getprop net.rmnet0.dns1` every 3 s (LTE up) or 2 s (blink cycle). The `wifi` LED is kept off (reserved for future use).

To verify after boot:
```bash
adb shell cat /sys/class/leds/green/brightness   # 255 when LTE up
adb shell cat /sys/class/leds/red/brightness     # 0 when LTE up
```

To control LEDs manually (for testing):
```bash
adb shell "echo 255 > /sys/class/leds/green/brightness"
adb shell "echo 0   > /sys/class/leds/red/brightness"
```

---

## Running Services (after hotspot disabled)

| Service              | State   | Role                                           |
|----------------------|---------|------------------------------------------------|
| `ril-daemon`         | running | Radio interface layer — manages LTE modem      |
| `netmgrd`            | running | Network manager daemon                         |
| `qmuxd`              | running | QMI multiplexer                                |
| `imsdatadaemon`      | running | IMS data (VoLTE support)                       |
| `imsqmidaemon`       | running | IMS QMI                                        |
| `adbd`               | running | ADB daemon                                     |
| `thermal-engine`     | running | Thermal management                             |
| `led_status.sh`      | running | LED connectivity indicator (launched by nat_forward.sh) |
| `lte_status_srv.sh`  | running | TCP server on port 8080 — LTE + WiFi signal/operator info for Pi daemon |
| `wpa_supplicant`     | running (WiFi) / stopped (LTE) | WiFi client — runs when `rover_wpa.conf` present and association succeeds |
| `com.mifiservice.hello` | disabled | WiFi hotspot manager — permanently disabled |
| `hostapd`            | stopped | WiFi AP — not started                          |
| `dnsmasq`            | stopped | Replaced by static IP on Pi side               |

---

## CPU Measurement

To measure CPU usage on the modem (useful for profiling services like dnsmasq):

```bash
# From Pi (adb must be available):
bash /tmp/modem-deploy/measure-cpu.sh [interval_seconds]

# Example — 5s window (default):
bash /tmp/modem-deploy/measure-cpu.sh

# Example — 10s window:
bash /tmp/modem-deploy/measure-cpu.sh 10
```

Or deploy first if not already on the Pi:

```bash
# From dev machine:
scp deploy/modem/measure-cpu.sh deploy/modem/measure-cpu-inner.sh pi@pi-rover.lan:/tmp/modem-deploy/
ssh pi@pi-rover.lan "bash /tmp/modem-deploy/measure-cpu.sh 5"
```

**Sample output:**

```
=== Modem CPU Measurement (5s window) ===

--- Total CPU ---
  User:    20%
  System:  33%
  IOWait:   1%
  Active:  55%
  Idle:    44%

--- Per-core ---
  cpu0:   8% active  (user=23 sys=19 idle=456 jiffies)
  cpu1: 100% active  (user=194 sys=329 idle=0 jiffies)

--- Processes with >0% CPU ---
  TOTAL%  PID      USER%  SYS%   NAME
  ----------------------------------------
  50%      1295      18%    31%    dnsmasq
   1%      1553       1%     0%    fiservice.hello
```

**Note:** dnsmasq consistently uses ~50% of one core on this device (Snapdragon 410, kernel 3.10).
This is a known behavior of the embedded dnsmasq build on Android 4.4 — the original MifiService
dnsmasq behaves the same way (~37% when WiFi hotspot is active). It does not affect rover operation.

---

## WiFi Client Mode (Home WiFi uplink)

The modem operates as a **WiFi client** (preferred uplink) with automatic LTE fallback.
Fully implemented in `nat_forward.sh` and deployed. Live-tested and confirmed working.

```
Home WiFi (192.168.50.0/24)
     │
   wlan0  ← UZ801 as WiFi client (wpa_supplicant + dhcpcd)
     │
   NAT / masquerade (iptables)
     │
   br0 (192.168.100.1/24)
     └── rndis0 ←USB→ usb0 on Pi (192.168.100.100)
```

If WiFi association or DHCP fails at boot, the script falls back to LTE (`rmnet0`) automatically.

### Why L2 bridging doesn't work

`brctl addif br0 wlan0` fails with _"Operation not supported on transport endpoint"_ — 802.11
client frames use 3-address format; transparent L2 bridging requires 4-address WDS frames which
home APs don't support. **NAT routing is the correct approach**, identical in pattern to the
existing LTE setup (`wlan0` replaces `rmnet0` as the egress interface).

### Components available on the modem

| Component | Location | Notes |
|-----------|----------|-------|
| WiFi kernel module | `/system/lib/modules/pronto/pronto_wlan.ko` | symlinked as `wlan.ko`; not auto-loaded after MifiService disabled |
| `wpa_supplicant` | `/system/bin/wpa_supplicant` | v2.1-devel, nl80211 + WPA2-PSK |
| `dhcpcd` | `/system/bin/dhcpcd` | v5.5.6; obtains DHCP lease from home router |

### Provisioning WiFi credentials

WiFi credentials are stored at `/data/misc/wifi/rover_wpa.conf` on the modem (persists across
reboots; survives firmware updates since `/data` is not wiped).

From the dev machine:

```bash
make setup-modem-wifi WIFI_SSID=S_HOME WIFI_PSK=password [RPI=pi@pi-rover.lan]
```

This runs `deploy/modem/setup-wifi-client.sh` on the Pi, which writes the config via `adb push`.

To revert to LTE-only:

```bash
make remove-modem-wifi [RPI=pi@pi-rover.lan]
# adb shell rm -f /data/misc/wifi/rover_wpa.conf && adb reboot
```

Config file format (no `update_config` — see pitfalls below):

```
ctrl_interface=/data/misc/wifi/sockets
ap_scan=1
fast_reauth=1

network={
    ssid="S_HOME"
    psk="password"
    key_mgmt=WPA-PSK
}
```

### Boot sequence (nat_forward.sh)

When `/data/misc/wifi/rover_wpa.conf` exists, `nat_forward.sh` runs this sequence:

1. **Load module** — `insmod /system/lib/modules/wlan.ko` if `wlan` not in `/proc/modules`; wait up to 15s for `wlan0` to appear, then sleep 5s for WCNSS firmware to finish initializing
2. **Kill stale wpa_supplicant** — using its tracked PID; remove stale socket files from `/data/misc/wifi/sockets/` (they persist across reboots and block new starts)
3. **Start wpa_supplicant** — `wpa_supplicant -i wlan0 -D nl80211 -c /data/misc/wifi/rover_wpa.conf -O /data/misc/wifi/sockets`
4. **Fast-fail check** — if no socket file after 3s, wpa_supplicant crashed; fall back to LTE immediately
5. **Wait for association** — poll `wpa_cli status | grep wpa_state=COMPLETED` up to 90s (cold WCNSS boot can take 60+ seconds)
6. **DHCP** — run `dhcpcd wlan0`; check for `default.*wlan0` route to confirm success
7. **NAT rules** — masquerade on `wlan0`, forward `br0 ↔ wlan0`; delete LTE default route so WiFi's is the only default
8. **Port forwarding** — DNAT specific ports on `wlan0` to the Pi; `POSTROUTING MASQUERADE` on `br0` to fix asymmetric routing (see below)
9. **LTE fallback** — if any step above fails, fall through to the standard `rmnet0` NAT rules

### Port forwarding to Pi (home network access)

When WiFi uplink is active, the modem port-forwards three ports from its `wlan0` IP
(`192.168.50.173`) directly to the Pi (`192.168.100.100`):

| Port | Protocol | Service |
|------|----------|---------|
| 22 | TCP | SSH |
| 14550 | UDP | MAVLink |
| 5600 | UDP | Video (RTP H.264) |

From any host on the home network:
```bash
ssh pi@192.168.50.173          # reaches Pi SSH
# MAVLink GCS: connect to 192.168.50.173:14550
# Video: udpsrc port=5600 on 192.168.50.173
```

**Asymmetric routing fix**: The Pi has both `wlan0` (home WiFi, metric 600) and `usb0`
(modem, metric 700). Without extra rules, DNAT'd packets arriving from `192.168.50.x` would
be replied to via `wlan0` directly — the client sees a SYN-ACK from the wrong IP and drops it.
Fix: `iptables -t nat -A POSTROUTING -o br0 -j MASQUERADE` on the modem makes the Pi see all
port-forwarded connections as coming from `192.168.100.1`, so it always replies via `usb0`.

> **Note:** The Pi's `wlan0` is now disabled (`sudo nmcli radio wifi off`) — only `usb0` is
> active. This eliminates the BCM43438 firmware disconnect-every-60s issue and the asymmetric
> routing concern entirely. The port-forward and MASQUERADE rules remain in place for when
> `wlan0` is re-enabled in future.

### Live test results

```
# Successful boot log (2026-04-08):
Wed Apr  8 11:44:31 EEST 2026 wifi: wpa_supplicant started pid=1352
Wed Apr  8 11:44:34 EEST 2026 wifi: associated after 0s
Wed Apr  8 11:44:43 EEST 2026 wifi: IP obtained, route: default via 192.168.50.1 dev wlan0 metric 324 — using WiFi as uplink

# Connectivity from Pi:
ping -I usb0 -c 3 8.8.8.8    # 133ms avg, 0% loss (via home WiFi → ISP)

# Routing on modem when WiFi uplink active:
default via 192.168.50.1 dev wlan0  metric 324
192.168.50.0/24 dev wlan0  src 192.168.50.173
192.168.100.0/24 dev br0   src 192.168.100.1
# (rmnet0 routes remain for specific IPs but no default via rmnet0)
```

### Pitfalls discovered during implementation

These traps are non-obvious and cost significant debugging time:

**1. Config filename `wpa_supplicant.conf` is special-cased**
Android's wpa_supplicant binary has hardcoded logic for the filename `wpa_supplicant.conf`
(triggers Android WiFi service socket path handling) and exits 255 when started with that path
outside the Android WiFi framework. **Use any other filename** — we use `rover_wpa.conf`.

**2. `update_config=1` corrupts the config file**
When `update_config=1` is set, wpa_supplicant rewrites the config file after reading it,
appending a NUL byte. On the next boot the parser fails silently and falls back to LTE.
**Omit `update_config` entirely** from the config.

**3. `ip link set wlan0 up` before wpa_supplicant breaks initialization**
Manually bringing `wlan0` UP puts the WCNSS chip into `<NO-CARRIER,BROADCAST,MULTICAST,UP>`
state. wpa_supplicant then cannot initialize the interface and exits immediately.
**Do not set wlan0 up** — wpa_supplicant manages interface state itself.

**4. Config must be `chmod 644`, not `600`**
Android's wpa_supplicant binary uses Linux capabilities to switch to the `wifi` user (uid=wifi)
regardless of being started as root. A `chmod 600 root:root` config is unreadable by the wifi
user → `"Failed to open config file: Permission denied"` in logcat.
**Always `chmod 644`** when writing the config via `adb push`.

**5. Stale socket files block new starts**
`/data/misc/wifi/sockets/wlan0` (and `p2p0`) are left from the previous run. Since `/data`
persists across reboots, these stale sockets prevent wpa_supplicant from binding its control
socket on the next boot. **`rm -f` the socket files** before starting wpa_supplicant.

**6. WCNSS firmware initialization timing is variable**
After a cold power cycle, the WCNSS WiFi chip can take anywhere from 0s to 60+ seconds to
finish firmware initialization before wpa_supplicant can use it. On a warm reboot (module
already loaded) it is typically instant. The 90s association timeout accommodates the worst case.

**8. WiFi RSSI is only readable via wpa_cli, not dumpsys or /proc**
`dumpsys wifi` always returns `RSSI: -9999` because the Android WiFi framework (`WifiService`)
is in `UNINITIALIZED` state — wpa_supplicant was started directly, bypassing the framework.
`/proc/net/wireless` always shows zeros because the Qualcomm WCNSS driver does not populate it.
The only working source is `wpa_cli -p /data/misc/wifi/sockets -iwlan0 signal_poll`, which
queries wpa_supplicant directly. Returns `FAIL` when not associated (safe to parse — `atoi("FAIL") = 0`).

**7. Android 4.4 shell is missing most Unix tools**
`tr`, `cut`, `awk`, `sed`, `killall`, `printf`, `tail` — none available. Use only `grep` and
POSIX shell builtins. For example, parse `ip route` output with `set -- $line; field=$3` instead
of `awk '{print $3}'`; check wpa_cli state with `grep -q "wpa_state=COMPLETED"` instead of
piping through `cut`.

---

## LTE Status Server

`lte_status_srv.sh` runs as a TCP server on port **8080** (`192.168.100.1:8080`). The Pi daemon
connects every 5 seconds and reads one line of `key=value` pairs, then the server loops and waits
for the next connection.

### Status line format

```
connected=1 rssi=27 netmode=LTE oper=KYIVSTAR uplink=wifi wifi_rssi=-65
```

| Key | Type | Description |
|-----|------|-------------|
| `connected` | `0`/`1` | LTE data active — `net.rmnet0.dns1` is non-empty |
| `rssi` | `0–31` | LTE signal strength (AT+CSQ scale; `0` = unknown) |
| `netmode` | string | Network type from `gsm.network.type` (`LTE`, `UMTS`, `EDGE`, …) |
| `oper` | string | Operator name from `gsm.operator.alpha` |
| `uplink` | `lte`/`wifi`/`unknown` | Active default-route interface (`rmnet0` → `lte`, `wlan0` → `wifi`) |
| `wifi_rssi` | `−10` to `−100` dBm, or `0` | WiFi AP signal strength; `0` when LTE uplink or not associated |

### WiFi RSSI source

Three approaches were evaluated on this modem — only one works:

| Source | Result | Reason |
|--------|--------|--------|
| `dumpsys wifi \| grep RSSI` | Always `-9999` | Android WiFi framework is `UNINITIALIZED` — WiFi is managed below the framework level |
| `/proc/net/wireless` | Always `0 0 0` | Qualcomm WCNSS driver does not populate this kernel file |
| `wpa_cli -p /data/misc/wifi/sockets -iwlan0 signal_poll` | **Correct RSSI** | Queries wpa_supplicant directly; returns `RSSI=-65\nLINKSPEED=86\n…` or `FAIL` if not associated |

The script only calls `wpa_cli` when `uplink=wifi`; on LTE uplink `wifi_rssi=0` is sent.

### How the Pi daemon uses this data

The daemon encodes both signals into a MAVLink **RADIO_STATUS** (message ID 109) sent at 1 Hz to QGroundControl:

| RADIO_STATUS field | WiFi uplink | LTE uplink |
|--------------------|-------------|------------|
| `rssi` | WiFi AP dBm (int8\_t as uint8\_t) | LTE dBm (`−113 + 2×CSQ`) |
| `remrssi` | LTE dBm (background signal) | `0` |
| `txbuf` | `50` (sentinel: WiFi active) | `100` (sentinel: LTE active) |

QGC displays `rssi` as **Local RSSI** and `remrssi` as **Remote RSSI** in the Telemetry RSSI Status widget. The signal icon appears automatically when `rssi > 0`.

> **Why not CELLULAR_STATUS (ID 334)?** QGC's codebase has no handler for message 334 — it is never decoded and no widget appears. RADIO_STATUS is fully supported.

---

## Pi Networking (current state)

The Pi's `wlan0` (BCM43438) is **disabled** via `sudo nmcli radio wifi off` (persistent across
reboots). Reason: the BCM43438 firmware has a bug causing a locally-generated deauthentication
every ~60s (reason=3), creating constant reconnect noise and asymmetric routing complexity.

The Pi is reachable exclusively via `usb0 → modem`:
- Direct (same subnet): `ssh pi@192.168.100.100`
- From home network via modem port-forward: `ssh pi@192.168.50.173`

To re-enable WiFi on the Pi if needed:
```bash
sudo nmcli radio wifi on
```

---

## Notes

- Firmware locale is `zh-CN` (Chinese OEM), but the modem works globally.
- `persist.sys.timezone = Europe/Kiev` — set from previous SIM usage.
- DNS upstream servers are read dynamically from `net.dns1`/`net.dns2` at boot — they reflect the current carrier automatically.
- Used by the daemon for LTE link monitoring — see [feature-roadmap.md](feature-roadmap.md).
