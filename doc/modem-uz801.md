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
SIM / LTE
  └── rmnet0  (modem data interface, 10.x.x.x, assigned by carrier)
        └── br0  (Linux bridge, 192.168.100.1/24)
              ├── rndis0  ←USB RNDIS→  usb0 on Pi  (192.168.100.x DHCP)
              └── wlan0   ←WiFi AP→    WiFi clients (192.168.100.x DHCP)
```

The modem acts as a NAT gateway: traffic from `br0` is masqueraded out through `rmnet0`. Both the Pi (via USB RNDIS) and any WiFi clients share the same `192.168.100.0/24` subnet.

The Pi receives its lease on `usb0` with:
- IP: `192.168.100.x/24`
- Gateway: `192.168.100.1`
- DNS: forwarded by modem's `dnsmasq` to carrier DNS

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

Write `/system/etc/nat_forward.sh` with the following content:

```sh
#!/system/bin/sh
# RNDIS tethering setup (MifiService WiFi hotspot disabled via pm disable)
# Called from /system/etc/init.qcom.post_boot.sh on boot_completed

LOG=/data/logs/nat_forward.log
log() { echo "$(date) $1" >> $LOG; }

log "=== nat_forward.sh start ==="

BRIDGE=br0
IFACE=rndis0
GW=192.168.100.1
DHCP_RANGE_START=192.168.100.100
DHCP_RANGE_END=192.168.100.200
LEASE_TIME=12h
LEASE_FILE=/data/misc/dhcp/dnsmasq.leases
PID_FILE=/data/misc/dhcp/dnsmasq.pid

# --- Bridge setup ---
log "bridge setup"
brctl addbr $BRIDGE 2>/dev/null
brctl addif $BRIDGE $IFACE 2>/dev/null
ip addr flush dev $BRIDGE 2>/dev/null
ip addr add $GW/24 dev $BRIDGE
ip link set $IFACE up
ip link set $BRIDGE up
log "bridge done"

# --- DHCP server ---
log "dnsmasq setup"
# Kill any stale instance
if [ -f $PID_FILE ]; then
    kill $(cat $PID_FILE) 2>/dev/null
    rm -f $PID_FILE
fi
# Use LTE operator DNS as upstream (from net.dns1/net.dns2 at runtime)
DNS1=$(getprop net.dns1)
DNS2=$(getprop net.dns2)
[ -z "$DNS1" ] && DNS1=8.8.8.8
[ -z "$DNS2" ] && DNS2=8.8.4.4
log "DNS1=$DNS1 DNS2=$DNS2"

/system/bin/dnsmasq \
    --interface=$BRIDGE \
    --dhcp-range=$DHCP_RANGE_START,$DHCP_RANGE_END,$LEASE_TIME \
    --dhcp-option=3,$GW \
    --dhcp-option=6,$GW \
    --server=$DNS1 \
    --server=$DNS2 \
    --no-hosts \
    --dhcp-leasefile=$LEASE_FILE \
    --pid-file=$PID_FILE
log "dnsmasq done rc=$?"

# --- IP forwarding ---
log "ip forwarding"
echo 1 > /proc/sys/net/ipv4/ip_forward
echo 1 > /proc/sys/net/ipv4/conf/br0/forwarding
echo 1 > /proc/sys/net/ipv4/conf/rmnet0/forwarding

# --- NAT forwarding ---
log "iptables"
iptables -t nat -D natctrl_nat_POSTROUTING -o rmnet0 -j MASQUERADE 2>/dev/null
iptables -t nat -A natctrl_nat_POSTROUTING -o rmnet0 -j MASQUERADE
iptables -F natctrl_FORWARD
iptables -A natctrl_FORWARD -i $BRIDGE -o rmnet0 -j ACCEPT
iptables -A natctrl_FORWARD -i rmnet0 -o $BRIDGE -m state --state ESTABLISHED,RELATED -j ACCEPT
log "iptables done"

# --- LED status daemon ---
log "launching led_status.sh"
/system/etc/led_status.sh > /data/logs/led_status.log 2>&1 &
log "led_status launched pid=$!"

log "=== nat_forward.sh done ==="
```

> **Important:** `adb push` strips the execute bit — always run `chmod 755` after pushing scripts to `/system`.

Push and make executable:

```bash
adb push nat_forward.sh /system/etc/nat_forward.sh
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

# 2. MifiService and hostapd are NOT running
adb shell ps | grep -E '(mifiservice|hostapd)'
# expected: no output

# 3. dnsmasq IS running
adb shell ps | grep dnsmasq
# expected: /system/bin/dnsmasq

# 4. Bridge is up with correct IP
adb shell ip addr show br0
# expected: inet 192.168.100.1/24 ... state UP

# 5. IP forwarding is enabled
adb shell cat /proc/sys/net/ipv4/ip_forward
# expected: 1

# 6. NAT rules are in place
adb shell iptables -t nat -L natctrl_nat_POSTROUTING -n
# expected: MASQUERADE rule present

# 7. Internet works from Pi
ping -I usb0 -c 4 google.com
# expected: 0% packet loss, ~35-50ms

# 8. LED shows green (LTE up)
adb shell cat /sys/class/leds/green/brightness
# expected: 255
adb shell cat /sys/class/leds/red/brightness
# expected: 0
```

### What changes on the modem filesystem

| File | Change |
|------|--------|
| `/system/etc/nat_forward.sh` | **Created** — bridge + DHCP + NAT setup script, also launches LED daemon |
| `/system/etc/led_status.sh` | **Created** — background LED connectivity monitor |
| `/system/etc/init.qcom.post_boot.sh` | **Appended** — calls `nat_forward.sh` at boot |
| `/data/logs/` | **Created** — log directory (`mkdir -p /data/logs && chmod 777 /data/logs`) |
| Package state (in `/data/system/packages.xml`) | `com.mifiservice.hello` marked `disabled` |

All changes survive reboots. `/system` changes survive firmware updates only if the update doesn't wipe the system partition.

Logs at runtime:
- `/data/logs/nat_forward.log` — boot execution trace (one entry per reboot)
- `/data/logs/led_status.log` — LED daemon startup and first 3 poll cycles

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
| `dnsmasq`            | running | DHCP + DNS for br0 (started by nat_forward.sh) |
| `com.mifiservice.hello` | disabled | WiFi hotspot manager — permanently disabled |
| `hostapd`            | stopped | WiFi AP — not started                          |
| `wpa_supplicant`     | stopped | Client Wi-Fi — not used                        |

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

## Remote Access (CGNAT + VPN)

### IP address type

The modem receives a **private IP on `rmnet0`** from the carrier — not a public routable address:

```
rmnet0: inet 10.1.228.249/30   ← private, assigned by KYIVSTAR
External (seen from internet):  46.211.43.50  ← shared with other subscribers
```

Verify:
```bash
adb shell ip addr show rmnet0
curl --interface usb0 https://api.ipify.org
```

If `rmnet0` shows `10.x`, `172.16-31.x`, `192.168.x`, or `100.64-100.127.x` (CGNAT range), you are behind carrier NAT. This is the case for KYIVSTAR consumer SIMs (and most LTE carriers globally).

**Consequence: no inbound connections are possible over LTE.** Port forwarding on the modem cannot help — packets from the internet never reach the modem's public IP because the carrier owns it.

---

### Tailscale (recommended)

[Tailscale](https://tailscale.com) creates a WireGuard-based mesh VPN between devices. Install on both the Pi and the ground station machine; each device gets a stable `100.x.x.x` address reachable from anywhere regardless of the underlying network.

```bash
# On Pi
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up

# On ground station — install Tailscale and sign in with the same account
```

After setup:
- QGC connects to `100.x.x.x:14550` (Pi's Tailscale IP) — works over LTE or WiFi transparently
- SSH: `ssh pi@100.x.x.x`
- Pi's Tailscale address is stable across network changes (LTE ↔ WiFi switch)
- Free tier: up to 3 users / 100 devices

Check whether the connection is direct P2P or going through a relay server:
```bash
tailscale status
# Look for "direct" (good — P2P) or "relay" (traffic via Tailscale DERP server)
```

---

### Encryption overhead

Tailscale uses **WireGuard** for encryption. WireGuard is designed for low overhead:

- CPU cost: ~0.1 ms per packet on ARM (negligible on Pi Zero 2W)
- The measured LTE latency is ~50 ms — encryption adds nothing noticeable
- Throughput tested: LTE 22.9 Mbit/s down / 40.1 Mbit/s up — well within video streaming budget

The real latency variable is whether Tailscale establishes a **direct P2P tunnel** or falls back to a **DERP relay**. Direct P2P is normal when one side (ground station) has a real public IP; relay adds one extra hop (~10–50 ms).

---

### Alternative: raw WireGuard via VPS

If the ground station also has a real public IP (typical home ISP), you can skip Tailscale and run WireGuard directly — the Pi initiates an outbound connection to the ground station. This avoids any coordination server and has identical encryption overhead.

If neither endpoint has a real public IP, a cheap VPS (Hetzner, DigitalOcean) can act as a WireGuard relay — full control, no third-party service dependency.

---

## Notes

- Firmware locale is `zh-CN` (Chinese OEM), but the modem works globally.
- `persist.sys.timezone = Europe/Kiev` — set from previous SIM usage.
- DNS upstream servers are read dynamically from `net.dns1`/`net.dns2` at boot — they reflect the current carrier automatically.
- Used by the daemon for LTE link monitoring — see [feature-roadmap.md](feature-roadmap.md).
