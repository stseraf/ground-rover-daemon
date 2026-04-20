# 4. UZ801 LTE Modem Setup

The rover's internet link is a **UZ801** 4G USB dongle — a tiny Android-based modem that exposes RNDIS (ethernet-over-USB), ADB, and serial over a single USB connection.

This is a **one-time** setup: disable the factory hotspot, deploy the rover scripts, optionally provision home WiFi credentials, then set up a static IP on the Pi side.

For hardware specs, internal architecture, and troubleshooting see [reference/uz801-internals.md](../reference/uz801-internals.md).

---

## 4.1 Verify ADB access

The UZ801 exposes ADB over USB — used for pushing scripts. From the Pi:

```bash
sudo apt install android-tools-adb
adb devices
# List of devices attached
# 0123456789ABCDEF  device
```

If the device shows `unauthorized`, accept the prompt (not applicable here — the UZ801's `userdebug` build auto-authorizes). If nothing shows up, re-plug the dongle.

Switch adbd to root (the `userdebug` firmware allows this):

```bash
adb root
```

---

## 4.2 Disable the factory WiFi hotspot

The factory firmware runs a WiFi hotspot (`4G-UFI-3D4`) managed by `com.mifiservice.hello`. We don't want it — disable the package permanently:

```bash
adb shell pm disable com.mifiservice.hello
# Package com.mifiservice.hello new state: disabled
```

LTE data and RNDIS are unaffected (they are managed by `ril-daemon` / `netmgrd`, independent of MifiService).

To re-enable: `adb shell pm enable com.mifiservice.hello`.

---

## 4.3 Deploy the rover scripts

`make deploy-modem` pushes the rover's boot-time scripts to the modem, appends a hook to the Android `post_boot` script, and reboots. After reboot the modem sets up NAT forwarding for the Pi, a status TCP server, and a link-switch TCP server.

From your dev machine:

```bash
make deploy-modem
```

What gets written on the modem:

| File | Purpose |
|---|---|
| `/system/etc/nat_forward.sh` | Bridge + NAT at boot; picks WiFi or LTE uplink |
| `/system/etc/led_status.sh` | LED daemon (green=LTE up, red blink=down) |
| `/system/etc/lte_status_srv.sh` | TCP server on port 8080 — serves signal/operator info |
| `/system/etc/link_switch_srv.sh` | TCP server on port 8081 — accepts `wifi`/`lte` switch commands |
| `/system/etc/init.qcom.post_boot.sh` | Appended to call `nat_forward.sh` on `boot_completed` |

`make deploy-modem` reboots the modem and then runs `verify-modem` which polls the Pi back online and checks:

- NAT rules present
- IP forwarding enabled
- Bridge `br0` up with `192.168.100.1/24`
- `lte_status_srv.sh` responding on port 8080
- LED in the correct state

---

## 4.4 Configure static IP on the Pi's `usb0`

The Pi needs a static address on the RNDIS interface to reach the modem's NAT gateway. Run once:

```bash
make setup-pi-usb
```

This writes a `NetworkManager` profile that configures `usb0` with:

- IP: `192.168.100.100/24`
- Gateway: `192.168.100.1`
- DNS: `192.168.100.1`

After this, `ping -I usb0 8.8.8.8` from the Pi should succeed.

---

## 4.5 (Optional) Use home WiFi instead of LTE

The modem can act as a WiFi client when at home — saves LTE data and improves latency. Provide your home SSID + PSK:

```bash
make setup-modem-wifi WIFI_SSID=S_HOME WIFI_PSK=password
```

This writes `/data/misc/wifi/rover_wpa.conf` on the modem. On the next modem boot `nat_forward.sh` will connect to the home WiFi and use it as the uplink. If WiFi is unreachable it falls back to LTE automatically.

To revert to LTE-only:

```bash
make remove-modem-wifi
```

Runtime switching between uplinks (without reboot) is also supported — see [features/lte-uplink.md](../features/lte-uplink.md).

---

## 4.6 Verify end-to-end

From the Pi:

```bash
ping -I usb0 -c 3 8.8.8.8              # internet via modem
cat /dev/tcp/192.168.100.1/8080        # reads one status line
# e.g. connected=1 rssi=27 netmode=LTE oper=KYIVSTAR uplink=lte wifi_rssi=0
```

From the dev machine:

```bash
make verify-modem
```

---

## Next

→ [05-wireguard.md](05-wireguard.md) — set up the VPN so you can reach the rover from any PC on the home LAN.
