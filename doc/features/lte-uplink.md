# LTE Link Monitoring & Uplink Switching

The UZ801 modem exposes its state (signal, operator, active uplink) to the Pi on TCP port `8080` and accepts uplink-switch commands on TCP port `8081`. The Pi daemon polls status at 1 Hz, forwards it as MAVLink `RADIO_STATUS`, and flips uplinks on demand via the `NET_LINK_PREF` parameter.

Build variant: `make LTE=usb`.

---

## Data path

```
Modem (192.168.100.1)
  ├── lte_status_srv.sh    — TCP 8080, one status line per connection
  └── link_switch_srv.sh   — TCP 8081, accepts "wifi" / "lte" commands

Pi daemon (UsbLteMonitor, LinkSwitcher)
  ├── poll 8080 every 5 s → parse status → RADIO_STATUS + STATUSTEXT
  ├── on NET_LINK_PREF change → connect 8081, write cmd
  └── on uplink change → bounce wg0 (WG session gets stuck on NAT path change)

QGC
  ├── RSSI status widget (local + remote)
  └── Parameters: NET_LINK_PREF
```

Source: [`src/lte/usb_lte_monitor.cpp`](../../src/lte/usb_lte_monitor.cpp), [`src/lte/link_switcher.cpp`](../../src/lte/link_switcher.cpp).

---

## Status line format (port 8080)

```
connected=1 rssi=27 netmode=LTE oper=KYIVSTAR uplink=wifi wifi_rssi=-65
```

| Key | Type | Description |
|---|---|---|
| `connected` | 0/1 | LTE data active (`net.rmnet0.dns1` non-empty) |
| `rssi` | 0–31 | LTE signal strength (AT+CSQ scale; 0 = unknown) |
| `netmode` | string | Network type from `gsm.network.type` (`LTE`, `UMTS`, …) |
| `oper` | string | Operator name (`gsm.operator.alpha`) |
| `uplink` | `lte` / `wifi` / `unknown` | Active default-route interface |
| `wifi_rssi` | −10 to −100 dBm, or 0 | WiFi AP signal; 0 on LTE or unassociated |

---

## How the daemon encodes this for QGC

QGC has no handler for `CELLULAR_STATUS` (msg 334), so the daemon uses `RADIO_STATUS` (msg 109) which is fully supported and drives the built-in RSSI widget.

| RADIO_STATUS field | WiFi uplink | LTE uplink |
|---|---|---|
| `rssi` | WiFi AP dBm | LTE dBm (`−113 + 2×CSQ`) |
| `remrssi` | LTE dBm (background) | `0` |
| `txbuf` | `50` (sentinel: WiFi active) | `100` (sentinel: LTE active) |

QGC displays `rssi` as **Local RSSI** and `remrssi` as **Remote RSSI**. The signal icon appears automatically when `rssi > 0`.

The daemon also emits `STATUSTEXT` messages on transitions:

- `"Link: WiFi S_HOME -65 dBm"` when switching to WiFi
- `"Link: LTE KYIVSTAR LTE"` when switching to LTE

Transient `uplink=unknown` states (seen during a switch while routes are mid-swap) are suppressed to avoid spurious duplicate messages.

---

## Runtime uplink switching

Controlled by the `NET_LINK_PREF` MAVLink parameter, settable live from QGC's Parameters panel.

| Value | Behavior |
|---|---|
| `0` | Auto — keep current uplink (default) |
| `1` | Prefer WiFi — daemon sends `wifi` to modem:8081 |
| `2` | Force LTE — daemon sends `lte` to modem:8081 |

The parameter is persisted in `params`.

---

## What the modem does on each command

The modem's `link_switch_srv.sh` receives one line per TCP connection:

| Command | Actions | Time |
|---|---|---|
| `wifi` | `killall wpa_supplicant dhcpcd` → `rmmod wlan` → `insmod` → wait for `wlan0` → start `wpa_supplicant` → wait ≤ 45 s for `/sys/class/net/wlan0/carrier=1` → delete LTE default route → `dhcpcd wlan0` → swap iptables to WiFi NAT + port-forwarding | ~15–45 s |
| `lte` | `killall wpa_supplicant dhcpcd` → flush wlan0 addresses → flush WiFi iptables → re-add LTE MASQUERADE + FORWARD rules → restore `ip route default via $net.rmnet0.gw dev rmnet0` | ~2 s |

On any failure during the `wifi` path, the modem falls back to `lte` automatically. The server is async — a new command cancels an in-progress switch.

Why the WiFi path needs the full module reload + unusual choices around `pidof`/`wpa_cli` is documented in [../reference/uz801-internals.md](../reference/uz801-internals.md).

---

## WireGuard auto-recovery

The kernel WireGuard session can get stuck when the modem's egress IP changes (same `wg0` config, but a new source IP arrives at the remote — WG waits for its own handshake timer rather than reacting). To avoid a stall, the daemon runs this on every real `wifi ↔ lte` transition:

```bash
sudo wg-quick down wg0 && sudo wg-quick up wg0
```

Runs in the background, logged to `/tmp/wg-restart.log` on the Pi. Relies on the Pi's `NOPASSWD: ALL` sudo config (set in [../setup/01-raspberry-pi.md](../setup/01-raspberry-pi.md)).

---

## End-to-end timing

| Transition | Time to QGC restored |
|---|---|
| LTE → WiFi | ~15–45 s (module reload + WPA2 + DHCP) + ~3 s WG re-handshake |
| WiFi → LTE | ~2 s (route restore) + ~3 s WG re-handshake |

QGC does **not** disconnect during the switch — the Pi's UDP sockets stay bound and the daemon keeps running; only the NAT path underneath is swapping. Video freezes briefly, MAVLink catches up within one heartbeat interval after the tunnel comes back.

---

## Wiring & first-time setup

See [../setup/03-wiring.md §3.5](../setup/03-wiring.md#35-uz801-lte-modem--custom-cable-required) for the custom cable pinout and [../setup/05-lte-modem.md](../setup/05-lte-modem.md) for the modem-side firmware setup.
