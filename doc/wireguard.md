# WireGuard Tunnel

Provides SSH and MAVLink/video access to the rover Pi from any PC on the home LAN, even when
the rover is on LTE with no inbound ports. The Pi initiates an outbound WireGuard tunnel to the
MikroTik hAP and gets a stable VPN IP (`192.168.216.6`). A `PersistentKeepalive = 3` ensures
the tunnel reconnects within ≤3s after an LTE/WiFi uplink restoration.

```
rover Pi (192.168.216.6)
  wg0 ──[UDP 46.33.34.161:23392]──▶ hAP back-to-home-vpn (192.168.216.1)
                                              │
PC (192.168.50.x) ──── ssh pi-rover.lan ─────▶ routed via hAP → tunnel → Pi
```

---

## How it interacts with the WiFi/LTE uplink

The Pi always has a single uplink: `usb0 → 192.168.100.1 (modem)`. Whether the modem is using
its WiFi uplink or LTE fallback is invisible to the Pi — its routing table never changes.

| Modem uplink | WireGuard path | NAT hops |
|---|---|---|
| WiFi (home LAN) | Pi → modem MASQUERADE → home router NAT → internet → hAP | Double NAT |
| LTE (carrier) | Pi → modem MASQUERADE → LTE carrier → internet → hAP | Single NAT |

The modem's port-forward rules (TCP 22, UDP 14550, UDP 5600) only match inbound traffic on
`wlan0` — they do not affect WireGuard's outbound UDP flow.

`PersistentKeepalive = 3` keeps both NAT layers alive in WiFi double-NAT mode
(3s ≪ 60s minimum NAT timeout) and drives fast reconnection:

- LTE/WiFi restores → Pi sends keepalive within ≤3s → hAP re-learns Pi's endpoint → handshake (~100ms)
- **Total reconnect time: ≤3s + ~100ms**

---

## Addresses

The `back-to-home-vpn` interface already has peers at `.2`–`.5` and `::2`–`::5`. The rover
uses the next free slot:

| | IPv4 | IPv6 |
|---|---|---|
| hAP `back-to-home-vpn` | `192.168.216.1` | `fc00:0:0:216::1` |
| rover Pi `wg0` | `192.168.216.6` | `fc00:0:0:216::6` |

DNS: `pi-rover.lan` is mapped to `192.168.216.6` in the hAP's static DNS, so
`ssh pi-rover.lan` resolves correctly from any home LAN device.

---

## One-time setup

### Step 1 — Get hAP WireGuard interface details (RouterOS)

```routeros
/interface wireguard print
```

Note two values from `back-to-home-vpn`:
- `public-key` — needed as `HAP_PUBKEY` in Step 2
- `listen-port` — the actual UDP port the hAP is listening on (e.g. `23392`); use as `HAP_ENDPOINT`

### Step 2 — Configure the Pi

```bash
make setup-wireguard HAP_PUBKEY="<key from Step 1>" HAP_ENDPOINT="46.33.34.161:<port from Step 1>"
```

This runs `deploy/pi/setup-wireguard.sh` on the Pi, which:
1. Installs `wireguard-tools`
2. Generates `/etc/wireguard/wg0.conf` with a fresh key pair
3. Enables and starts `wg-quick@wg0`
4. Prints the Pi's public key and the exact RouterOS peer command to run

**Optional overrides** (if `.6` is already taken):
```bash
make setup-wireguard HAP_PUBKEY="..." HAP_ENDPOINT="46.33.34.161:23392" \
  PI_WG_ADDR=192.168.216.6/24 PI_WG_ADDR6=fc00:0:0:216::6/128
```

The generated `/etc/wireguard/wg0.conf`:
```ini
[Interface]
PrivateKey = <generated>
Address = 192.168.216.6/24, fc00:0:0:216::6/128

[Peer]
PublicKey = <HAP_PUBKEY>
Endpoint = 46.33.34.161:23392
AllowedIPs = 192.168.216.0/24, 192.168.50.0/24
PersistentKeepalive = 3
```

`AllowedIPs` routes VPN-subnet and home LAN traffic through the tunnel. The Pi's default route
via `usb0` is unchanged (MAVLink, video, LTE monitoring are unaffected).

### Step 3 — Add Pi as a peer (RouterOS)

Paste the command printed by Step 2:

```routeros
/interface wireguard peers add interface=back-to-home-vpn \
  comment="rover" \
  public-key="<PI-PUBKEY>" \
  allowed-address=192.168.216.6/32,fc00:0:0:216::6/128 \
  persistent-keepalive=3s
```

No `endpoint-address` is set — the Pi initiates, and the hAP learns the Pi's current NAT'd
IP:port on the first handshake. On LTE IP changes, the hAP updates automatically.

Allow WireGuard UDP inbound on the PPPoE interface (if not already present):

```routeros
/ip firewall filter add chain=input in-interface=pppoe-out1 \
  protocol=udp dst-port=23392 action=accept \
  comment="WireGuard back-to-home-vpn" place-before=0
```

If the LAN ↔ VPN forwarding rules are not already in place:

```routeros
/ip firewall filter add chain=forward in-interface=bridge out-interface=back-to-home-vpn \
  action=accept comment="LAN to rover" place-before=0
/ip firewall filter add chain=forward in-interface=back-to-home-vpn out-interface=bridge \
  action=accept comment="rover to LAN" place-before=0
```

### Step 4 — Update DNS (RouterOS)

```routeros
/ip dns static set [find name=pi-rover.lan] address=192.168.216.6
```

---

## Usage

```bash
# From any PC on the home LAN (192.168.50.x) — works on both WiFi and LTE uplinks:
ssh pi-rover.lan
# or by IP:
ssh pi@192.168.216.6

# Tunnel status on the Pi:
sudo wg show
```

---

## Verification

```bash
# Pi — service status:
sudo systemctl status wg-quick@wg0

# Pi — peer handshake (latest-handshake should be within ~3s):
sudo wg show

# Pi — ping hAP over tunnel:
ping 192.168.216.1

# From PC on home LAN — end-to-end:
ssh pi-rover.lan
```

---

## Troubleshooting

**`wg show` shows no handshake and 0 B received**
Either the Pi peer hasn't been added on the hAP yet, or the endpoint port is wrong.
Verify the port with `/interface wireguard print` on the hAP and ensure `HAP_ENDPOINT` matches:
```bash
sudo grep Endpoint /etc/wireguard/wg0.conf
# must match the listen-port shown by /interface wireguard print
```

**`wg show` shows no peer or public key mismatch**
The Pi peer has not been added on the hAP, or the public keys don't match. Verify:
```routeros
/interface wireguard peers print
```

**Tunnel up but PC can't reach 192.168.216.6**
The forward firewall rules are missing or ordered after a DROP rule:
```routeros
/ip firewall filter print where chain=forward
# The "LAN to rover" accept rule must appear before any DROP-all forward rule
```

**`wg show` shows handshake but ping to 192.168.216.1 fails**
Check the hAP has an IP on `back-to-home-vpn`:
```routeros
/ip address print
# Should show 192.168.216.1/24 on back-to-home-vpn
```

**Tunnel doesn't come up after reboot**
```bash
sudo systemctl status wg-quick@wg0
sudo journalctl -u wg-quick@wg0 -n 30
```
Common cause: `wireguard-tools` not installed or `/etc/wireguard/wg0.conf` missing. Re-run
`make setup-wireguard`.

**Re-run setup (regenerate keys)**
```bash
sudo rm /etc/wireguard/wg0.conf
make setup-wireguard HAP_PUBKEY="..." HAP_ENDPOINT="46.33.34.161:23392"
# Then update the peer on the hAP with the new public key:
# /interface wireguard peers remove [find where public-key=<old>]
# /interface wireguard peers add interface=back-to-home-vpn public-key=<new> \
#   allowed-address=192.168.216.6/32,fc00:0:0:216::6/128
```
