# 5. WireGuard VPN Setup

The rover is on LTE (or home WiFi) with no inbound ports. WireGuard gives it a stable, home-LAN-reachable IP (`192.168.216.6` / `pi-rover.lan`) so you can SSH in, receive MAVLink, and receive video from any PC on the home LAN — regardless of whether the rover's uplink is LTE or WiFi.

The Pi initiates an outbound tunnel to the MikroTik hAP's `back-to-home-vpn` interface. `PersistentKeepalive = 3` keeps both NAT layers open and restores the tunnel in ≤ 3 seconds after any uplink change.

```
rover Pi (192.168.216.6)
  wg0 ──[UDP to hAP:23392]──▶ back-to-home-vpn (192.168.216.1)
                                              │
PC on home LAN (192.168.50.x) ─── ssh pi-rover.lan ──▶ hAP → tunnel → Pi
```

---

## 5.1 Prerequisites

- A MikroTik hAP (or any WireGuard-capable router) with a WireGuard interface already set up on the home side.
- An open UDP port on the router's WAN (default: `23392`).
- DNS entry (`pi-rover.lan` → `192.168.216.6`) on the router.

---

## 5.2 Get the hAP's WireGuard details (RouterOS)

```routeros
/interface wireguard print
```

Record two values from the `back-to-home-vpn` entry:
- `public-key` → used as `HAP_PUBKEY` below
- `listen-port` → used in `HAP_ENDPOINT` (public-IP:port)

---

## 5.3 Configure the Pi

```bash
make setup-wireguard HAP_PUBKEY="<key from step 5.2>" \
                     HAP_ENDPOINT="<public-ip>:<port>"
```

This:
1. Installs `wireguard-tools`.
2. Generates `/etc/wireguard/wg0.conf` with a fresh key pair.
3. Enables `wg-quick@wg0` so it starts on boot.
4. Prints the Pi's public key and the exact RouterOS command to run in step 5.4.

**Optional overrides** (if `.6` is already taken):

```bash
make setup-wireguard HAP_PUBKEY="…" HAP_ENDPOINT="…" \
                     PI_WG_ADDR=192.168.216.7/24 \
                     PI_WG_ADDR6=fc00:0:0:216::7/128
```

---

## 5.4 Add the Pi as a peer (RouterOS)

Paste the command printed by step 5.3. It looks like:

```routeros
/interface wireguard peers add interface=back-to-home-vpn \
  comment="rover" \
  public-key="<PI_PUBKEY>" \
  allowed-address=192.168.216.6/32,fc00:0:0:216::6/128 \
  persistent-keepalive=3s
```

The Pi initiates — the hAP learns the Pi's current NAT'd IP:port on the first handshake and follows it through LTE IP changes automatically.

If not already present, allow inbound WireGuard UDP and LAN ↔ VPN forwarding:

```routeros
/ip firewall filter add chain=input in-interface=pppoe-out1 \
  protocol=udp dst-port=23392 action=accept \
  comment="WireGuard back-to-home-vpn" place-before=0

/ip firewall filter add chain=forward in-interface=bridge \
  out-interface=back-to-home-vpn action=accept \
  comment="LAN to rover" place-before=0

/ip firewall filter add chain=forward in-interface=back-to-home-vpn \
  out-interface=bridge action=accept \
  comment="rover to LAN" place-before=0
```

---

## 5.5 DNS (RouterOS)

```routeros
/ip dns static add name=pi-rover.lan address=192.168.216.6
# or to update an existing entry:
/ip dns static set [find name=pi-rover.lan] address=192.168.216.6
```

---

## 5.6 Verify

On the Pi:

```bash
sudo systemctl status wg-quick@wg0
sudo wg show                     # latest handshake should be within seconds
ping 192.168.216.1               # hAP's VPN address
```

From any PC on the home LAN:

```bash
ssh pi-rover.lan                 # works from WiFi or LTE uplink
```

---

## 5.7 How it interacts with the LTE ↔ WiFi uplink

The Pi's only uplink is `usb0` (the modem). Whether the modem is using LTE or home WiFi is invisible to the Pi — its routing table never changes.

| Modem uplink | Path | NAT hops |
|---|---|---|
| WiFi | Pi → modem MASQUERADE → home router NAT → internet → hAP | Double NAT |
| LTE | Pi → modem MASQUERADE → LTE carrier → internet → hAP | Single NAT |

`PersistentKeepalive = 3` keeps both NAT layers open (3 s ≪ 60 s minimum NAT timeout). When an uplink switch changes the source IP, the Pi daemon also bounces `wg0` automatically to force a fresh handshake within ~3 s.

See [features/lte-uplink.md](../features/lte-uplink.md) for the runtime switching feature.

---

## Troubleshooting

**`wg show` shows no handshake, 0 B received**
The hAP peer is missing or the endpoint port is wrong. Verify:
```bash
sudo grep Endpoint /etc/wireguard/wg0.conf
```
Must match `listen-port` from `/interface wireguard print` on the hAP.

**Tunnel up but PC can't reach 192.168.216.6**
Forward firewall rules are missing or ordered after a DROP rule:
```routeros
/ip firewall filter print where chain=forward
# The "LAN to rover" accept rule must appear before any DROP-all rule
```

**Handshake succeeds but `ping 192.168.216.1` fails**
The hAP doesn't have an IP on its WireGuard interface:
```routeros
/ip address print
# Expect: 192.168.216.1/24 on back-to-home-vpn
```

**Tunnel doesn't come up after reboot**
```bash
sudo systemctl status wg-quick@wg0
sudo journalctl -u wg-quick@wg0 -n 30
```
Usually: `wireguard-tools` not installed or `/etc/wireguard/wg0.conf` missing. Re-run `make setup-wireguard`.

**Regenerate keys**
```bash
sudo rm /etc/wireguard/wg0.conf
make setup-wireguard HAP_PUBKEY="…" HAP_ENDPOINT="…"
# Then on the hAP: remove the old peer and add the new one using the public key printed by setup-wireguard.
```

---

## Next

→ [06-qgroundcontrol.md](06-qgroundcontrol.md) — connect QGroundControl and verify everything works.
