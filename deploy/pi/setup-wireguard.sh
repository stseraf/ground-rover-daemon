#!/usr/bin/env bash
# One-time setup for the rover WireGuard tunnel to the MikroTik hAP back-to-home-vpn.
# Installs wireguard-tools, generates a key pair, writes /etc/wireguard/wg0.conf,
# enables the wg-quick@wg0 systemd service, and prints the RouterOS peer command.
#
# Prerequisites: run '/interface wireguard print' on the hAP first to get its public key.
#
# Run via make (recommended):
#   make setup-wireguard HAP_PUBKEY="<key from RouterOS>" [RPI=pi@pi-rover.lan]
#
# Or directly on the Pi (must run as root):
#   sudo HAP_PUBKEY="<key>" bash setup-wireguard.sh
#
# Optional env vars:
#   HAP_ENDPOINT  — WireGuard endpoint (default: 46.33.34.161:51820)
#   PI_WG_ADDR    — Pi's WireGuard IPv4 address (default: 192.168.216.6/24)
#   PI_WG_ADDR6   — Pi's WireGuard IPv6 address (default: fc00:0:0:216::6/128)

set -e

HAP_PUBKEY="${HAP_PUBKEY:-}"
HAP_ENDPOINT="${HAP_ENDPOINT:-46.33.34.161:23392}"
PI_WG_ADDR="${PI_WG_ADDR:-192.168.216.6/24}"
PI_WG_ADDR6="${PI_WG_ADDR6:-fc00:0:0:216::6/128}"
WG_CONF=/etc/wireguard/wg0.conf
WG_ALLOWED="192.168.216.0/24, 192.168.50.0/24"
WG_KEEPALIVE=3

step() { echo ""; echo "==> $1"; }
ok()   { echo "  [OK] $1"; }

if [ "$(id -u)" -ne 0 ]; then
    echo "ERROR: run with sudo" >&2
    exit 1
fi

if [ -z "$HAP_PUBKEY" ]; then
    echo ""
    echo "ERROR: HAP_PUBKEY is not set."
    echo ""
    echo "Run this on the MikroTik hAP to get the public key:"
    echo "  /interface wireguard print"
    echo ""
    echo "Then re-run:"
    echo "  make setup-wireguard HAP_PUBKEY=\"<public-key>\" [RPI=pi@pi-rover.lan]"
    exit 1
fi

step "Installing wireguard-tools"
apt-get install -y wireguard wireguard-tools
ok "wireguard-tools installed"

step "Generating WireGuard key pair"
mkdir -p /etc/wireguard
chmod 700 /etc/wireguard

if [ -f "$WG_CONF" ]; then
    ok "wg0.conf already exists — skipping key generation (remove $WG_CONF to regenerate)"
    PRIVKEY=$(awk '/PrivateKey/{print $3}' "$WG_CONF")
    PUBKEY=$(echo "$PRIVKEY" | wg pubkey)
else
    PRIVKEY=$(wg genkey)
    PUBKEY=$(echo "$PRIVKEY" | wg pubkey)

    step "Writing $WG_CONF"
    cat > "$WG_CONF" <<EOF
[Interface]
PrivateKey = ${PRIVKEY}
Address = ${PI_WG_ADDR}, ${PI_WG_ADDR6}

[Peer]
PublicKey = ${HAP_PUBKEY}
Endpoint = ${HAP_ENDPOINT}
AllowedIPs = ${WG_ALLOWED}
PersistentKeepalive = ${WG_KEEPALIVE}
EOF
    chmod 600 "$WG_CONF"
    ok "Written $WG_CONF"
fi

step "Enabling wg-quick@wg0 service"
systemctl enable --now wg-quick@wg0
ok "wg-quick@wg0 enabled and started"

PI_WG_IP="${PI_WG_ADDR%%/*}"
PI_WG_IP6="${PI_WG_ADDR6%%/*}"

echo ""
echo "============================================================"
echo "  RouterOS command — run on the MikroTik hAP console"
echo "============================================================"
echo ""
echo "/interface wireguard peers add interface=back-to-home-vpn \\"
echo "  comment=\"rover\" \\"
echo "  public-key=\"${PUBKEY}\" \\"
echo "  allowed-address=${PI_WG_IP}/32,${PI_WG_IP6}/128 \\"
echo "  persistent-keepalive=3s"
echo ""
echo "# If LAN <-> VPN forwarding rules are not already present:"
echo "/ip firewall filter add chain=forward in-interface=bridge out-interface=back-to-home-vpn action=accept comment=\"LAN to rover\""
echo "/ip firewall filter add chain=forward in-interface=back-to-home-vpn out-interface=bridge action=accept comment=\"rover to LAN\""
echo "============================================================"
echo ""
echo "After adding the peer on the hAP, verify the tunnel:"
echo "  sudo wg show                # peer with latest-handshake within ~3s"
echo "  ping 192.168.216.1          # ping hAP over tunnel"
echo "  ssh pi@${PI_WG_IP}         # from PC on home LAN (192.168.50.x)"
