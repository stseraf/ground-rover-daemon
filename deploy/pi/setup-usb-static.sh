#!/usr/bin/env bash
# Configure static IP on the Pi's RNDIS USB interface (usb0) for LTE modem tethering.
# The modem runs without dnsmasq — static assignment replaces DHCP.
#
# Run directly on the Pi:  bash setup-usb-static.sh
# Or via make:             make setup-pi-usb [RPI=pi@pi-rover.lan]
#
# Persistent across reboots:
#   NetworkManager: writes a connection profile to /etc/NetworkManager/system-connections/
#   dhcpcd:         appends a static block to /etc/dhcpcd.conf
# Both apply the config automatically whenever usb0 appears (including after modem reconnect).

set -e

IFACE=usb0
STATIC_IP=192.168.100.100
PREFIX=24
GATEWAY=192.168.100.1
DNS1=8.8.8.8
DNS2=8.8.4.4
NM_CONN_NAME="modem-usb-static"

step() { echo ""; echo "==> $1"; }
die()  { echo "ERROR: $1" >&2; exit 1; }

step "Detecting network manager"
if systemctl is-active --quiet NetworkManager 2>/dev/null; then
    NM=true
    echo "  NetworkManager detected"
elif systemctl is-active --quiet dhcpcd 2>/dev/null; then
    NM=false
    echo "  dhcpcd detected"
else
    die "Neither NetworkManager nor dhcpcd is active"
fi

if $NM; then
    step "Configuring static IP via NetworkManager"
    sudo nmcli connection delete "$NM_CONN_NAME" 2>/dev/null || true
    sudo nmcli connection add \
        type ethernet \
        ifname "$IFACE" \
        con-name "$NM_CONN_NAME" \
        ipv4.method manual \
        ipv4.addresses "${STATIC_IP}/${PREFIX}" \
        ipv4.gateway "$GATEWAY" \
        ipv4.dns "${DNS1} ${DNS2}" \
        ipv4.route-metric 100 \
        connection.autoconnect yes
    sudo nmcli connection up "$NM_CONN_NAME" 2>/dev/null || true
else
    step "Configuring static IP via dhcpcd"
    DHCPCD_CONF=/etc/dhcpcd.conf
    if grep -q "# modem-usb-static" "$DHCPCD_CONF" 2>/dev/null; then
        sudo sed -i '/# modem-usb-static/,/# end modem-usb-static/d' "$DHCPCD_CONF"
    fi
    sudo tee -a "$DHCPCD_CONF" > /dev/null <<EOF

# modem-usb-static
interface $IFACE
static ip_address=${STATIC_IP}/${PREFIX}
static routers=$GATEWAY
static domain_name_servers=$DNS1 $DNS2
# end modem-usb-static
EOF
    sudo ip addr flush dev "$IFACE" 2>/dev/null || true
    sudo ip addr add "${STATIC_IP}/${PREFIX}" dev "$IFACE" 2>/dev/null || true
    sudo ip link set "$IFACE" up 2>/dev/null || true
    sudo ip route replace default via "$GATEWAY" dev "$IFACE" 2>/dev/null || true
fi

step "Verifying"
sleep 1
IP=$(ip addr show "$IFACE" 2>/dev/null | grep "inet ${STATIC_IP}" || echo "")
[ -n "$IP" ] && echo "  [OK] $IFACE has ${STATIC_IP}/${PREFIX}" \
             || echo "  [WARN] IP not yet visible — interface appears when modem connects"

GW_REACHABLE=$(ping -c 1 -W 2 "$GATEWAY" 2>/dev/null | grep "1 received" || echo "")
[ -n "$GW_REACHABLE" ] && echo "  [OK] Gateway $GATEWAY reachable" \
                       || echo "  [WARN] Gateway not reachable — is the modem connected and booted?"

echo ""
echo "Static IP configuration complete."
echo "Pi will use ${STATIC_IP}/${PREFIX} on $IFACE with GW $GATEWAY on every boot."
