#!/usr/bin/env bash
# Verify UZ801 modem is healthy after deploy/reboot.
# Run ON THE PI via: make verify-modem
# Run directly on Pi: bash /tmp/modem-deploy/verify-modem.sh

set -e

step() { echo ""; echo "==> $1"; }
die()  { echo "ERROR: $1" >&2; exit 1; }

step "Verifying"

MIFI=$(adb shell pm list packages -d 2>/dev/null | grep mifiservice || echo "")
[ -n "$MIFI" ] && echo "  [OK] MifiService disabled" || echo "  [WARN] Could not confirm MifiService state"

DNSMASQ=$(adb shell ps 2>/dev/null | grep dnsmasq || echo "")
[ -z "$DNSMASQ" ] && echo "  [OK] dnsmasq not running (static IP mode)" || echo "  [WARN] dnsmasq still running — stale process?"

LTE_SRV_PID=$(adb shell "cat /data/logs/lte_status_srv.pid 2>/dev/null" | tr -d '\r' || echo "")
if [ -n "$LTE_SRV_PID" ] && adb shell "kill -0 $LTE_SRV_PID 2>/dev/null" >/dev/null 2>&1; then
    echo "  [OK] lte_status_srv running (pid=$LTE_SRV_PID)"
else
    echo "  [WARN] lte_status_srv not running"
fi

BR0=$(adb shell ip addr show br0 2>/dev/null | grep "192.168.100.1" || echo "")
[ -n "$BR0" ] && echo "  [OK] br0 has correct IP" || die "br0 missing or wrong IP"

IP_FWD=$(adb shell cat /proc/sys/net/ipv4/ip_forward 2>/dev/null | tr -d '\r' || echo 0)
[ "$IP_FWD" = "1" ] && echo "  [OK] IP forwarding enabled" || die "IP forwarding not enabled"

GREEN=$(adb shell cat /sys/class/leds/green/brightness 2>/dev/null | tr -d '\r' || echo 0)
RED=$(adb shell cat /sys/class/leds/red/brightness 2>/dev/null | tr -d '\r' || echo 255)
[ "$GREEN" = "255" ] && [ "$RED" = "0" ] && echo "  [OK] LED: green (LTE up)" || echo "  [WARN] LED not green (green=$GREEN red=$RED) — LTE may still be connecting"

DEFAULT_DEV=$(adb shell ip route 2>/dev/null | grep "^default" | grep -o "dev [^ ]*" | grep -o "[^ ]*$" | head -1 | tr -d '\r')
case "$DEFAULT_DEV" in
    wlan0)  UPLINK="wifi (wlan0)"  ;;
    rmnet0) UPLINK="lte (rmnet0)"  ;;
    "")     UPLINK="unknown — no default route?" ;;
    *)      UPLINK="unknown ($DEFAULT_DEV)" ;;
esac
echo "  [OK] Active uplink: $UPLINK"

# Firewall rules — depend on active uplink
NAT=$(adb shell iptables -t nat -L natctrl_nat_POSTROUTING -n -v 2>/dev/null | tr -d '\r')
FWD=$(adb shell iptables -L natctrl_FORWARD -n -v 2>/dev/null | tr -d '\r')

echo "$NAT" | grep -q "MASQUERADE.*$DEFAULT_DEV" \
    && echo "  [OK] NAT MASQUERADE on $DEFAULT_DEV" \
    || echo "  [WARN] NAT MASQUERADE missing on $DEFAULT_DEV"

echo "$FWD" | grep -q "br0.*$DEFAULT_DEV" \
    && echo "  [OK] FORWARD br0 → $DEFAULT_DEV" \
    || echo "  [WARN] FORWARD rule missing: br0 → $DEFAULT_DEV"

echo "$FWD" | grep -q "$DEFAULT_DEV.*br0" \
    && echo "  [OK] FORWARD $DEFAULT_DEV → br0 (ESTABLISHED)" \
    || echo "  [WARN] FORWARD rule missing: $DEFAULT_DEV → br0"

if [ "$DEFAULT_DEV" = "wlan0" ]; then
    DNAT=$(adb shell iptables -t nat -L PREROUTING -n 2>/dev/null | tr -d '\r')
    echo "$DNAT" | grep -q "dpt:22"    && echo "  [OK] Port-forward 22/tcp  → Pi" || echo "  [WARN] Port-forward 22/tcp missing"
    echo "$DNAT" | grep -q "dpt:14550" && echo "  [OK] Port-forward 14550/udp → Pi" || echo "  [WARN] Port-forward 14550/udp missing"
    echo "$DNAT" | grep -q "dpt:5600"  && echo "  [OK] Port-forward 5600/udp  → Pi" || echo "  [WARN] Port-forward 5600/udp missing"
fi

echo ""
echo "Modem verification complete."
