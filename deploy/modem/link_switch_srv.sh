#!/system/bin/sh
# Link switch command server — receives "wifi" or "lte" commands on TCP port 8081.
# Pi daemon connects, sends a one-line command, and closes the connection.
# Applies the corresponding uplink switch asynchronously; result is visible via
# lte_status_srv.sh (port 8080) on the next poll.
#
# Called from /system/etc/nat_forward.sh on boot_completed.
#
# Notes on this WCNSS chip:
#  - `pidof` is unreliable: misses wpa_supplicant running under user=wifi. Use
#    `busybox killall` instead.
#  - `wpa_cli` is unreliable: the ctrl_interface socket file (wlan0) sometimes
#    disappears from /data/misc/wifi/sockets/ even while wpa_supplicant is alive,
#    making wpa_cli commands fail. We use /sys/class/net/wlan0/carrier to check
#    association status instead.
#  - Killing wpa_supplicant leaves the chip firmware in a stuck state that
#    prevents a new wpa_supplicant from associating. A full module reload
#    (rmmod + insmod) is required to reset it.

PORT=8081
LOG=/data/logs/link_switch_srv.log
PID_FILE=/data/logs/link_switch_srv.pid

BRIDGE=br0
WIFI_IFACE=wlan0
WIFI_CONF=/data/misc/wifi/rover_wpa.conf
WIFI_SOCKETS=/data/misc/wifi/sockets

log() { echo "$(date) $1" >> $LOG; }

# Singleton: kill any previous instance
if [ -f $PID_FILE ]; then
    kill $(cat $PID_FILE) 2>/dev/null
    rm -f $PID_FILE
fi
echo $$ > $PID_FILE

log "started pid=$$"

# ---------------------------------------------------------------------------
# Switch to LTE uplink
# Kills wpa_supplicant/dhcpcd, flushes wlan0 IP, sets up LTE NAT, restores route.
# Fast (~2s). Safe to call from WiFi mode or when already on LTE.
# ---------------------------------------------------------------------------

do_switch_lte() {
    log "switch_lte: start"

    # Kill ALL wpa_supplicant instances (pidof is unreliable here — it misses
    # processes running as user=wifi even when our script runs as root).
    if busybox killall wpa_supplicant 2>/dev/null; then
        sleep 1
        log "switch_lte: killed wpa_supplicant (killall)"
    fi

    if busybox killall dhcpcd 2>/dev/null; then
        sleep 1
        log "switch_lte: killed dhcpcd (killall)"
    fi

    ip addr flush dev $WIFI_IFACE 2>/dev/null
    log "switch_lte: flushed wlan0 addresses"

    # Flush all FORWARD and WiFi NAT rules, then re-apply LTE rules
    iptables -F natctrl_FORWARD 2>/dev/null
    iptables -t nat -F PREROUTING 2>/dev/null
    iptables -t nat -D natctrl_nat_POSTROUTING -o $WIFI_IFACE -j MASQUERADE 2>/dev/null
    # Also remove the POSTROUTING masquerade added for port-forwarding asymmetric routing
    iptables -t nat -D POSTROUTING -o $BRIDGE -j MASQUERADE 2>/dev/null

    echo 1 > /proc/sys/net/ipv4/conf/rmnet0/forwarding
    iptables -t nat -D natctrl_nat_POSTROUTING -o rmnet0 -j MASQUERADE 2>/dev/null
    iptables -t nat -A natctrl_nat_POSTROUTING -o rmnet0 -j MASQUERADE
    iptables -A natctrl_FORWARD -i $BRIDGE -o rmnet0 -j ACCEPT
    iptables -A natctrl_FORWARD -i rmnet0 -o $BRIDGE -m state --state ESTABLISHED,RELATED -j ACCEPT
    log "switch_lte: LTE iptables applied"

    # Restore LTE default route if not already present
    if ! ip route 2>/dev/null | grep -q "default.*rmnet0"; then
        LTE_GW=$(getprop net.rmnet0.gw 2>/dev/null)
        if [ -n "$LTE_GW" ] && [ "$LTE_GW" != "0.0.0.0" ]; then
            ip route add default via $LTE_GW dev rmnet0 2>/dev/null
            log "switch_lte: restored default route via $LTE_GW dev rmnet0"
        else
            log "switch_lte: warning — net.rmnet0.gw not set, route not restored"
        fi
    else
        log "switch_lte: LTE default route already present"
    fi

    log "switch_lte: done"
}

# ---------------------------------------------------------------------------
# Switch to WiFi uplink
# Full module reload + fresh wpa_supplicant start. Uses /sys/class/net/wlan0/carrier
# to detect association (wpa_cli is not reliable on this device).
# Slow (~15-45s). Falls back to LTE automatically on failure.
# ---------------------------------------------------------------------------

do_switch_wifi() {
    log "switch_wifi: start"

    if [ ! -f "$WIFI_CONF" ]; then
        log "switch_wifi: no config at $WIFI_CONF — cannot switch, staying on LTE"
        return 1
    fi

    # Kill all stale processes reliably (pidof misses user=wifi processes)
    if busybox killall wpa_supplicant 2>/dev/null; then
        sleep 1
        log "switch_wifi: killed stale wpa_supplicant"
    fi
    if busybox killall dhcpcd 2>/dev/null; then
        sleep 1
        log "switch_wifi: killed stale dhcpcd"
    fi

    # Full module reload to reset WCNSS chip state. Without this, a new
    # wpa_supplicant can't associate even though the driver accepts it.
    if grep -q "^wlan " /proc/modules; then
        log "switch_wifi: unloading wlan module"
        rmmod wlan 2>/dev/null
        sleep 2
        if grep -q "^wlan " /proc/modules; then
            log "switch_wifi: rmmod failed (module still loaded) — LTE fallback"
            do_switch_lte
            return 1
        fi
    fi
    insmod /system/lib/modules/wlan.ko 2>/dev/null
    log "switch_wifi: module loaded, waiting for $WIFI_IFACE"
    MOD_WAIT=0
    while [ $MOD_WAIT -lt 15 ]; do
        ip link show $WIFI_IFACE >/dev/null 2>&1 && break
        sleep 1; MOD_WAIT=$((MOD_WAIT + 1))
    done
    log "switch_wifi: $WIFI_IFACE appeared after ${MOD_WAIT}s"
    sleep 5  # WCNSS firmware settle time

    # Clean stale sockets
    mkdir -p $WIFI_SOCKETS; chmod 777 $WIFI_SOCKETS
    rm -f $WIFI_SOCKETS/$WIFI_IFACE $WIFI_SOCKETS/p2p0 2>/dev/null

    wpa_supplicant -i $WIFI_IFACE -D nl80211 \
        -c $WIFI_CONF -O $WIFI_SOCKETS \
        > /data/logs/wpa_supplicant.log 2>&1 &
    log "switch_wifi: wpa_supplicant started pid=$!"
    sleep 3

    if ! ip link show $WIFI_IFACE >/dev/null 2>&1; then
        log "switch_wifi: wlan0 disappeared after wpa_supplicant start — LTE fallback"
        busybox killall wpa_supplicant 2>/dev/null
        do_switch_lte
        return 1
    fi

    # Wait for association using kernel carrier flag (wpa_cli unreliable here).
    # carrier=1 means wlan0 has a link-layer association to an AP.
    WIFI_WAIT=0; ASSOCIATED=0
    while [ $WIFI_WAIT -lt 45 ]; do
        CARRIER=$(cat /sys/class/net/$WIFI_IFACE/carrier 2>/dev/null)
        if [ "$CARRIER" = "1" ]; then
            ASSOCIATED=1
            break
        fi
        sleep 1; WIFI_WAIT=$((WIFI_WAIT + 1))
    done

    if [ $ASSOCIATED -eq 0 ]; then
        log "switch_wifi: no carrier after ${WIFI_WAIT}s — LTE fallback"
        busybox killall wpa_supplicant 2>/dev/null
        do_switch_lte
        return 1
    fi
    log "switch_wifi: associated (carrier up) after ${WIFI_WAIT}s"

    # Remove LTE default route before dhcpcd — DHCP Discover packets must exit
    # via wlan0 to reach the home router. With rmnet0 as default, they'd go out
    # via LTE and never get a response. do_switch_lte restores the route on fallback.
    LTE_ROUTE=$(ip route 2>/dev/null | grep "default.*rmnet0" || echo "")
    if [ -n "$LTE_ROUTE" ]; then
        set -- $LTE_ROUTE; LTE_GW_TMP=$3
        ip route del default via $LTE_GW_TMP dev rmnet0 2>/dev/null
        log "switch_wifi: removed LTE route before dhcpcd"
    fi

    dhcpcd -h pi-rover $WIFI_IFACE 2>/dev/null
    sleep 5

    if ! ip route 2>/dev/null | grep -q "default.*$WIFI_IFACE"; then
        log "switch_wifi: dhcpcd failed (no default route) — LTE fallback"
        busybox killall dhcpcd 2>/dev/null
        busybox killall wpa_supplicant 2>/dev/null
        do_switch_lte
        return 1
    fi
    log "switch_wifi: IP obtained — updating iptables"

    # Flush FORWARD, remove LTE NAT, apply WiFi NAT + port forwarding
    iptables -F natctrl_FORWARD 2>/dev/null
    iptables -t nat -D natctrl_nat_POSTROUTING -o rmnet0 -j MASQUERADE 2>/dev/null

    echo 1 > /proc/sys/net/ipv4/conf/$WIFI_IFACE/forwarding
    iptables -t nat -D natctrl_nat_POSTROUTING -o $WIFI_IFACE -j MASQUERADE 2>/dev/null
    iptables -t nat -A natctrl_nat_POSTROUTING -o $WIFI_IFACE -j MASQUERADE
    iptables -A natctrl_FORWARD -i $BRIDGE -o $WIFI_IFACE -j ACCEPT
    iptables -A natctrl_FORWARD -i $WIFI_IFACE -o $BRIDGE -m state --state ESTABLISHED,RELATED -j ACCEPT

    iptables -t nat -F PREROUTING 2>/dev/null
    PI=192.168.100.100
    iptables -t nat -A PREROUTING -i $WIFI_IFACE -p tcp --dport 22    -j DNAT --to-destination $PI:22
    iptables -t nat -A PREROUTING -i $WIFI_IFACE -p udp --dport 14550 -j DNAT --to-destination $PI:14550
    iptables -t nat -A PREROUTING -i $WIFI_IFACE -p udp --dport 5600  -j DNAT --to-destination $PI:5600
    iptables -t nat -A POSTROUTING -o $BRIDGE -j MASQUERADE
    iptables -A natctrl_FORWARD -i $WIFI_IFACE -o $BRIDGE -d $PI -p tcp --dport 22    -j ACCEPT
    iptables -A natctrl_FORWARD -i $WIFI_IFACE -o $BRIDGE -d $PI -p udp --dport 14550 -j ACCEPT
    iptables -A natctrl_FORWARD -i $WIFI_IFACE -o $BRIDGE -d $PI -p udp --dport 5600  -j ACCEPT

    # Ensure no LTE default route remains
    LTE_ROUTE=$(ip route 2>/dev/null | grep "default.*rmnet0" || echo "")
    if [ -n "$LTE_ROUTE" ]; then
        set -- $LTE_ROUTE; LTE_GW=$3
        ip route del default via $LTE_GW dev rmnet0 2>/dev/null
        log "switch_wifi: removed stale LTE default route via $LTE_GW"
    fi

    log "switch_wifi: done"
}

# ---------------------------------------------------------------------------
# Main loop — accept one command per connection, execute asynchronously
# ---------------------------------------------------------------------------

SWITCH_PID=

while true; do
    CMD=$(busybox nc -l -p $PORT)

    # Cancel any in-progress switch before starting the new one
    if [ -n "$SWITCH_PID" ] && kill -0 "$SWITCH_PID" 2>/dev/null; then
        log "cmd: '$CMD' — cancelling in-progress switch pid=$SWITCH_PID"
        kill "$SWITCH_PID" 2>/dev/null
        wait "$SWITCH_PID" 2>/dev/null
    fi

    log "cmd: '$CMD'"
    case "$CMD" in
        wifi) do_switch_wifi & SWITCH_PID=$! ;;
        lte)  do_switch_lte  & SWITCH_PID=$! ;;
        *)    log "unknown command: '$CMD'" ;;
    esac
done
