#!/system/bin/sh
# RNDIS tethering setup (MifiService WiFi hotspot disabled via pm disable)
# Called from /system/etc/init.qcom.post_boot.sh on boot_completed
#
# Uplink selection:
#   - If /data/misc/wifi/wpa_supplicant.conf exists: connect wlan0 as client, NAT via wlan0
#   - Otherwise: NAT via rmnet0 (LTE, original behaviour)

LOG=/data/logs/nat_forward.log
log() { echo "$(date) $1" >> $LOG; }

log "=== nat_forward.sh start ==="

BRIDGE=br0
IFACE=rndis0
GW=192.168.100.1
WIFI_IFACE=wlan0
WIFI_CONF=/data/misc/wifi/rover_wpa.conf
WIFI_SOCKETS=/data/misc/wifi/sockets

# --- Bridge setup ---
log "bridge setup"
brctl addbr $BRIDGE 2>/dev/null
brctl addif $BRIDGE $IFACE 2>/dev/null
ip addr flush dev $BRIDGE 2>/dev/null
ip addr add $GW/24 dev $BRIDGE
ip link set $IFACE up
ip link set $BRIDGE up
log "bridge done"

# --- IP forwarding ---
log "ip forwarding"
echo 1 > /proc/sys/net/ipv4/ip_forward
echo 1 > /proc/sys/net/ipv4/conf/br0/forwarding
echo 1 > /proc/sys/net/ipv4/conf/rmnet0/forwarding

# --- WiFi client uplink (preferred if config exists) ---
WIFI_UPLINK=0

if [ -f "$WIFI_CONF" ]; then
    log "wifi: config found at $WIFI_CONF"

    # Load WiFi kernel module if not already loaded, then wait for wlan0 to appear
    if ! grep -q "^wlan " /proc/modules; then
        insmod /system/lib/modules/wlan.ko 2>/dev/null
        log "wifi: module loaded, waiting for $WIFI_IFACE"
        MOD_WAIT=0
        while [ $MOD_WAIT -lt 15 ]; do
            ip link show $WIFI_IFACE >/dev/null 2>&1 && break
            sleep 1
            MOD_WAIT=$((MOD_WAIT + 1))
        done
        log "wifi: $WIFI_IFACE appeared after ${MOD_WAIT}s"
        # Extra settle time for WCNSS firmware to finish initializing
        sleep 5
    fi
    # Do NOT set interface UP here — wpa_supplicant manages interface state.
    # Manually running 'ip link set wlan0 up' puts the interface into NO-CARRIER
    # state on this WCNSS chip, which prevents wpa_supplicant from initializing.

    # Kill any stale wpa_supplicant
    STALE_PID=$(pidof wpa_supplicant 2>/dev/null)
    [ -n "$STALE_PID" ] && kill $STALE_PID 2>/dev/null && sleep 1

    mkdir -p $WIFI_SOCKETS
    chmod 777 $WIFI_SOCKETS
    # Remove stale socket files (/data persists across reboots; old sockets block wpa_supplicant)
    rm -f $WIFI_SOCKETS/$WIFI_IFACE $WIFI_SOCKETS/p2p0 2>/dev/null
    wpa_supplicant -i $WIFI_IFACE -D nl80211 \
        -c $WIFI_CONF -O $WIFI_SOCKETS \
        > /data/logs/wpa_supplicant.log 2>&1 &
    WPA_BGPID=$!
    log "wifi: wpa_supplicant started pid=$WPA_BGPID"
    sleep 3

    # Fast-fail if wpa_supplicant exited immediately (no socket = init failure)
    if ! ip link show $WIFI_IFACE >/dev/null 2>&1 || \
       ! ls $WIFI_SOCKETS/$WIFI_IFACE >/dev/null 2>&1; then
        log "wifi: wpa_supplicant failed to initialize (no socket), falling back to LTE"
        kill $WPA_BGPID 2>/dev/null
        WIFI_UPLINK=0
    else

    # Wait for association (up to 90s — cold WCNSS boot can take 60+ seconds to scan/associate)
    # Use grep to check for COMPLETED directly — no tr/cut needed
    WIFI_WAIT=0
    STATE=""
    while [ $WIFI_WAIT -lt 45 ]; do
        wpa_cli -p $WIFI_SOCKETS -i $WIFI_IFACE status 2>/dev/null \
            | grep -q "wpa_state=COMPLETED" && STATE="COMPLETED" && break
        sleep 1
        WIFI_WAIT=$((WIFI_WAIT + 1))
    done

    if [ "$STATE" = "COMPLETED" ]; then
        log "wifi: associated after ${WIFI_WAIT}s"

        # Kill stale dhcpcd
        DHCP_PID=$(pidof dhcpcd 2>/dev/null)
        [ -n "$DHCP_PID" ] && kill $DHCP_PID 2>/dev/null && sleep 1

        dhcpcd $WIFI_IFACE 2>/dev/null
        sleep 3

        # Check if dhcpcd set a default route via wlan0 (any default route on wlan0 = success)
        if ip route 2>/dev/null | grep -q "default.*$WIFI_IFACE"; then
            WIFI_GW=$(ip route 2>/dev/null | grep "default.*$WIFI_IFACE")
            log "wifi: IP obtained, route: $WIFI_GW — using WiFi as uplink"
            WIFI_UPLINK=1
        else
            log "wifi: dhcpcd failed, falling back to LTE"
            DHCP_PID=$(pidof dhcpcd 2>/dev/null)
            [ -n "$DHCP_PID" ] && kill $DHCP_PID 2>/dev/null
            kill $WPA_BGPID 2>/dev/null
        fi
    else
        log "wifi: association timeout after ${WIFI_WAIT}s, falling back to LTE"
        kill $WPA_BGPID 2>/dev/null
    fi
    fi  # end fast-fail block
else
    log "wifi: no config at $WIFI_CONF, using LTE"
fi

# --- NAT forwarding ---
log "iptables (uplink=$([ $WIFI_UPLINK -eq 1 ] && echo wifi || echo lte))"
iptables -F natctrl_FORWARD

if [ $WIFI_UPLINK -eq 1 ]; then
    # WiFi uplink: NAT via wlan0; delete LTE default route so WiFi's is used
    echo 1 > /proc/sys/net/ipv4/conf/$WIFI_IFACE/forwarding
    iptables -t nat -D natctrl_nat_POSTROUTING -o $WIFI_IFACE -j MASQUERADE 2>/dev/null
    iptables -t nat -A natctrl_nat_POSTROUTING -o $WIFI_IFACE -j MASQUERADE
    iptables -A natctrl_FORWARD -i $BRIDGE -o $WIFI_IFACE -j ACCEPT
    iptables -A natctrl_FORWARD -i $WIFI_IFACE -o $BRIDGE -m state --state ESTABLISHED,RELATED -j ACCEPT
    # Port forwarding: expose Pi (192.168.100.100) services to home network via wlan0
    # Clear any stale DNAT rules first
    iptables -t nat -F PREROUTING 2>/dev/null
    PI=192.168.100.100
    iptables -t nat -A PREROUTING -i $WIFI_IFACE -p tcp --dport 22    -j DNAT --to-destination $PI:22
    iptables -t nat -A PREROUTING -i $WIFI_IFACE -p udp --dport 14550 -j DNAT --to-destination $PI:14550
    iptables -t nat -A PREROUTING -i $WIFI_IFACE -p udp --dport 5600  -j DNAT --to-destination $PI:5600
    # Masquerade source of port-forwarded traffic on the br0 side so the Pi always
    # replies via usb0 -> modem (avoids asymmetric routing when Pi also has wlan0)
    iptables -t nat -A POSTROUTING -o $BRIDGE -j MASQUERADE
    # Allow new connections to forwarded ports through the bridge
    iptables -A natctrl_FORWARD -i $WIFI_IFACE -o $BRIDGE -d $PI -p tcp --dport 22    -j ACCEPT
    iptables -A natctrl_FORWARD -i $WIFI_IFACE -o $BRIDGE -d $PI -p udp --dport 14550 -j ACCEPT
    iptables -A natctrl_FORWARD -i $WIFI_IFACE -o $BRIDGE -d $PI -p udp --dport 5600  -j ACCEPT
    # Remove LTE default route so WiFi (set by dhcpcd) is the only default
    # Parse "default via X.X.X.X dev rmnet0" with shell word splitting only
    LTE_ROUTE=$(ip route 2>/dev/null | grep "default.*rmnet0" || echo "")
    if [ -n "$LTE_ROUTE" ]; then
        set -- $LTE_ROUTE          # splits into: default via X.X.X.X dev rmnet0 ...
        LTE_GW=$3                  # third word is the gateway IP
        ip route del default via $LTE_GW dev rmnet0 2>/dev/null
    fi
    log "iptables: WiFi NAT via $WIFI_IFACE, port-fwd 22/tcp 14550/udp 5600/udp -> $PI, LTE default route removed"
else
    # LTE uplink (original behaviour)
    iptables -t nat -D natctrl_nat_POSTROUTING -o rmnet0 -j MASQUERADE 2>/dev/null
    iptables -t nat -A natctrl_nat_POSTROUTING -o rmnet0 -j MASQUERADE
    iptables -A natctrl_FORWARD -i $BRIDGE -o rmnet0 -j ACCEPT
    iptables -A natctrl_FORWARD -i rmnet0 -o $BRIDGE -m state --state ESTABLISHED,RELATED -j ACCEPT
    log "iptables: LTE NAT via rmnet0"
fi

log "iptables done"

# --- LED status daemon ---
log "launching led_status.sh"
/system/etc/led_status.sh > /data/logs/led_status.log 2>&1 &
log "led_status launched pid=$!"

# --- LTE status TCP server (Pi daemon connects to port 8080 for signal/operator info) ---
log "launching lte_status_srv.sh"
/system/etc/lte_status_srv.sh > /data/logs/lte_status_srv.log 2>&1 &
log "lte_status_srv launched pid=$!"

log "=== nat_forward.sh done ==="
