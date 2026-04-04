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
