#!/system/bin/sh
# LTE status TCP server — serves one status line per connection on port 8080.
# Pi daemon connects to 192.168.100.1:8080 every 5s to read LTE signal/status.
#
# Status line format (key=value space-separated):
#   connected=1 rssi=15 netmode=LTE oper=KYIVSTAR
#
# Called from /system/etc/nat_forward.sh on boot_completed.

PORT=8080
PID_FILE=/data/logs/lte_status_srv.pid
LOG=/data/logs/lte_status_srv.log
OIFS=$IFS

log() { echo "$(date) $1" >> $LOG; }

# Singleton: kill any previous instance
if [ -f $PID_FILE ]; then
    kill $(cat $PID_FILE) 2>/dev/null
    rm -f $PID_FILE
fi
echo $$ > $PID_FILE

log "started pid=$$"

# BusyBox nc is used explicitly (Android toolbox nc is absent on this modem).
# BusyBox syntax: nc -l -p PORT
NC_CMD="busybox nc -l -p $PORT"
log "nc cmd: $NC_CMD"

# --- Main loop ---
while true; do
    # LTE connected if rmnet0 has a carrier-assigned DNS
    DNS=$(getprop net.rmnet0.dns1)
    if [ -n "$DNS" ]; then
        CONNECTED=1
    else
        CONNECTED=0
    fi

    # Active uplink: which interface is the current default route (rmnet0 or wlan0)?
    # Parse with shell builtins only (Android 4.4 shell lacks head/cut/awk/sed).
    DEFAULT_DEV=
    ROUTE=$(ip route 2>/dev/null | grep "^default ")
    set -- $ROUTE
    while [ $# -gt 0 ]; do
        [ "$1" = "dev" ] && { DEFAULT_DEV=$2; break; }
        shift
    done
    case "$DEFAULT_DEV" in
        wlan0)  UPLINK=wifi ;;
        rmnet0) UPLINK=lte  ;;
        *)      UPLINK=unknown ;;
    esac

    # Operator and network type from Android properties
    OPER=$(getprop gsm.operator.alpha)
    NETMODE=$(getprop gsm.network.type)
    [ -z "$OPER" ]    && OPER=unknown
    [ -z "$NETMODE" ] && NETMODE=unknown

    # RSSI from telephony registry.
    # SignalStrength format: gsm_ss gsm_ber cdma_dbm cdma_ecio evdo_dbm evdo_ecio evdo_snr lte_ss lte_rsrp ...
    # Field 1 = GSM signal (99=unknown), field 8 = LTE signal strength (0-31 scale, -1=unknown).
    # Prefer LTE field; fall back to GSM if LTE unavailable.
    RSSI=0
    while read line; do
        case $line in
            *mSignalStrength=SignalStrength:*)
                after=${line#*SignalStrength: }
                IFS=' '
                set -- $after
                IFS=$OIFS
                GSM_SS=$1
                LTE_SS=$8
                if [ -n "$LTE_SS" ] && [ "$LTE_SS" -ge 0 ] && [ "$LTE_SS" -le 31 ] 2>/dev/null; then
                    RSSI=$LTE_SS
                elif [ -n "$GSM_SS" ] && [ "$GSM_SS" -ne 99 ] 2>/dev/null; then
                    RSSI=$GSM_SS
                fi
                ;;
        esac
    done << DUMPSYS
$(dumpsys telephony.registry 2>/dev/null)
DUMPSYS

    STATUS="connected=$CONNECTED rssi=$RSSI netmode=$NETMODE oper=$OPER uplink=$UPLINK"

    # Serve status line — nc exits after one connection, then loop restarts listener
    echo "$STATUS" | $NC_CMD
done
