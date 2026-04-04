#!/system/bin/sh
# LED connectivity status monitor
# Green solid  = LTE data up (rmnet0 has carrier IP + DNS)
# Red blinking = LTE data down / connecting (1s on, 1s off)
# wifi LED     = off (reserved for future use)

LED_RED=/sys/class/leds/red/brightness
LED_GREEN=/sys/class/leds/green/brightness
LED_WIFI=/sys/class/leds/wifi/brightness

PID_FILE=/data/logs/led_status.pid
LOG=/data/logs/led_status.log

log() { echo "$(getprop ro.build.date.utc) $$ $1" >> $LOG; }

# Singleton: kill any previous instance
if [ -f $PID_FILE ]; then
    kill $(cat $PID_FILE) 2>/dev/null
    rm -f $PID_FILE
fi
echo $$ > $PID_FILE

log "started"

led_on()  { echo 255 > $1; }
led_off() { echo 0   > $1; }

led_off $LED_WIFI

iter=0
while true; do
    iter=$((iter + 1))
    # LTE data is up if rmnet0 has a carrier-assigned DNS
    DNS=$(getprop net.rmnet0.dns1)
    if [ -n "$DNS" ]; then
        # LTE up — green solid, red off
        led_on  $LED_GREEN
        led_off $LED_RED
        [ $iter -le 3 ] && log "iter=$iter LTE_UP dns=$DNS"
        sleep 3
    else
        # LTE down / connecting — red blink (1s on, 1s off)
        led_off $LED_GREEN
        led_on  $LED_RED
        [ $iter -le 3 ] && log "iter=$iter LTE_DOWN"
        sleep 1
        led_off $LED_RED
        sleep 1
    fi
done
