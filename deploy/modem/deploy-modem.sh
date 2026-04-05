#!/usr/bin/env bash
# Deploy UZ801 modem configuration to a fresh/replacement dongle.
# Run this script ON THE PI (adb is on the Pi, modem connects via USB).
#
# From dev machine:  make deploy-modem
# From Pi directly:  bash /tmp/modem-deploy/deploy-modem.sh
#
# Prerequisites: modem connected to Pi via USB, adb available on Pi
#   adb devices   # should show the modem
#
# What this script does:
#   1. Disables MifiService (WiFi hotspot)
#   2. Creates /data/logs on the modem
#   3. Pushes nat_forward.sh and led_status.sh to /system/etc/
#   4. Appends the boot hook to init.qcom.post_boot.sh
#   5. Reboots and verifies

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

step() { echo ""; echo "==> $1"; }
die()  { echo "ERROR: $1" >&2; exit 1; }

# --- Check ADB ---
step "Checking ADB"
adb devices | grep -v "List of" | grep -q "device" || die "No ADB device found. Is the modem connected?"
echo "ADB device found."

# --- Restart as root ---
step "Restarting ADB as root"
adb root
sleep 2

# --- Disable MifiService ---
step "Disabling MifiService (WiFi hotspot)"
adb shell pm disable com.mifiservice.hello

# --- Remount /system ---
step "Remounting /system as writable"
adb remount

# --- Create log directory ---
step "Creating /data/logs"
adb shell "mkdir -p /data/logs && chmod 777 /data/logs"

# --- Push scripts ---
step "Pushing nat_forward.sh"
adb push "$SCRIPT_DIR/nat_forward.sh" /system/etc/nat_forward.sh
adb shell chmod 755 /system/etc/nat_forward.sh

step "Pushing led_status.sh"
adb push "$SCRIPT_DIR/led_status.sh" /system/etc/led_status.sh
adb shell chmod 755 /system/etc/led_status.sh

# --- Hook into boot ---
step "Hooking nat_forward.sh into init.qcom.post_boot.sh"
HOOK_MARKER="nat_forward.sh"
ALREADY=$(adb shell "grep -c $HOOK_MARKER /system/etc/init.qcom.post_boot.sh" 2>/dev/null | tr -d '\r' || echo 0)
if [ "$ALREADY" -gt 0 ]; then
    echo "  Hook already present, skipping."
else
    adb pull /system/etc/init.qcom.post_boot.sh /tmp/init.qcom.post_boot.sh
    printf '\n# Restore NAT forwarding rules for RNDIS tethering (MifiService WiFi hotspot disabled)\n/system/etc/nat_forward.sh\n' \
        >> /tmp/init.qcom.post_boot.sh
    adb push /tmp/init.qcom.post_boot.sh /system/etc/init.qcom.post_boot.sh
    rm -f /tmp/init.qcom.post_boot.sh
    echo "  Hook appended."
fi

# --- Reboot ---
step "Rebooting modem"
adb reboot
echo "Waiting for modem to come back up (up to 90s)..."
BOOT_TIMEOUT=90
ELAPSED=0
while [ $ELAPSED -lt $BOOT_TIMEOUT ]; do
    sleep 5
    ELAPSED=$((ELAPSED + 5))
    BOOT=$(adb shell getprop sys.boot_completed 2>/dev/null | tr -d '\r' || echo "")
    [ "$BOOT" = "1" ] && break
    echo "  ...${ELAPSED}s elapsed, waiting..."
done
[ "$BOOT" = "1" ] || die "Boot not complete after ${BOOT_TIMEOUT}s"

# --- Verify ---
step "Verifying"
echo "  [OK] Boot complete (${ELAPSED}s)"

MIFI=$(adb shell pm list packages -d 2>/dev/null | grep mifiservice || echo "")
[ -n "$MIFI" ] && echo "  [OK] MifiService disabled" || echo "  [WARN] Could not confirm MifiService state"

DNSMASQ=$(adb shell ps 2>/dev/null | grep dnsmasq || echo "")
[ -z "$DNSMASQ" ] && echo "  [OK] dnsmasq not running (static IP mode)" || echo "  [WARN] dnsmasq still running — stale process?"

BR0=$(adb shell ip addr show br0 2>/dev/null | grep "192.168.100.1" || echo "")
[ -n "$BR0" ] && echo "  [OK] br0 has correct IP" || die "br0 missing or wrong IP"

IP_FWD=$(adb shell cat /proc/sys/net/ipv4/ip_forward 2>/dev/null | tr -d '\r' || echo 0)
[ "$IP_FWD" = "1" ] && echo "  [OK] IP forwarding enabled" || die "IP forwarding not enabled"

GREEN=$(adb shell cat /sys/class/leds/green/brightness 2>/dev/null | tr -d '\r' || echo 0)
RED=$(adb shell cat /sys/class/leds/red/brightness 2>/dev/null | tr -d '\r' || echo 255)
[ "$GREEN" = "255" ] && [ "$RED" = "0" ] && echo "  [OK] LED: green (LTE up)" || echo "  [WARN] LED not green (green=$GREEN red=$RED) — LTE may still be connecting"

echo ""
echo "Modem deployment complete."
