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

step "Pushing lte_status_srv.sh"
adb push "$SCRIPT_DIR/lte_status_srv.sh" /system/etc/lte_status_srv.sh
adb shell chmod 755 /system/etc/lte_status_srv.sh

step "Pushing link_switch_srv.sh"
adb push "$SCRIPT_DIR/link_switch_srv.sh" /system/etc/link_switch_srv.sh
adb shell chmod 755 /system/etc/link_switch_srv.sh

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
# Use timeout: adb reboot hangs when the modem USB disconnects mid-reboot.
# SSH session may also drop if the Pi routes traffic through the modem — so we
# exit immediately here. Run 'make verify-modem' once the modem is back up.
timeout 10 adb reboot || true
echo ""
echo "Modem rebooting. Run 'make verify-modem' in ~60s to confirm boot."
