#!/usr/bin/env bash
# Write wpa_supplicant config to the UZ801 modem for WiFi client mode.
# Run ON THE PI (adb must be available and modem connected).
#
# Usage:
#   WIFI_SSID=S_HOME WIFI_PSK=password bash setup-wifi-client.sh
#   bash setup-wifi-client.sh S_HOME password
#
# From dev machine:
#   make setup-modem-wifi WIFI_SSID=S_HOME WIFI_PSK=password
#
# The config is written to /data/misc/wifi/wpa_supplicant.conf on the modem.
# /data persists across reboots and firmware updates.
# nat_forward.sh reads this file at boot and connects as a WiFi client if present.
# To disable WiFi uplink: make remove-modem-wifi (or adb shell rm /data/misc/wifi/wpa_supplicant.conf)

set -e

WIFI_SSID="${1:-${WIFI_SSID}}"
WIFI_PSK="${2:-${WIFI_PSK}}"

die() { echo "ERROR: $1" >&2; exit 1; }

[ -z "$WIFI_SSID" ] && die "WIFI_SSID not set. Usage: WIFI_SSID=name WIFI_PSK=pass bash setup-wifi-client.sh"
[ -z "$WIFI_PSK"  ] && die "WIFI_PSK not set. Usage: WIFI_SSID=name WIFI_PSK=pass bash setup-wifi-client.sh"

# Check ADB
adb devices | grep -v "List of" | grep -q "device" || die "No ADB device found."

echo "==> Writing wpa_supplicant config for SSID: $WIFI_SSID"
adb shell "mkdir -p /data/misc/wifi/sockets && chmod 777 /data/misc/wifi /data/misc/wifi/sockets"

# Write config via a temp file pushed from the host (avoids adb shell escaping issues)
TMPFILE=$(mktemp /tmp/wpa_supplicant_modem.conf.XXXXXX)
cat > "$TMPFILE" <<EOF
ctrl_interface=/data/misc/wifi/sockets
ap_scan=1
fast_reauth=1

network={
    ssid="${WIFI_SSID}"
    psk="${WIFI_PSK}"
    key_mgmt=WPA-PSK
}
EOF

adb push "$TMPFILE" /data/misc/wifi/rover_wpa.conf
rm -f "$TMPFILE"
adb shell chmod 644 /data/misc/wifi/rover_wpa.conf

echo "==> Verifying"
SSID_CHECK=$(adb shell grep "ssid=" /data/misc/wifi/rover_wpa.conf 2>/dev/null | tr -d '\r' || echo "")
[ -n "$SSID_CHECK" ] && echo "  [OK] Config written: $SSID_CHECK" || die "Config write failed"

echo ""
echo "WiFi client config written. Reboot modem to activate:"
echo "  adb reboot"
echo "Or re-run nat_forward.sh if modem is already booted:"
echo "  adb shell /system/etc/nat_forward.sh"
echo ""
echo "To remove (revert to LTE-only):"
echo "  adb shell rm /data/misc/wifi/rover_wpa.conf && adb reboot"
