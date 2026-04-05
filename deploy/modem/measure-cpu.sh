#!/usr/bin/env bash
# Measure CPU usage on the UZ801 modem via ADB.
# Pushes measure-cpu-inner.sh to the modem and runs it there.
#
# Usage (run from Pi where adb is available):
#   bash measure-cpu.sh [INTERVAL_SECONDS]
#
# Examples:
#   bash measure-cpu.sh        # 5s window (default)
#   bash measure-cpu.sh 10     # 10s window

INTERVAL=${1:-5}
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

adb devices | grep -v "List of" | grep -q "device" \
    || { echo "ERROR: No ADB device found." >&2; exit 1; }

adb push "$SCRIPT_DIR/measure-cpu-inner.sh" /data/logs/_measure_cpu.sh >/dev/null
adb shell chmod 755 /data/logs/_measure_cpu.sh
adb shell sh /data/logs/_measure_cpu.sh "$INTERVAL"
