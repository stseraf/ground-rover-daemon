#pragma once

#include "lte_monitor.hpp"
#include "config.hpp"

// Monitors a USB LTE modem that enumerates as usb0 (e.g. Qualcomm 05c6:90b6).
// Presence check: reads /sys/class/net/usb0/flags for IFF_UP (pure sysfs, no deps).
// Signal quality: connects to lte_status_srv.sh TCP daemon on modem at STATUS_HOST:STATUS_PORT.
// Traffic stats: reads /proc/net/dev for usb0 RX/TX byte counters.
class UsbLteMonitor : public ILteMonitor {
public:
    void update() override;
    const LteStatus& status() const override { return status_; }

private:
    bool check_interface();
    bool fetch_signal();
    bool fetch_traffic();

    LteStatus status_{};
    bool      was_signal_low_ = false; // edge-detect low-signal warnings on active uplink
};
