#pragma once

#include "lte_monitor.hpp"
#include "config.hpp"

// Monitors a USB LTE modem that enumerates as usb0 (e.g. Qualcomm 05c6:90b6).
// Presence check: reads /sys/class/net/usb0/flags for IFF_UP (pure sysfs, no deps).
// Signal quality: raw TCP HTTP to the modem's management API at 192.168.100.1.
// The modem API requires no authentication — funcNo=1001 works without prior login.
class UsbLteMonitor : public ILteMonitor {
public:
    void update() override;
    const LteStatus& status() const override { return status_; }

private:
    bool check_interface();
    bool fetch_signal();

    // Send a raw HTTP POST and collect the full response.
    // Returns false on any socket error.
    bool http_post(const char* body, const char* extra_headers,
                   char* resp_buf, int resp_buf_size);

    LteStatus status_{};
};
