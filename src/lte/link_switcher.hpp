#pragma once

// Sends a one-shot uplink switch command to the modem's link_switch_srv.sh (port 8081).
// Does not wait for the switch to complete — the result is observable via the next
// UsbLteMonitor::update() poll (port 8080, every 5 s).
class LinkSwitcher {
public:
    // cmd must be "wifi" or "lte". Returns true if the command was sent to the modem.
    bool send(const char* cmd);
};
