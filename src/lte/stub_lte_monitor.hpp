#pragma once

#include "lte_monitor.hpp"

// No-op monitor for host/dev builds — always reports modem absent.
class StubLteMonitor : public ILteMonitor {
public:
    void update() override {}
    const LteStatus& status() const override { return status_; }
private:
    LteStatus status_{};
};
