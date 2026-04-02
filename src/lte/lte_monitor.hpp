#pragma once

#include "lte_status.hpp"

class ILteMonitor {
public:
    virtual ~ILteMonitor() = default;
    virtual void update() = 0;
    virtual const LteStatus& status() const = 0;
};
