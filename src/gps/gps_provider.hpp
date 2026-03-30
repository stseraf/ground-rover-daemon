#pragma once

#include "gps_fix.hpp"

class IGpsProvider {
public:
    virtual ~IGpsProvider() = default;

    // Call every main-loop iteration; reads hardware and updates internal fix.
    virtual void update() = 0;

    virtual const GpsFix& fix() const = 0;
};
