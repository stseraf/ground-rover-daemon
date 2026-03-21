#pragma once

#include <cstdint>

class IMotorDriver {
public:
    virtual ~IMotorDriver() = default;
    // left/right in [-1000, 1000]; 0 = stop (coast)
    virtual void set(int16_t left, int16_t right) = 0;
    virtual void stop() = 0;
};
