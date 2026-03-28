#pragma once

#include <cstdint>

class IMotorDriver {
public:
    virtual ~IMotorDriver() = default;
    virtual void set(int16_t left, int16_t right) = 0;
    virtual void stop() = 0;
    virtual void engage() {}   // enable driver (STBY HIGH) — called on arm
    virtual void release() {}  // stop + disable driver (STBY LOW) — called on disarm
};
