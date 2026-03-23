#pragma once

#include "motor_driver.hpp"

// No-op stub — used for host builds and DRIVER=stub.
class UartMotorDriver : public IMotorDriver {
public:
    void set(int16_t, int16_t) override {}
    void stop() override {}
};
