#pragma once

#include "motor_driver.hpp"

// Stub UART motor driver — no-op until UART backend is implemented (Feature 3).
class UartMotorDriver : public IMotorDriver {
public:
    void set(int16_t /*left*/, int16_t /*right*/) override {}
    void stop() override {}
};
