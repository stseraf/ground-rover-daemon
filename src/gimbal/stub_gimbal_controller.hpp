#pragma once

#include "gimbal_controller.hpp"

// No-op stub — used for host builds and GIMBAL=stub.
class StubGimbalController : public IGimbalController {
public:
    void set(int16_t, int16_t) override {}
    void center() override {}
};
