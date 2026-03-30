#pragma once

#include "gps_provider.hpp"

class StubGpsProvider : public IGpsProvider {
public:
    void update() override {}
    const GpsFix& fix() const override { return fix_; }

private:
    GpsFix fix_{};
};
