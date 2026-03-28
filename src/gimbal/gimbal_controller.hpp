#pragma once

#include <cstdint>

class IGimbalController {
public:
    virtual ~IGimbalController() = default;
    virtual void set(int16_t pan, int16_t tilt) = 0;
    virtual void center() = 0;
};
