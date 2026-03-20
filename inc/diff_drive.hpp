#pragma once

#include <cstdint>
#include <algorithm>

#include "config.hpp"

struct DriveOutput {
    int16_t left;
    int16_t right;
};

static inline int16_t apply_dead_zone(int16_t v)
{
    return (v > -Config::DRIVE_DEAD_ZONE && v < Config::DRIVE_DEAD_ZONE) ? 0 : v;
}

static inline int16_t clamp_axis(int32_t v)
{
    return static_cast<int16_t>(
        std::max<int32_t>(-Config::DRIVE_AXIS_MAX,
        std::min<int32_t>( Config::DRIVE_AXIS_MAX, v)));
}

struct DriveSlew {
    DriveOutput current{0, 0};

    // Call each time a new target is known; elapsed_ms = time since last call.
    // Returns the slew-limited output (also stored in current).
    DriveOutput step(DriveOutput target, uint32_t elapsed_ms)
    {
        int32_t max_delta = static_cast<int32_t>(Config::DRIVE_AXIS_MAX)
                          * static_cast<int32_t>(elapsed_ms)
                          / static_cast<int32_t>(Config::DRIVE_SLEW_TIME_MS);
        current.left  = clamp_axis(current.left  + std::clamp(
            static_cast<int32_t>(target.left)  - current.left,  -max_delta, max_delta));
        current.right = clamp_axis(current.right + std::clamp(
            static_cast<int32_t>(target.right) - current.right, -max_delta, max_delta));
        return current;
    }

    void reset() { current = {0, 0}; }
};

// z: forward(+1000) / backward(-1000)
// y: right(+1000)   / left(-1000)
static inline DriveOutput compute_diff_drive(int16_t z, int16_t y)
{
    z = apply_dead_zone(z);
    y = apply_dead_zone(y);
    return { clamp_axis(z + y), clamp_axis(z - y) };
}
