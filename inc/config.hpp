#pragma once

#include <cstdint>

namespace Config {
    constexpr uint8_t  MAV_SYS_ID           = 1;
    constexpr uint8_t  MAV_COMP_ID          = 1;
    constexpr uint16_t UDP_BIND_PORT        = 14550;
    constexpr uint64_t HEARTBEAT_INTERVAL_US = 1'000'000; // 1 Hz
    constexpr unsigned LOOP_SLEEP_US        = 1'000;      // 1 ms

    // Differential drive
    constexpr int16_t  DRIVE_DEAD_ZONE      = 50;   // axis values within [-N, N] treated as 0
    constexpr int16_t  DRIVE_AXIS_MAX       = 1000;
    constexpr uint16_t DRIVE_PWM_CENTER     = 1500; // µs — motor stopped
    constexpr uint16_t DRIVE_PWM_HALF_RANGE = 400;  // µs — added/subtracted at full stick (range: 1100..1900)
    constexpr uint32_t DRIVE_SLEW_TIME_MS   = 500; // ms to ramp from 0 to full scale

    // Stub telemetry values
    constexpr uint16_t DUMMY_LOAD_PERMILLE  = 500;  // stub: 50.0% CPU load
    constexpr uint16_t DUMMY_BATTERY_MV     = 12000; // stub: 12 V
    constexpr int16_t  DUMMY_CURRENT_CA     = 0;     // stub: 0 A
    constexpr int8_t   DUMMY_BATTERY_PCT    = 80;    // stub: 80%
}
