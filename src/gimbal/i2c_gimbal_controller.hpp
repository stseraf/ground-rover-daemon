#pragma once

#include "gimbal_controller.hpp"

// Two-axis gimbal controller over I2C — RPi writes to Arduino acting as I2C slave.
//
// Protocol: 4-byte big-endian write per update
//   [pan_hi] [pan_lo] [tilt_hi] [tilt_lo]
//   pan/tilt: int16_t in range [-1000, +1000] (same scale as MANUAL_CONTROL axes)
//
// Arduino firmware must buffer bytes in Wire.onReceive() (ISR context) and
// call Servo.write() from loop(), not directly from the ISR.
class I2cGimbalController : public IGimbalController {
public:
    I2cGimbalController();
    ~I2cGimbalController() override;

    void set(int16_t pan, int16_t tilt) override;
    void center() override;

private:
    int fd_ = -1;

    void write_frame(int16_t pan, int16_t tilt);
};
