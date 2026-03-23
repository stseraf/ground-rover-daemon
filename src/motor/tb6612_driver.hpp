#pragma once

#include <cstdint>

#include "motor_driver.hpp"

// TB6612FNG dual H-bridge motor driver via /dev/gpiochip0 (GPIO v2 ioctl) + sysfs PWM.
//
// Pin mapping (BCM numbering set in config.hpp):
//   Motor A (left):  PWMA → pwmchip0/pwm0, AIN1, AIN2
//   Motor B (right): PWMB → pwmchip0/pwm1, BIN1, BIN2
//   STBY: held HIGH for driver lifetime; pulled LOW on destruction.
//
// Direction logic (value in [-1000, 1000]):
//   value > 0  → IN1=1, IN2=0  (forward)
//   value < 0  → IN1=0, IN2=1  (reverse)
//   value == 0 → IN1=0, IN2=0  (coast)
//
// Direction changes zero duty first to prevent H-bridge shoot-through.
class Tb6612Driver : public IMotorDriver {
public:
    Tb6612Driver();
    ~Tb6612Driver() override;

    Tb6612Driver(const Tb6612Driver&)            = delete;
    Tb6612Driver& operator=(const Tb6612Driver&) = delete;

    void set(int16_t left, int16_t right) override;
    void stop() override;

private:
    static constexpr unsigned IDX_AIN1 = 0;
    static constexpr unsigned IDX_AIN2 = 1;
    static constexpr unsigned IDX_BIN1 = 2;
    static constexpr unsigned IDX_BIN2 = 3;
    static constexpr unsigned IDX_STBY = 4;

    int chip_fd_ = -1;
    int line_fd_ = -1;
    int fd_pwma_ = -1;
    int fd_pwmb_ = -1;
    int dir_a_   = 0;
    int dir_b_   = 0;

    void set_pin(unsigned idx, int value);
    void set_channel(unsigned idx_in1, unsigned idx_in2, int fd_duty, int& dir, int16_t value);

    static void pwm_export_and_init(const char* base_path);
    static void pwm_disable(const char* base_path);
    static int  pwm_open_duty_fd(const char* base_path);
    static void pwm_write_duty(int fd, uint32_t duty_ns);
    static void write_file(const char* path, const char* value);
};
