#pragma once

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <fcntl.h>
#include <unistd.h>

#include <gpiod.h>

#include "motor_driver.hpp"
#include "config.hpp"

// TB6612FNG dual H-bridge motor driver via libgpiod 2.x + hardware PWM (sysfs).
//
// Pin mapping (BCM numbering set in config.hpp):
//   Motor A (left):  PWMA → /sys/class/pwm/pwmchip0/pwm0, AIN1, AIN2
//   Motor B (right): PWMB → /sys/class/pwm/pwmchip0/pwm1, BIN1, BIN2
//   STBY: held HIGH for driver lifetime; LOW on destruction.
//
// Direction logic per channel (value in [-1000, 1000]):
//   value > 0  → IN1=1, IN2=0, duty = value/1000          (forward)
//   value < 0  → IN1=0, IN2=1, duty = abs(value)/1000     (reverse)
//   value == 0 → IN1=0, IN2=0, duty = 0                   (coast)
//
// On direction reversal the duty is zeroed first to avoid H-bridge shoot-through.

class Tb6612Driver : public IMotorDriver {
public:
    Tb6612Driver()
    {
        chip_ = gpiod_chip_open(Config::Tb6612::GPIOCHIP);
        if (!chip_) {
            std::perror("gpiod_chip_open");
            std::exit(1);
        }

        gpiod_line_config*    line_cfg = gpiod_line_config_new();
        gpiod_request_config* req_cfg  = gpiod_request_config_new();
        if (!line_cfg || !req_cfg) {
            std::fprintf(stderr, "tb6612: failed to allocate gpiod config\n");
            std::exit(1);
        }
        gpiod_request_config_set_consumer(req_cfg, "tb6612");

        add_output_pin(line_cfg, Config::Tb6612::AIN1_PIN, 0);
        add_output_pin(line_cfg, Config::Tb6612::AIN2_PIN, 0);
        add_output_pin(line_cfg, Config::Tb6612::BIN1_PIN, 0);
        add_output_pin(line_cfg, Config::Tb6612::BIN2_PIN, 0);
        add_output_pin(line_cfg, Config::Tb6612::STBY_PIN, 1);

        gpio_req_ = gpiod_chip_request_lines(chip_, req_cfg, line_cfg);

        gpiod_line_config_free(line_cfg);
        gpiod_request_config_free(req_cfg);

        if (!gpio_req_) {
            std::perror("gpiod_chip_request_lines");
            std::exit(1);
        }

        pwm_export_and_init(Config::Tb6612::PWMA_PATH);
        pwm_export_and_init(Config::Tb6612::PWMB_PATH);

        fd_pwma_ = pwm_open_duty_fd(Config::Tb6612::PWMA_PATH);
        fd_pwmb_ = pwm_open_duty_fd(Config::Tb6612::PWMB_PATH);
    }

    ~Tb6612Driver() override
    {
        stop();

        set_pin(Config::Tb6612::STBY_PIN, 0);

        if (gpio_req_) gpiod_line_request_release(gpio_req_);
        if (chip_)     gpiod_chip_close(chip_);

        pwm_disable(Config::Tb6612::PWMA_PATH);
        pwm_disable(Config::Tb6612::PWMB_PATH);

        if (fd_pwma_ >= 0) ::close(fd_pwma_);
        if (fd_pwmb_ >= 0) ::close(fd_pwmb_);
    }

    Tb6612Driver(const Tb6612Driver&)            = delete;
    Tb6612Driver& operator=(const Tb6612Driver&) = delete;

    void set(int16_t left, int16_t right) override
    {
        set_channel(Config::Tb6612::AIN1_PIN, Config::Tb6612::AIN2_PIN, fd_pwma_, dir_a_, left);
        set_channel(Config::Tb6612::BIN1_PIN, Config::Tb6612::BIN2_PIN, fd_pwmb_, dir_b_, right);
    }

    void stop() override { set(0, 0); }

private:
    gpiod_chip*         chip_     = nullptr;
    gpiod_line_request* gpio_req_ = nullptr;
    int fd_pwma_ = -1;
    int fd_pwmb_ = -1;
    int dir_a_   = 0;
    int dir_b_   = 0;

    void set_pin(unsigned offset, int value)
    {
        gpiod_line_request_set_value(gpio_req_, offset,
            value ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE);
    }

    void set_channel(unsigned in1, unsigned in2, int fd_duty, int& dir, int16_t value)
    {
        int new_dir = (value > 0) ? 1 : (value < 0) ? -1 : 0;

        if (value == 0) {
            pwm_write_duty(fd_duty, 0);
            set_pin(in1, 0);
            set_pin(in2, 0);
        } else if (new_dir != dir) {
            // Direction change: zero duty before flipping pins to avoid shoot-through
            pwm_write_duty(fd_duty, 0);
            set_pin(in1, new_dir > 0 ? 1 : 0);
            set_pin(in2, new_dir > 0 ? 0 : 1);
            pwm_write_duty(fd_duty,
                static_cast<uint32_t>(value > 0 ? value : -value)
                * Config::Tb6612::PWM_PERIOD_NS / 1000);
        } else {
            pwm_write_duty(fd_duty,
                static_cast<uint32_t>(value > 0 ? value : -value)
                * Config::Tb6612::PWM_PERIOD_NS / 1000);
        }

        dir = new_dir;
    }

    static void add_output_pin(gpiod_line_config* line_cfg, unsigned offset, int initial)
    {
        gpiod_line_settings* s = gpiod_line_settings_new();
        gpiod_line_settings_set_direction(s, GPIOD_LINE_DIRECTION_OUTPUT);
        gpiod_line_settings_set_output_value(s,
            initial ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE);
        gpiod_line_config_add_line_settings(line_cfg, &offset, 1, s);
        gpiod_line_settings_free(s);
    }

    static void pwm_export_and_init(const char* base_path)
    {
        char chip_dir[128];
        std::snprintf(chip_dir, sizeof(chip_dir), "%s", base_path);
        char* slash = std::strrchr(chip_dir, '/');
        if (slash) *slash = '\0';

        const char* index_str = base_path + std::strlen(base_path) - 1; // last char: "0" or "1"

        char exp_path[160];
        std::snprintf(exp_path, sizeof(exp_path), "%s/export", chip_dir);

        int fd = ::open(exp_path, O_WRONLY);
        if (fd < 0) {
            std::perror(exp_path);
            std::exit(1);
        }
        char export_buf[8];
        int export_len = std::snprintf(export_buf, sizeof(export_buf), "%s\n", index_str);
        ssize_t n = ::write(fd, export_buf, static_cast<size_t>(export_len));
        int write_errno = errno;
        ::close(fd);
        if (n < 0 && write_errno != EBUSY) {
            errno = write_errno;
            std::perror("pwm export write");
            std::exit(1);
        }

        ::usleep(100000); // allow kernel to create the pwmN directory

        char period_path[160], duty_path[160], enable_path[160];
        std::snprintf(period_path, sizeof(period_path), "%s/period",     base_path);
        std::snprintf(duty_path,   sizeof(duty_path),   "%s/duty_cycle", base_path);
        std::snprintf(enable_path, sizeof(enable_path), "%s/enable",     base_path);

        char period_str[16];
        std::snprintf(period_str, sizeof(period_str), "%u", Config::Tb6612::PWM_PERIOD_NS);

        write_file(duty_path,   "0");
        write_file(period_path, period_str);
        write_file(duty_path,   "0");
        write_file(enable_path, "1");
    }

    static void pwm_disable(const char* base_path)
    {
        char enable_path[160];
        std::snprintf(enable_path, sizeof(enable_path), "%s/enable", base_path);
        write_file(enable_path, "0");
    }

    static int pwm_open_duty_fd(const char* base_path)
    {
        char duty_path[160];
        std::snprintf(duty_path, sizeof(duty_path), "%s/duty_cycle", base_path);
        int fd = ::open(duty_path, O_WRONLY);
        if (fd < 0) {
            std::perror(duty_path);
            std::exit(1);
        }
        return fd;
    }

    static void pwm_write_duty(int fd, uint32_t duty_ns)
    {
        char buf[16];
        int len = std::snprintf(buf, sizeof(buf), "%u", duty_ns);
        ::lseek(fd, 0, SEEK_SET);
        ssize_t unused __attribute__((unused)) = ::write(fd, buf, static_cast<size_t>(len));
    }

    static void write_file(const char* path, const char* value)
    {
        int fd = ::open(path, O_WRONLY);
        if (fd < 0) {
            std::perror(path);
            std::exit(1);
        }
        ssize_t unused __attribute__((unused)) = ::write(fd, value, std::strlen(value));
        ::close(fd);
    }
};
