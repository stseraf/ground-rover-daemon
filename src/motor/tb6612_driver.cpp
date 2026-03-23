#include "tb6612_driver.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/gpio.h>

#include "config.hpp"

Tb6612Driver::Tb6612Driver()
{
    chip_fd_ = ::open(Config::Tb6612::GPIOCHIP, O_RDONLY | O_CLOEXEC);
    if (chip_fd_ < 0) { std::perror(Config::Tb6612::GPIOCHIP); std::exit(1); }

    struct gpio_v2_line_request req = {};
    req.offsets[IDX_AIN1] = Config::Tb6612::AIN1_PIN;
    req.offsets[IDX_AIN2] = Config::Tb6612::AIN2_PIN;
    req.offsets[IDX_BIN1] = Config::Tb6612::BIN1_PIN;
    req.offsets[IDX_BIN2] = Config::Tb6612::BIN2_PIN;
    req.offsets[IDX_STBY] = Config::Tb6612::STBY_PIN;
    req.num_lines = 5;
    std::strncpy(req.consumer, "tb6612", sizeof(req.consumer) - 1);

    req.config.flags = GPIO_V2_LINE_FLAG_OUTPUT;
    req.config.num_attrs = 1;
    req.config.attrs[0].attr.id     = GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES;
    req.config.attrs[0].attr.values = 0; // STBY starts LOW — engage() on arm
    req.config.attrs[0].mask        = 0x1f; // apply to all 5 pins

    if (::ioctl(chip_fd_, GPIO_V2_GET_LINE_IOCTL, &req) < 0) {
        std::perror("GPIO_V2_GET_LINE_IOCTL");
        std::exit(1);
    }
    line_fd_ = req.fd;

    pwm_export_and_init(Config::Tb6612::PWMA_PATH);
    pwm_export_and_init(Config::Tb6612::PWMB_PATH);

    fd_pwma_ = pwm_open_duty_fd(Config::Tb6612::PWMA_PATH);
    fd_pwmb_ = pwm_open_duty_fd(Config::Tb6612::PWMB_PATH);
}

Tb6612Driver::~Tb6612Driver()
{
    stop();
    set_pin(IDX_STBY, 0);

    if (line_fd_ >= 0) ::close(line_fd_);
    if (chip_fd_ >= 0) ::close(chip_fd_);

    pwm_disable(Config::Tb6612::PWMA_PATH);
    pwm_disable(Config::Tb6612::PWMB_PATH);

    if (fd_pwma_ >= 0) ::close(fd_pwma_);
    if (fd_pwmb_ >= 0) ::close(fd_pwmb_);
}

void Tb6612Driver::set(int16_t left, int16_t right)
{
    set_channel(IDX_AIN1, IDX_AIN2, fd_pwma_, dir_a_, left);
    set_channel(IDX_BIN1, IDX_BIN2, fd_pwmb_, dir_b_, right);
}

void Tb6612Driver::stop()
{
    set(0, 0);
}

void Tb6612Driver::engage()
{
    set_pin(IDX_STBY, 1);
}

void Tb6612Driver::release()
{
    stop();
    set_pin(IDX_STBY, 0);
}

void Tb6612Driver::set_pin(unsigned idx, int value)
{
    struct gpio_v2_line_values v = {};
    v.mask = UINT64_C(1) << idx;
    v.bits = value ? (UINT64_C(1) << idx) : 0;
    ::ioctl(line_fd_, GPIO_V2_LINE_SET_VALUES_IOCTL, &v);
}

void Tb6612Driver::set_channel(unsigned idx_in1, unsigned idx_in2,
                                int fd_duty, int& dir, int16_t value)
{
    int new_dir = (value > 0) ? 1 : (value < 0) ? -1 : 0;

    if (value == 0) {
        pwm_write_duty(fd_duty, 0);
        set_pin(idx_in1, 0);
        set_pin(idx_in2, 0);
    } else if (new_dir != dir) {
        // Direction change: zero duty before flipping pins to avoid shoot-through
        pwm_write_duty(fd_duty, 0);
        set_pin(idx_in1, new_dir > 0 ? 1 : 0);
        set_pin(idx_in2, new_dir > 0 ? 0 : 1);
        pwm_write_duty(fd_duty,
            static_cast<uint32_t>(static_cast<uint64_t>(value > 0 ? value : -value)
            * Config::Tb6612::PWM_PERIOD_NS / 1000));
    } else {
        pwm_write_duty(fd_duty,
            static_cast<uint32_t>(static_cast<uint64_t>(value > 0 ? value : -value)
            * Config::Tb6612::PWM_PERIOD_NS / 1000));
    }

    dir = new_dir;
}

void Tb6612Driver::pwm_export_and_init(const char* base_path)
{
    char chip_dir[128];
    std::snprintf(chip_dir, sizeof(chip_dir), "%s", base_path);
    char* slash = std::strrchr(chip_dir, '/');
    if (slash) *slash = '\0';

    const char* index_str = base_path + std::strlen(base_path) - 1; // last char: "0" or "1"

    char exp_path[160];
    std::snprintf(exp_path, sizeof(exp_path), "%s/export", chip_dir);

    int fd = ::open(exp_path, O_WRONLY);
    if (fd < 0) { std::perror(exp_path); std::exit(1); }
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

void Tb6612Driver::pwm_disable(const char* base_path)
{
    char enable_path[160];
    std::snprintf(enable_path, sizeof(enable_path), "%s/enable", base_path);
    write_file(enable_path, "0");
}

int Tb6612Driver::pwm_open_duty_fd(const char* base_path)
{
    char duty_path[160];
    std::snprintf(duty_path, sizeof(duty_path), "%s/duty_cycle", base_path);
    int fd = ::open(duty_path, O_WRONLY);
    if (fd < 0) { std::perror(duty_path); std::exit(1); }
    return fd;
}

void Tb6612Driver::pwm_write_duty(int fd, uint32_t duty_ns)
{
    char buf[16];
    int len = std::snprintf(buf, sizeof(buf), "%u", duty_ns);
    ::lseek(fd, 0, SEEK_SET);
    ssize_t unused __attribute__((unused)) = ::write(fd, buf, static_cast<size_t>(len));
}

void Tb6612Driver::write_file(const char* path, const char* value)
{
    int fd = ::open(path, O_WRONLY);
    if (fd < 0) { std::perror(path); std::exit(1); }
    ssize_t unused __attribute__((unused)) = ::write(fd, value, std::strlen(value));
    ::close(fd);
}
