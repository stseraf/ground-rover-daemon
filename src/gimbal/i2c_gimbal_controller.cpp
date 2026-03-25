#include "i2c_gimbal_controller.hpp"
#include "config.hpp"
#include "logger.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cerrno>
#include <cstring>
#include <stdexcept>

I2cGimbalController::I2cGimbalController()
{
    fd_ = open(Config::Gimbal::I2C_BUS, O_RDWR);
    if (fd_ < 0) {
        throw std::runtime_error(std::string("gimbal: open ") +
                                 Config::Gimbal::I2C_BUS + ": " + std::strerror(errno));
    }
    if (ioctl(fd_, I2C_SLAVE, static_cast<int>(Config::Gimbal::I2C_ADDR)) < 0) {
        close(fd_);
        throw std::runtime_error(std::string("gimbal: I2C_SLAVE ioctl: ") + std::strerror(errno));
    }
    center();
    logger::line("[gimbal] I2C opened %s addr=0x%02X", Config::Gimbal::I2C_BUS, Config::Gimbal::I2C_ADDR);
}

I2cGimbalController::~I2cGimbalController()
{
    center();
    close(fd_);
}

void I2cGimbalController::set(int16_t pan, int16_t tilt)
{
    write_frame(pan, tilt);
}

void I2cGimbalController::center()
{
    write_frame(0, 0);
}

void I2cGimbalController::write_frame(int16_t pan, int16_t tilt)
{
    uint8_t buf[4];
    buf[0] = static_cast<uint8_t>((pan  >> 8) & 0xFF);
    buf[1] = static_cast<uint8_t>( pan        & 0xFF);
    buf[2] = static_cast<uint8_t>((tilt >> 8) & 0xFF);
    buf[3] = static_cast<uint8_t>( tilt       & 0xFF);
    if (write(fd_, buf, sizeof(buf)) < 0) {
        logger::line("[gimbal] write error: %s", std::strerror(errno));
    }
}
