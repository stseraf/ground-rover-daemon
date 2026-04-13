#pragma once

#include <cstdint>

namespace Config {
    constexpr uint8_t  MAV_SYS_ID  = 1;
    constexpr uint8_t  MAV_COMP_ID = 1;

    constexpr uint16_t UDP_BIND_PORT         = 14550;
    constexpr uint64_t HEARTBEAT_INTERVAL_US  = 1'000'000;
    constexpr uint64_t GST_MONITOR_INTERVAL_US = 3'000'000;  // 3 s
    constexpr uint64_t LOOP_SLEEP_US         = 1'000;

    constexpr int16_t  DRIVE_AXIS_MAX        = 1000;
    constexpr int16_t  DRIVE_DEAD_ZONE       = 30;
    constexpr uint16_t DRIVE_PWM_CENTER      = 1500;
    constexpr uint16_t DRIVE_PWM_HALF_RANGE  = 400;
    constexpr uint32_t DRIVE_SLEW_TIME_MS    = 500;
    constexpr uint64_t MC_TIMEOUT_US         = 500'000; // 500 ms — stop motors if no MANUAL_CONTROL

    // TB6612FNG motor driver
    namespace Tb6612 {
        // BCM GPIO pin numbers
        constexpr unsigned AIN1_PIN = 20;  // Motor A (left)  direction bit 1
        constexpr unsigned AIN2_PIN = 16;  // Motor A (left)  direction bit 2
        constexpr unsigned BIN1_PIN = 26;  // Motor B (right) direction bit 1
        constexpr unsigned BIN2_PIN = 19;  // Motor B (right) direction bit 2
        constexpr unsigned STBY_PIN = 6;   // Standby — active-HIGH enables driver

        // Hardware PWM sysfs paths
        // Requires in /boot/firmware/config.txt:
        //   dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4
        constexpr const char* PWMA_PATH = "/sys/class/pwm/pwmchip0/pwm0"; // GPIO12, Motor A
        constexpr const char* PWMB_PATH = "/sys/class/pwm/pwmchip0/pwm1"; // GPIO13, Motor B

        // 20_000_000 ns = 50 Hz — standard RC/ESC frequency; hardware PWM holds
        // duty cycle between writes, so motors keep running between QGC updates (100 ms)
        constexpr uint32_t PWM_PERIOD_NS = 20'000'000;

        constexpr const char* GPIOCHIP = "/dev/gpiochip0";
    }

    // I2C gimbal controller (2-axis: pan=mc.x, tilt=mc.r)
    // Override I2C_BUS at compile time: make CXXFLAGS_EXTRA=-DGIMBAL_I2C_BUS='"/dev/i2c-0"'
    namespace Gimbal {
        constexpr const char* I2C_BUS  = "/dev/i2c-1";
        constexpr uint8_t     I2C_ADDR = 0x10;
        constexpr int16_t     DEAD_ZONE = 30;
    }

    // GPS module (GY-GPS6MV2 / u-blox NEO-6M, NMEA 0183 over UART)
    namespace Gps {
        constexpr const char* UART_DEV  = "/dev/serial0";
        constexpr int         BAUD_RATE = 9600;
    }

    constexpr const char* PARAM_FILE  = "params";  // relative to WorkingDirectory
    constexpr const char* QGC_IP_FILE = "qgc_ip";  // optional override: one IP per line

    // USB LTE modem (Qualcomm 05c6:90b6, enumerates as usb0)
    namespace Lte {
        constexpr uint64_t    POLL_INTERVAL_US = 5'000'000ULL; // 5 s
        constexpr const char* STATUS_HOST      = "192.168.100.1";
        constexpr uint16_t    STATUS_PORT      = 8080;         // lte_status_srv.sh TCP port
        constexpr const char* IFACE_FLAGS_PATH = "/sys/class/net/usb0/flags";
        constexpr const char* PROC_NET_DEV     = "/proc/net/dev";
    }

    // Stub telemetry values
    constexpr uint16_t DUMMY_LOAD_PERMILLE  = 500;
    constexpr uint16_t DUMMY_BATTERY_MV     = 12000;
    constexpr int16_t  DUMMY_CURRENT_CA     = -1;
    constexpr int8_t   DUMMY_BATTERY_PCT    = -1;
}
