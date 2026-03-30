#pragma once

#include "gps_provider.hpp"

// Reads NMEA 0183 sentences from a UART device (e.g. GY-GPS6MV2 / u-blox NEO-6M).
// Parses $GPRMC (position, velocity, validity) and $GPGGA (altitude, fix quality,
// satellite count, HDOP). Both sentences arrive once per second at 9600 baud.
// The UART fd is opened O_NONBLOCK; update() must be called every main-loop iteration.
class NmeaGpsProvider : public IGpsProvider {
public:
    explicit NmeaGpsProvider(const char* dev, int baud);
    ~NmeaGpsProvider() override;

    void update() override;
    const GpsFix& fix() const override { return fix_; }
    void set_raw_log(bool v) { raw_log_ = v; }

private:
    void on_sentence(const char* line);
    void parse_gprmc(const char* line);
    void parse_gpgga(const char* line);
    void parse_gsv(const char* line);
    void parse_gntxt(const char* line);

    int    fd_  = -1;
    GpsFix fix_{};

    static constexpr int BUF_LEN = 128;
    char   line_buf_[BUF_LEN];
    int    line_pos_ = 0;

    // Staging area: accumulate RMC fields then flush on GGA
    int32_t  rmc_lat_   = 0;
    int32_t  rmc_lon_   = 0;
    uint16_t rmc_vel_   = 0;
    uint16_t rmc_cog_   = 0;
    bool     rmc_valid_ = false;
    bool     rmc_seen_  = false;

    bool     raw_log_         = false;

    // Visible satellite tracking (from GSV sentences)
    uint8_t  vis_gps_         = 0;
    uint8_t  vis_glo_         = 0;
    uint8_t  prev_vis_total_  = 255;  // 255 = not yet logged

    // Antenna status tracking (from GNTXT sentences)
    char     prev_ant_status_[16] = {};

    // State-change tracking (for event logging)
    uint8_t  prev_sats_       = 255;  // 255 = not yet logged
    bool     prev_valid_      = false;
    uint8_t  prev_hdop_class_ = 0;   // 0=unknown, 1=good, 2=moderate, 3=poor
};
