#include "nmea_gps_provider.hpp"

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <ctime>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "logger.hpp"
#include "config.hpp"

// ---- helpers ----------------------------------------------------------------

static speed_t baud_constant(int baud)
{
    switch (baud) {
        case 4800:   return B4800;
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        default:     return B9600;
    }
}

// Parse "DDMM.MMMM" / "DDDMM.MMMM" + direction to degE7.
// Returns 0 on invalid input.
static int32_t nmea_to_degE7(const char* field, char dir)
{
    if (!field || field[0] == '\0') return 0;
    const char* dot = strchr(field, '.');
    if (!dot) return 0;

    // Degrees occupy all characters before the last two digits before the decimal
    int pre_dot = static_cast<int>(dot - field);
    if (pre_dot < 2) return 0;
    int deg_chars = pre_dot - 2;

    double degrees = 0.0;
    for (int i = 0; i < deg_chars; i++)
        degrees = degrees * 10.0 + (field[i] - '0');

    double minutes = atof(field + deg_chars);
    double result  = degrees + minutes / 60.0;
    if (dir == 'S' || dir == 'W') result = -result;
    return static_cast<int32_t>(result * 1e7);
}

// Split a comma-separated NMEA sentence into fields (in-place, modifies buf).
// Returns number of fields found. Stops at '*' checksum delimiter.
static int split_fields(char* buf, char* fields[], int max_fields)
{
    int n = 0;
    fields[n++] = buf;
    for (char* p = buf; *p && *p != '*' && n < max_fields; p++) {
        if (*p == ',') {
            *p = '\0';
            fields[n++] = p + 1;
        }
    }
    // Null-terminate the last field at the checksum '*' if present
    for (char* p = fields[n - 1]; *p; p++) {
        if (*p == '*') { *p = '\0'; break; }
    }
    return n;
}

// ---- NmeaGpsProvider --------------------------------------------------------

NmeaGpsProvider::NmeaGpsProvider(const char* dev, int baud)
{
    fd_ = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        logger::line("[gps] open %s failed: %s", dev, strerror(errno));
        return;
    }

    struct termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
        logger::line("[gps] tcgetattr failed: %s", strerror(errno));
        close(fd_); fd_ = -1;
        return;
    }

    speed_t spd = baud_constant(baud);
    cfsetispeed(&tty, spd);
    cfsetospeed(&tty, spd);

    // 8N1, raw mode
    tty.c_cflag  = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tty.c_cflag |=  CREAD | CLOCAL;
    tty.c_iflag  = 0;
    tty.c_oflag  = 0;
    tty.c_lflag  = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        logger::line("[gps] tcsetattr failed: %s", strerror(errno));
        close(fd_); fd_ = -1;
        return;
    }

    logger::line("[gps] opened %s at %d baud", dev, baud);
}

NmeaGpsProvider::~NmeaGpsProvider()
{
    if (fd_ >= 0) close(fd_);
}

void NmeaGpsProvider::update()
{
    if (fd_ < 0) return;

    uint8_t tmp[64];
    ssize_t n = read(fd_, tmp, sizeof(tmp));
    if (n <= 0) return;  // EAGAIN or nothing available

    for (ssize_t i = 0; i < n; i++) {
        char c = static_cast<char>(tmp[i]);
        if (c == '\r') continue;
        if (c == '\n') {
            if (line_pos_ > 0) {
                line_buf_[line_pos_] = '\0';
                on_sentence(line_buf_);
            }
            line_pos_ = 0;
        } else if (c == '$') {
            // Start of a new sentence — flush any partial buffer first
            if (line_pos_ > 0) {
                line_buf_[line_pos_] = '\0';
                on_sentence(line_buf_);
            }
            line_pos_ = 0;
            line_buf_[line_pos_++] = c;
        } else if (line_pos_ < BUF_LEN - 1) {
            line_buf_[line_pos_++] = c;
        } else {
            // Overlong line — discard and reset
            line_pos_ = 0;
        }
    }
}

void NmeaGpsProvider::on_sentence(const char* line)
{
    if (raw_log_) logger::line("[gps] %s", line);
    if (strncmp(line, "$GPRMC", 6) == 0 || strncmp(line, "$GNRMC", 6) == 0)
        parse_gprmc(line);
    else if (strncmp(line, "$GPGGA", 6) == 0 || strncmp(line, "$GNGGA", 6) == 0)
        parse_gpgga(line);
    else if (strncmp(line, "$GPGSV", 6) == 0 || strncmp(line, "$GLGSV", 6) == 0)
        parse_gsv(line);
    else if (strncmp(line, "$GNTXT", 6) == 0)
        parse_gntxt(line);
}

// $GPRMC,HHMMSS.ss,A,DDMM.MMMM,N,DDDMM.MMMM,E,speed_kn,cog_deg,DDMMYY,,,*cs
void NmeaGpsProvider::parse_gprmc(const char* line)
{
    char buf[BUF_LEN];
    strncpy(buf, line, BUF_LEN - 1);
    buf[BUF_LEN - 1] = '\0';

    char* f[16];
    int   nf = split_fields(buf, f, 16);
    if (nf < 9) return;

    rmc_valid_ = (f[2][0] == 'A');
    rmc_lat_   = nmea_to_degE7(f[3], f[4][0]);
    rmc_lon_   = nmea_to_degE7(f[5], f[6][0]);

    // Speed: knots → cm/s  (1 kn = 51.4444 cm/s)
    rmc_vel_ = static_cast<uint16_t>(atof(f[7]) * 51.4444);

    // Course over ground → centidegrees
    rmc_cog_ = static_cast<uint16_t>(atof(f[8]) * 100.0);

    rmc_seen_ = true;
}

// $GPGSV / $GLGSV — track total visible satellite count from field 3, sentence 1 only
void NmeaGpsProvider::parse_gsv(const char* line)
{
    char buf[BUF_LEN];
    strncpy(buf, line, BUF_LEN - 1);
    buf[BUF_LEN - 1] = '\0';

    char* f[6];
    if (split_fields(buf, f, 6) < 4) return;

    // Only process the first sentence of each group (sentence_num == 1)
    if (atoi(f[2]) != 1) return;

    uint8_t total = static_cast<uint8_t>(atoi(f[3]));
    if (strncmp(line, "$GPGSV", 6) == 0)
        vis_gps_ = total;
    else
        vis_glo_ = total;

    uint8_t vis_total = vis_gps_ + vis_glo_;
    if (vis_total != prev_vis_total_)
        prev_vis_total_ = vis_total;
}

// $GNTXT — log antenna status changes and firmware model on boot
void NmeaGpsProvider::parse_gntxt(const char* line)
{
    char buf[BUF_LEN];
    strncpy(buf, line, BUF_LEN - 1);
    buf[BUF_LEN - 1] = '\0';

    char* f[6];
    if (split_fields(buf, f, 6) < 5) return;
    const char* text = f[4];

    if (strncmp(text, "ANTSTATUS=", 10) == 0) {
        const char* status = text + 10;
        if (strncmp(status, prev_ant_status_, 15) != 0) {
            logger::line("[gps] antenna: %s", status);
            strncpy(prev_ant_status_, status, 15);
            prev_ant_status_[15] = '\0';
        }
    } else if (strncmp(text, "MOD=", 4) == 0) {
        logger::line("[gps] module: %s", text + 4);
    }
}

// $GPGGA,HHMMSS.ss,DDMM.MMMM,N,DDDMM.MMMM,E,Q,sats,hdop,alt,M,...*cs
void NmeaGpsProvider::parse_gpgga(const char* line)
{
    char buf[BUF_LEN];
    strncpy(buf, line, BUF_LEN - 1);
    buf[BUF_LEN - 1] = '\0';

    char* f[16];
    int   nf = split_fields(buf, f, 16);
    if (nf < 10) return;

    int quality = atoi(f[6]);

    // Map GPGGA fix quality to GPS_FIX_TYPE
    uint8_t fix_type;
    switch (quality) {
        case 0:  fix_type = 1; break;  // GPS_FIX_TYPE_NO_FIX
        case 1:  fix_type = 3; break;  // GPS_FIX_TYPE_3D_FIX
        case 2:  fix_type = 4; break;  // GPS_FIX_TYPE_DGPS
        default: fix_type = 3; break;
    }

    // Merge RMC + GGA into the published fix
    fix_.fix_type   = fix_type;
    fix_.satellites = static_cast<uint8_t>(atoi(f[7]));
    fix_.hdop_100   = static_cast<uint16_t>(atof(f[8]) * 100.0);
    fix_.alt_mm     = static_cast<int32_t>(atof(f[9]) * 1000.0);

    if (rmc_seen_) {
        fix_.lat_degE7 = rmc_lat_;
        fix_.lon_degE7 = rmc_lon_;
        fix_.vel_cm_s  = rmc_vel_;
        fix_.cog_cdeg  = rmc_cog_;
        fix_.valid     = rmc_valid_ && (quality > 0);
    }

    // Timestamp: use system realtime clock when we receive a valid fix
    if (fix_.valid) {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        fix_.time_usec = static_cast<uint64_t>(ts.tv_sec) * 1000000ULL
                       + static_cast<uint64_t>(ts.tv_nsec) / 1000;
    }

    // Fix state change
    if (fix_.valid != prev_valid_) {
        if (fix_.valid)
            logger::line("[gps] fix acquired  sats=%u hdop=%.2f lat=%.7f lon=%.7f alt=%.1fm",
                         fix_.satellites, fix_.hdop_100 / 100.0,
                         fix_.lat_degE7 / 1e7, fix_.lon_degE7 / 1e7, fix_.alt_mm / 1000.0);
        else
            logger::line("[gps] fix lost");
        prev_valid_ = fix_.valid;
    }

    // Satellite count change (suppress repeated 0 during no-fix)
    if (fix_.satellites != prev_sats_ && (fix_.satellites > 0 || prev_sats_ == 255)) {
        logger::line("[gps] satellites: %u", fix_.satellites);
        prev_sats_ = fix_.satellites;
    }

    // Signal quality class change
    uint8_t hdop_class = (fix_.hdop_100 < 200) ? 1
                       : (fix_.hdop_100 < 500) ? 2 : 3;
    if (hdop_class != prev_hdop_class_) {
        const char* label = (hdop_class == 1) ? "good"
                          : (hdop_class == 2) ? "moderate" : "poor";
        logger::line("[gps] signal %s  hdop=%.2f", label, fix_.hdop_100 / 100.0);
        prev_hdop_class_ = hdop_class;
    }
}
