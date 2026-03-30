#pragma once

#include <cstdint>

struct GpsFix {
    bool     valid       = false;
    int32_t  lat_degE7  = 0;       // degrees * 1e7 (WGS84)
    int32_t  lon_degE7  = 0;       // degrees * 1e7 (WGS84)
    int32_t  alt_mm     = 0;       // altitude MSL, mm
    uint16_t vel_cm_s   = 0;       // ground speed, cm/s
    uint16_t cog_cdeg   = 0;       // course over ground, centidegrees (0–35999)
    uint8_t  fix_type   = 0;       // GPS_FIX_TYPE_* (0=no GPS, 1=no fix, 3=3D, 4=DGPS)
    uint8_t  satellites = 0;       // satellites visible
    uint16_t hdop_100   = 9999;    // HDOP * 100 (9999 = unknown)
    uint64_t time_usec  = 0;       // Unix epoch, microseconds (from system clock at fix)
};
