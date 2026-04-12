#pragma once

#include <cstdint>
#include <vector>

struct SensorMode {
    uint16_t width, height;
    float    fps;
    char     name[32];  // e.g. "1296x972 46.34fps full"
};

struct CameraInfo {
    int      index;       // 0, 1, ... (libcamera camera index)
    char     model[32];   // e.g. "ov5647"
    uint16_t max_w;       // sensor max width
    uint16_t max_h;       // sensor max height
    std::vector<SensorMode> modes;
};
