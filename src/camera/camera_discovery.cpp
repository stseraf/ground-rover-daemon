#include "camera/camera_discovery.hpp"

#include <cstdio>
#include <cstring>
#include <cmath>

#include "logger.hpp"

// Parse libcamera-hello --list-cameras output.
//
// Expected format:
//   Available cameras
//   -----------------
//   0 : ov5647 [2592x1944 10-bit GBRG] (/base/.../ov5647@36)
//       Modes: 'SGBRG10_CSI2P' : 640x480 [58.92 fps - (16, 0)/2560x1920 crop]
//                                 1296x972 [46.34 fps - (0, 0)/2592x1944 crop]
//                                 ...

std::vector<CameraInfo> discover_cameras()
{
    std::vector<CameraInfo> cameras;

    FILE* fp = popen("rpicam-hello --list-cameras 2>&1", "r");
    if (!fp) {
        logger::line("[camera] popen(rpicam-hello) failed — no cameras");
        return cameras;
    }

    char line[256];
    CameraInfo* cur = nullptr;

    while (std::fgets(line, sizeof(line), fp)) {
        // Try camera header: "  0 : ov5647 [2592x1944 ...]"
        {
            int      idx;
            char     model[32];
            uint16_t mw, mh;
            if (std::sscanf(line, " %d : %31s [%hux%hu", &idx, model, &mw, &mh) == 4) {
                cameras.push_back({});
                cur = &cameras.back();
                cur->index = idx;
                cur->max_w = mw;
                cur->max_h = mh;
                std::strncpy(cur->model, model, sizeof(cur->model) - 1);
                cur->model[sizeof(cur->model) - 1] = '\0';
                continue;
            }
        }

        if (!cur) continue;

        // Try mode line — scan forward so the pattern is found even when
        // preceded by "Modes: 'FORMAT' : " on the first mode line.
        {
            uint16_t w, h, cw, ch;
            int16_t  cx, cy;
            float    fps;
            bool found = false;
            for (const char* p = line; *p && !found; ++p) {
                if (*p < '0' || *p > '9') continue;  // quick pre-check
                if (std::sscanf(p, "%hux%hu [%f fps - (%hd, %hd)/%hux%hu",
                                &w, &h, &fps, &cx, &cy, &cw, &ch) == 7)
                    found = true;
            }
            if (found) {
                SensorMode mode{};
                mode.width  = w;
                mode.height = h;
                mode.fps    = fps;

                // "full" = crop starts at (0,0) and covers the full sensor
                bool full = (cx == 0 && cy == 0 &&
                             cw == cur->max_w && ch == cur->max_h);
                std::snprintf(mode.name, sizeof(mode.name),
                              "%ux%u %.2ffps %s",
                              w, h, fps, full ? "full" : "crop");

                cur->modes.push_back(mode);
            }
        }
    }

    pclose(fp);

    if (cameras.empty()) {
        logger::line("[camera] no cameras found");
    } else {
        for (const auto& cam : cameras) {
            logger::line("[camera] %d: %s [%ux%u] — %zu mode(s)",
                         cam.index, cam.model, cam.max_w, cam.max_h, cam.modes.size());
            for (const auto& m : cam.modes)
                logger::line("[camera]     stream %u: %s",
                             static_cast<unsigned>(&m - cam.modes.data() + 1), m.name);
        }
    }

    return cameras;
}
