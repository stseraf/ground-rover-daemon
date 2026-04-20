#include "param_store.hpp"

#include <climits>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "config.hpp"
#include "logger.hpp"

ParamStore::ParamStore()
{
    params_[0] = {"DRIVE_DEAD_ZONE",  static_cast<float>(Config::DRIVE_DEAD_ZONE)};
    params_[1] = {"DRIVE_SLEW_MS",   static_cast<float>(Config::DRIVE_SLEW_TIME_MS)};
    params_[2] = {"DRIVE_TRIM",      0.0f};
    params_[3] = {"CTRL_TIMEOUT_MS", static_cast<float>(Config::MC_TIMEOUT_US / 1000)};
    params_[4] = {"GPS_RAW_LOG",     0.0f};
    params_[5] = {"VIDEO_BITRATE",   5000000.0f};
    params_[6] = {"VIDEO_FPS",       30.0f};
    params_[7] = {"NET_LINK_PREF",   0.0f};  // 0=auto, 1=WiFi-prefer, 2=LTE-force
    params_[8] = {"NET_LOG_STEP_MB", 10.0f}; // 0=disabled; MB between session-traffic logs
    load(Config::PARAM_FILE);
}

int ParamStore::find_by_name(const char* id) const
{
    for (int i = 0; i < COUNT; ++i) {
        if (std::strncmp(params_[i].name, id, 16) == 0)
            return i;
    }
    return -1;
}

void ParamStore::load(const char* path)
{
    FILE* fp = std::fopen(path, "r");
    if (!fp) return; // first run — silently use defaults

    char line[64];
    while (std::fgets(line, sizeof(line), fp)) {
        char  key[16];
        float val;
        if (std::sscanf(line, "%15[^=]=%f", key, &val) == 2) {
            int idx = find_by_name(key);
            if (idx >= 0)
                params_[idx].value = val;
        }
    }
    std::fclose(fp);
    char abspath[PATH_MAX];
    const char* display = realpath(path, abspath) ? abspath : path;
    logger::line("[params] loaded from %s", display);
}

void ParamStore::save(const char* path) const
{
    FILE* fp = std::fopen(path, "w");
    if (!fp) {
        logger::line("[params] warning: cannot write to %s", path);
        return;
    }
    for (uint16_t i = 0; i < COUNT; ++i)
        std::fprintf(fp, "%s=%.6g\n", params_[i].name, params_[i].value);
    std::fclose(fp);
    char abspath[PATH_MAX];
    const char* display = realpath(path, abspath) ? abspath : path;
    logger::line("[params] saved to %s", display);
}
